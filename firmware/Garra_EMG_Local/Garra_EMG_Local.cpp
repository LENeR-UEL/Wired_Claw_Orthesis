/**
 * Wired_Claw_Orthesis - Local EMG acquisition and claw motor control.
 *
 * Target: ESP32-C3 Mini. Updated by Adler Truber.
 * The AD8232 signal is filtered and averaged to classify contraction or
 * relaxation. A state machine drives the N20 motor through an H-bridge,
 * with limit switches defining the opening and closing endpoints.
 */


#include <Arduino.h>
#include <filters.h> // libFilter, Martin Bloedorn: https://github.com/MartinBloedorn/libFilter


// ==========================================
// 1. ESP32-C3 PIN ASSIGNMENTS
// ==========================================
const int PIN_BAT     = 0;  // ADC1_CH0 - Local battery
const int potPin      = 1;  // ADC1_CH1 - Manual threshold (potentiometer)
const int PIN_EMG     = 2;  // ADC1_CH2 - AD8232 signal (GPIO2)


// Limit switches (configured with internal pull-ups)
// Stop polarities match the wiring: opening = HIGH, closing = LOW.
const int Screw_nut_  = 3;  // OPEN limit switch (screw contact)
const int Push_Claw_  = 4;  // CLOSE limit switch (push button)


// Motor control pins (H-bridge)
const int H1          = 5;  // H-bridge -> CLOSE (relaxation)
const int H2          = 6;  // H-bridge -> OPEN (contraction)


// Vibration sensor and LED
const int pinoSensor  = 7;  // TILT sensor
const int LED_PIN     = 10; // Single LED


// ==========================================
// 2. EMG FILTER CONFIGURATION
// ==========================================
// Frequencies are configured in Hz and Ts in seconds. The actual acquisition
// interval includes ADC/filter execution and a 500 us delay; it does not match
// Ts. These settings therefore do not establish measured physical cutoffs.
// Use a second-order high-pass and a third-order low-pass filter.
const float fHP = 10.0;
const float fLP = 40.0;
const float Ts  = 0.01;
Filter hp(fHP, Ts, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
Filter lp(fLP, Ts, IIR::ORDER::OD3);


// ==========================================
// 3. GLOBAL VARIABLES
// ==========================================
// MES is the mean absolute filtered signal in ADC-derived units.
// MES1..MES4 are scaled versions of the same value, not independent samples.
float MES = 0, MES1 = 0, MES2 = 0, MES3 = 0, MES4 = 0;
float batteryPercent = 0;
bool homingDone = false; // Records completion; not used to gate later transitions.


enum State { IDLE = 0, CLOSE, HOLD, OPEN, CALIBRATING };
State state = CALIBRATING; // The claw always starts by seeking its home position
State prevState = IDLE;


// Sample the tilt input at most once every 5 ms, including during EMG collection.
// Thirteen HIGH observations within 200 ms stop the motor. These are level
// observations, not rising edges. Require 500 ms without HIGH before resuming.
const int limiteHits = 13;
const int janelaTempo = 200;
int contadorHits = 0;
unsigned long tempoInicioJanela = 0;
bool vibracaoDetectada = false;
unsigned long lastTiltSample = 0;
const unsigned long tiltSampleInterval = 5;
const unsigned long vibrationQuietTime = 500;


// Motor ramp levels on the 0..255 PWM scale (approximately 10/30/60/90%).
// The 60 ms interval is a minimum condition checked once per loop; blocking
// acquisition can make the interval between applied steps longer.
const int pwmLevels[] = {25, 76, 153, 230};
const int maxRampSteps = 4;
int rampStep = 0;
unsigned long pwmTimer = 0;
const unsigned long rampDelay = 60;


unsigned long motorTimer = 0; // Most recent HIGH observation during vibration.
unsigned long lastPlotTime = 0;


/**
 * Update the vibration interlock without a blocking delay.
 * Stop both outputs immediately when the detection threshold is reached.
 * Continued HIGH observations extend the stop; LOW for 500 ms releases it.
 */
void updateVibration() {
    unsigned long tiltNow = millis();
    if (tiltNow - lastTiltSample < tiltSampleInterval) return;
    lastTiltSample = tiltNow;

    if (contadorHits > 0 && tiltNow - tempoInicioJanela >= janelaTempo) {
        contadorHits = 0;
    }

    if (digitalRead(pinoSensor) == HIGH) {
        if (contadorHits == 0) tempoInicioJanela = tiltNow;
        if (contadorHits < limiteHits) contadorHits++;
        if (contadorHits >= limiteHits) vibracaoDetectada = true;
        if (vibracaoDetectada) motorTimer = tiltNow;
    }

    if (vibracaoDetectada && tiltNow - motorTimer >= vibrationQuietTime) {
        vibracaoDetectada = false;
        contadorHits = 0;
    }

    if (vibracaoDetectada) {
        analogWrite(H1, 0);
        analogWrite(H2, 0);
    }
}


// ==========================================
// 4. EMG ACQUISITION
// ==========================================
/**
 * Compute the mean absolute filtered signal over N samples (N must be > 0).
 * Filter history persists between calls because hp and lp are global objects.
 * This function blocks: 400 delay intervals alone take 200 ms, excluding
 * ADC and processing time. Limit switches are not polled during acquisition.
 */
float readMES(int N = 400) {
    float sum = 0;
    for (int i = 0; i < N; i++) {
        updateVibration();
        float v = analogRead(PIN_EMG);
        // Filter before rectification to preserve the signed input waveform.
        v = lp.filterIn(v);
        v = hp.filterIn(v);
        sum += abs(v);
        delayMicroseconds(500);
    }
   
    return (sum / N) * 1.0; // Unity software gain.
}


// ==========================================
// 5. SETUP (INITIALIZATION)
// ==========================================
void setup() {
    Serial.begin(115200);
   
    // Set the ADC resolution for the C3
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);


    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
 
    // Establish zero PWM on both motor outputs before the startup delay.
    pinMode(H1, OUTPUT);
    pinMode(H2, OUTPUT);
    analogWrite(H1, 0);
    analogWrite(H2, 0);
 
    pinMode(Screw_nut_, INPUT_PULLUP);
    pinMode(Push_Claw_, INPUT_PULLUP);
    pinMode(pinoSensor, INPUT);

    // Wait two seconds with the motor off before executing the homing state.
    delay(2000);
}


// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
    // Time reference for this iteration; it is not refreshed after acquisition.
    unsigned long now = millis();


    // Average 50 ADC readings and reconstruct battery voltage with a factor-2
    // divider. Map 3.6..4.2 V linearly to 0..100%; this is a voltage estimate.
    // Battery level affects the LED only, not motor operation.
    uint32_t rawSum = 0;
    for(int i=0; i<50; i++) rawSum += analogRead(PIN_BAT);
    float voltage = ((rawSum / 50.0) / 4095.0) * 3.3 * 2.0;
    batteryPercent = constrain((voltage - 3.6) / 0.6 * 100.0, 0, 100);

    updateVibration();

    // Acquire a complete sample block, then derive the five voting inputs.
    MES = readMES(400);
    MES1 = MES * 0.99;
    MES2 = MES * 0.96;
    MES3 = MES * 0.93;
    MES4 = MES * 0.90;


    // ==========================================
    // 7. SENSITIVITY AND LIMIT LOGIC (CLAW)
    // ==========================================
   
    // A 12-bit potentiometer reading maps to a threshold of about 5..14.99.
    float MESth = (analogRead(potPin) * 0.00244) + 5;
    int stopOpen = digitalRead(Screw_nut_); // HIGH = fully open; LOW = within travel
    int stopClose = digitalRead(Push_Claw_); // LOW = fully closed; HIGH = within travel

    // Unanimous threshold comparison. For nonnegative MES, contraction requires
    // MES >= MESth/0.90, while relaxation requires MES <= MESth. Between these
    // thresholds neither flag is true; only other state conditions can transition.
    bool contraido = (MES >= MESth && MES1 >= MESth && MES2 >= MESth && MES3 >= MESth && MES4 >= MESth);
    bool relaxado = (MES <= MESth && MES1 <= MESth && MES2 <= MESth && MES3 <= MESth && MES4 <= MESth);

    // Restart the ramp when a state change is observed on the next iteration.
    if (state != prevState) {
        rampStep = 0;
        pwmTimer = now;
        prevState = state;
    }


    // ==========================================
    // 8. STATE MACHINE
    // ==========================================
    // Limit conditions take priority over EMG requests in the moving states.
    // A direction-change request updates the state; its motor outputs are
    // applied on the next iteration. Endpoint branches explicitly zero PWM.
    // Vibration overrides every motor state, including homing. Keep the current
    // state while stopped and reset the ramp for a gradual restart on release.
    if (vibracaoDetectada) {
        analogWrite(H1, 0);
        analogWrite(H2, 0);
        rampStep = 0;
        pwmTimer = millis();
    } else {
        switch (state) {
            case CALIBRATING:
                // Seek the closed endpoint independently of the EMG classification.
                if (stopClose == LOW) {
                    analogWrite(H1, 0); analogWrite(H2, 0);
                    homingDone = true;
                    state = IDLE;
                } else {
                    if (rampStep < maxRampSteps - 1 && (now - pwmTimer >= rampDelay)) {
                        rampStep++;
                        pwmTimer = now;
                    }
                    analogWrite(H1, pwmLevels[rampStep]); analogWrite(H2, 0);
                }
                break;


            case IDLE:
                analogWrite(H1, 0); analogWrite(H2, 0);
                // Open if a contraction is detected and opening travel remains
                if (stopOpen == LOW && contraido) state = OPEN;
                // Close if relaxation is detected and closing travel remains
                else if (stopClose == HIGH && relaxado) state = CLOSE;
                break;


            case CLOSE:
                // Stop at the closed endpoint before evaluating a reversal request.
                if (stopClose == LOW) { // Stop the motor when the closing limit switch is reached
                    analogWrite(H1, 0); analogWrite(H2, 0);
                    delay(50);
                    state = HOLD;
                    break;
                }
                if (contraido) { state = OPEN; break; } // Contraction during closing requests a direction reversal
               
                if (rampStep < maxRampSteps - 1 && (now - pwmTimer >= rampDelay)) {
                    rampStep++;
                    pwmTimer = now;
                }
                analogWrite(H1, pwmLevels[rampStep]); analogWrite(H2, 0);
                break;


            case HOLD:
                // Motor off; holding depends on mechanics, not active force control.
                analogWrite(H1, 0); analogWrite(H2, 0);
                if (contraido) state = OPEN;
                else if (stopClose == HIGH && relaxado) state = CLOSE;
                break;


            case OPEN:
                // Zero both outputs as soon as this branch detects the open endpoint.
                if (stopOpen == HIGH) {
                    analogWrite(H1, 0); analogWrite(H2, 0);
                    state = IDLE;
                    break;
                }
                if (relaxado) { state = CLOSE; break; }  // Relaxation during opening requests closing


                if (rampStep < maxRampSteps - 1 && (now - pwmTimer >= rampDelay)) {
                    rampStep++;
                    pwmTimer = now;
                }
                analogWrite(H1, 0); analogWrite(H2, pwmLevels[rampStep]);
                break;
        }
    }

    // Indicate battery status only while the motor is idle or held off.
    // At or above 20%, request a 200 ms on/off cycle; below 20%, stay on.
    // Actual LED updates follow loop timing and may be slower than this cycle.
    if (state == IDLE || state == HOLD) {
        if (batteryPercent >= 20.0) {
            digitalWrite(LED_PIN, (now % 400 < 200) ? HIGH : LOW);
        } else if (batteryPercent < 20.0) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
    } else {
        digitalWrite(LED_PIN, LOW);
    }

    // Optional Serial Plotter output at 115200 baud; disabled by default.
    /*
    if (now - lastPlotTime >= 50) {
        lastPlotTime = now;
        Serial.print("MESth:"); Serial.print(MESth); Serial.print(",");
        Serial.print("StopOpen:"); Serial.print(stopOpen); Serial.print(",");
        Serial.print("StopClose:"); Serial.print(stopClose); Serial.print(",");
        Serial.print("STATE:"); Serial.println(state);
    }
    */
}