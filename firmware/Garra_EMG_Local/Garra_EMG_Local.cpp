// Claw Controller with Local EMG (No BLE)
// Updated by Adler Truber - ESP32-C3 Mini Version
// Signal Acquisition + State Machine


#include <Arduino.h>
#include <filters.h> // libFilter, Martin Bloedorn: https://github.com/MartinBloedorn/libFilter


// ==========================================
// 1. ESP32-C3 PIN ASSIGNMENTS
// ==========================================
const int PIN_BAT     = 0;  // ADC1_CH0 - Local battery
const int potPin      = 1;  // ADC1_CH1 - Manual threshold (potentiometer)
const int PIN_EMG     = 2;  // ADC1_CH2 - AD8232 signal (GPIO2)


// Limit switches (configured with internal pull-ups)
// Original receiver polarities: opening stops at HIGH; closing stops at LOW.
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
const float fHP = 10.0;
const float fLP = 40.0;
const float Ts  = 0.01;
Filter hp(fHP, Ts, IIR::ORDER::OD3, IIR::TYPE::HIGHPASS);
Filter lp(fLP, Ts, IIR::ORDER::OD3);


// ==========================================
// 3. GLOBAL VARIABLES
// ==========================================
float MES = 0, MES1 = 0, MES2 = 0, MES3 = 0, MES4 = 0;
float batteryPercent = 0;
bool homingDone = false;


enum State { IDLE = 0, CLOSE, HOLD, OPEN, CALIBRATING };
State state = CALIBRATING; // The claw always starts by seeking its home position
State prevState = IDLE;


// Vibration Detection - Tilt
const int limiteHits = 13;
const int janelaTempo = 200;
int contadorHits = 0;
unsigned long tempoInicioJanela = 0;
bool vibracaoDetectada = false;


// Soft Start Variables (PWM)
const int pwmLevels[] = {25, 76, 153, 230};
const int maxRampSteps = 4;
int rampStep = 0;
unsigned long pwmTimer = 0;
const unsigned long rampDelay = 60;


unsigned long motorTimer = 0;
unsigned long lastPlotTime = 0;


// ==========================================
// 4. ORIGINAL EMG ACQUISITION FUNCTION
// ==========================================
float readMES(int N = 400) {
    float sum = 0;
    for (int i = 0; i < N; i++) {
        float v = analogRead(PIN_EMG);
        v = lp.filterIn(v);
        v = hp.filterIn(v);
        sum += abs(v);
        delayMicroseconds(500);
    }
   
    return (sum / N) * 1.0;
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
 
    pinMode(H1, OUTPUT);
    pinMode(H2, OUTPUT);
    analogWrite(H1, 0);
    analogWrite(H2, 0);
 
    pinMode(Screw_nut_, INPUT_PULLUP);
    pinMode(Push_Claw_, INPUT_PULLUP);
    pinMode(pinoSensor, INPUT);

    // Initial delay equivalent to the receiver's 2-second post-connection delay.
    // Both motor outputs are already set to zero PWM.
    delay(2000);
}


// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
    unsigned long now = millis();


    // --- BATTERY READING ---
    uint32_t rawSum = 0;
    for(int i=0; i<50; i++) rawSum += analogRead(PIN_BAT);
    float voltage = ((rawSum / 50.0) / 4095.0) * 3.3 * 2.0;
    batteryPercent = constrain((voltage - 3.6) / 0.6 * 100.0, 0, 100);

    // --- TILT SENSOR ---
    int leituraTilt = digitalRead(pinoSensor);
    if (leituraTilt == HIGH) {
        if (contadorHits == 0) tempoInicioJanela = now;
        contadorHits++;
        delay(5);
    }
    if (now - tempoInicioJanela > janelaTempo) {
        if (contadorHits >= limiteHits) {
            vibracaoDetectada = true;
            motorTimer = now;
        } else if (now - tempoInicioJanela > (janelaTempo + 500)) {
            vibracaoDetectada = false;
        }
        contadorHits = 0;
    }


    // --- EMG PROCESSING ---
    MES = readMES(400);
    MES1 = MES * 0.99;
    MES2 = MES * 0.96;
    MES3 = MES * 0.93;
    MES4 = MES * 0.90;


    // ==========================================
    // 7. SENSITIVITY AND LIMIT LOGIC (CLAW)
    // ==========================================
   
    float MESth = (analogRead(potPin) * 0.00244) + 5;
    int stopOpen = digitalRead(Screw_nut_); // HIGH = fully open; LOW = within travel
    int stopClose = digitalRead(Push_Claw_); // LOW = fully closed; HIGH = within travel

    // Original voting rule: all five values derived from MES must agree.
    bool contraido = (MES >= MESth && MES1 >= MESth && MES2 >= MESth && MES3 >= MESth && MES4 >= MESth);
    bool relaxado = (MES <= MESth && MES1 <= MESth && MES2 <= MESth && MES3 <= MESth && MES4 <= MESth);

    // Reset the PWM ramp
    if (state != prevState) {
        rampStep = 0;
        pwmTimer = now;
        prevState = state;
    }


    // ==========================================
    // 8. STATE MACHINE
    // ==========================================
    switch (state) {
        case CALIBRATING:
            // At startup, close until the closing limit switch reads LOW.
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
            analogWrite(H1, 0); analogWrite(H2, 0);
            if (contraido) state = OPEN;
            else if (stopClose == HIGH && relaxado) state = CLOSE;
            break;


        case OPEN:
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


    // Local battery status LED.
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

    // Original debug output disabled; uncomment to use the Serial Plotter
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