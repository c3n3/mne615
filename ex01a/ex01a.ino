#include "math.h"
#include "Arduino.h"

void setup()
{
    Serial.begin(115200);
    pinMode(45, INPUT);
    pinMode(41, OUTPUT);
}

uint32_t time = millis();
uint32_t interval = 250;
double r = 0;

void loop()
{
    // Set pin 41 if pin 45 is high or if resistance is greater than 5000
    digitalWrite(41, digitalRead(45) || r >= 5000);

    // Delay by interval
    if (millis() - time >= interval) {

        // Read in to calculate voltage and resistance
        double v2 = ((double)analogRead(A2) / 1023.0) * 5.0;
        r = (3.3 / (v2)) * 10000.0 - 10000.0 ;

        // Print results
        Serial.print("R="); Serial.print(r, 2); Serial.println(" Ohm");

        // Uncertainty calculation
        double firstTerm = ((3.3 / v2 - 1)) * ( 0.01 * r + 0.05);
        double res = (5.0 / 1024.0) / 2;
        double secondTerm = res * (-10000.0) * 3.3 / (v2 * v2);

        // Print results
        Serial.print("Accuracy=");
        Serial.print(sqrt(firstTerm*firstTerm + secondTerm*secondTerm)*100/r);
        Serial.println(" %");

        // Add to interval
        time += interval;
    }
}
