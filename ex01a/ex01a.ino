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
    digitalWrite(41, digitalRead(45) || r >= 5000);
    if (millis() - time >= interval) {
        int read = analogRead(A2);
        double v2 = ((double)read / 1023.0) * 5.0;
        r = (3.3 / (v2)) * 10000.0 - 10000.0 ;
        Serial.print("R="); Serial.print(r, 2); Serial.println(" Ohm");
        time += interval;
        double firstTerm = (3.3 / v2 - 1) * 0.01;
        double res = 5.0 / 1024.0; 
        double secondTerm = 10 * res * (-10000.0) * 3.3 / (v2 * v2 * r);
        double uncertainty = sqrt(firstTerm*firstTerm + secondTerm*secondTerm)*100;
        Serial.print("1 = "); Serial.println(firstTerm, 10);
        Serial.print("2 = "); Serial.println(secondTerm, 10);
        Serial.print("Accuracy=");
        Serial.print(uncertainty);
        Serial.println(" %");
    }
}
