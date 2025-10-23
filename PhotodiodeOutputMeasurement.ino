// Analog input pins
int sensorPin1 = A1;   // 1
int sensorPin2 = A2;   // 2
int sensorPin3 = A3;   // 3
int sensorPin4 = A4;   // 4

// Variables to store ADC reading
int ADCValue1 = 0;  // 1
int ADCValue2 = 0;  // 2
int ADCValue3 = 0;  // 3
int ADCValue4 = 0;  // 4

// Variables to store converted voltage
float voltageValue1 = 0.0;  // 1
float voltageValue2 = 0.0;  // 2
float voltageValue3 = 0.0;  // 3
float voltageValue4 = 0.0;  // 4

// Variable to store divider scale
float dividerScale = 2.5; // (Rtop + Rbot) / Rbot = 15k / 10k

// Variables to store rescaled voltage 
float actualVoltage1 = 0.0; // 1
float actualVoltage2 = 0.0; // 2
float actualVoltage3 = 0.0; // 3
float actualVoltage4 = 0.0; // 4


void setup() {
  Serial.begin(9600); // Start serial monitor
}

void loop() {
  // Voltage conversion for Photodiode 1
  ADCValue1 = analogRead(sensorPin1);      // Read raw ADC (0–1023)
  voltageValue1 = ADCValue1 * (5.0 / 1023.0);  // Convert to voltage (assuming 5V reference)
  actualVoltage1 = voltageValue1 * dividerScale; // Scale voltageValue to real voltage value

  // Voltage conversion for Photodiode 2
  ADCValue2 = analogRead(sensorPin2);      // Read raw ADC (0–1023)
  voltageValue2 = ADCValue2 * (5.0 / 1023.0);  // Convert to voltage (assuming 5V reference)
  actualVoltage2 = voltageValue2 * dividerScale; // Scale voltageValue to real voltage value

  // Voltage conversion for Photodiode 3
 /* ADCValue3 = analogRead(sensorPin3);      // Read raw ADC (0–1023)
  voltageValue3 = ADCValue3 * (5.0 / 1023.0);  // Convert to voltage (assuming 5V reference)
  actualVoltage3 = voltageValue3 * dividerScale; // Scale voltageValue to real voltage value

  // Voltage conversion for Photodiode 4
  ADCValue4 = analogRead(sensorPin4);      // Read raw ADC (0–1023)
  voltageValue4 = ADCValue4 * (5.0 / 1023.0);  // Convert to voltage (assuming 5V reference) 
  actualVoltage4 = voltageValue4 * dividerScale; // Scale voltageValue to real voltage value */
  
  // Print voltage statements
  Serial.print("P1: "); Serial.println(actualVoltage1);   // 1
  Serial.print("P2: "); Serial.println(actualVoltage2);   // 2
  /*Serial.print("P3: "); Serial.println(voltageValue3);    // 3
  Serial.print("P4: "); Serial.println(voltageValue4);    // 4
  Serial.println(); */
  delay(10000);                                           // Delay 100 ms
}