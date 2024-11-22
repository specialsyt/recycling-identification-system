// Define the ADC pin to read from
#define ADC_PIN A0  // Analog pin A0

// Define the built-in LED pin
#define LED_BUILTIN 13

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  while (!Serial);  // Wait for the Serial Monitor to open (for M0 boards)

  // Configure the ADC pin as input
  pinMode(ADC_PIN, INPUT);

  // Set ADC resolution to 12 bits
  analogReadResolution(12);
  
  // Optional: Explicitly set the reference voltage
  analogReference(AR_DEFAULT);

  // Initialize the LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Adafruit Feather M0 Express ADC Example");
}

void loop() {
  // Read the analog value (12-bit resolution by default: 0-4095)
  int adcValue = analogRead(ADC_PIN);

  // Convert to voltage (assuming 3.3V reference voltage)
  float voltage = adcValue * (3.3 / 4095.0);

  // Print the ADC value and voltage to Serial Monitor
  Serial.print("ADC Value: ");
  Serial.print(adcValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);  // Print voltage with 3 decimal places
  Serial.println(" V");

  if (voltage < 3.0) {
      // Turn the LED on
      
      digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Small delay to avoid flooding the Serial Monitor
  delay(500);
}

