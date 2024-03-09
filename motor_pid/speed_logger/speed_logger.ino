// Define the pins for the encoder
#define ENCODER_PIN 2

#define MOTOR_A 8
#define MOTOR_B 9
#define MOTOR_PWM 10

// Variables to store encoder ticks and previous encoder state
volatile int encoderTicks = 0;
volatile int prevEncoderState = 0;

// Variables for motor speed calculation
unsigned long prevTime = 0;
float motorSpeed = 0;

// Setup function
void setup() {
  // Set encoder pins as inputs
  pinMode(ENCODER_PIN, INPUT_PULLUP);

  // Attach interrupt for encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), updateEncoder, CHANGE);

  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Timestamp,Speed");

  delay(1000);

  digitalWrite(MOTOR_A, LOW);
    digitalWrite(MOTOR_B, HIGH);
    analogWrite(MOTOR_PWM, 100);
}

// Loop function
void loop() {
  // Calculate motor speed every 100 milliseconds
  if (millis() - prevTime >= 5) {
    motorSpeed = (float)encoderTicks / 12.0 * 1000.0 / (millis() - prevTime); // Calculation based on encoder resolution and time
    Serial.print(millis());
    Serial.print(",");
    Serial.println(motorSpeed);
    prevTime = millis();
    encoderTicks = 0;
  }
}

// Function to update encoder ticks
void updateEncoder() {
    int currEncoderState = digitalRead(ENCODER_PIN);
    if (prevEncoderState == 0 && currEncoderState == 1) {
        encoderTicks++;
    }
    prevEncoderState = currEncoderState;
}
