#define POT_PIN 33
#define LED_PIN 25

#define PWM_LED_PIN 32
#define FREQUENCY 5000
#define LED_CHANNEL 0
#define RESOLUTION 8

void setup() {
  pinMode(POT_PIN, INPUT);
  ledcSetup(LED_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(PWM_LED_PIN, LED_CHANNEL);

  Serial.begin(115200);
}

void loop() {
    int pot_value = analogRead(POT_PIN);
    float voltage = pot_value * (3.3 / 4095.0);
    ledcWrite(LED_CHANNEL, voltage / 3.3 * 255);
    Serial.println(voltage);
    delay(100);
}
