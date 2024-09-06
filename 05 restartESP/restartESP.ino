void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Restarting in 10 seconds...");
  delay(10000);
  ESP.restart();
}

void loop() {
  // put your main code here, to run repeatedly:

}
