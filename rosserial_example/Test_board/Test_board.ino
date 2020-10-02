

void setup()
{
    Serial.begin(115200);
}
void loop(){
#ifdef ESP8266
  Serial.println("ESP");
#else
  Serial.println("Arduino");
#endif
  }
