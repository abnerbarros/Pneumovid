void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("inicializando");

}

char dado;
void loop() {
  // put your main code here, to run repeatedly:
  
if(Serial.available()){
   dado = Serial.read();
   Serial.print(dado);
}
}
