#include <SoftwareSerial.h>
SoftwareSerial softSerial(10, 11);
char ip;
String ip_full="";
int count = 0;
char message[]="hello";

void setup()  
{ 
  Serial.begin(9600);
  
  softSerial.begin(9600);
} 
void loop()  
{ 
  delay(100);
  
  // for (int i = 0; i < strlen(message); i++) {
  //   // Perform encoding-specific operations on the character
  //   // For example, convert from ASCII to UTF-8

  //   // Example: UTF-8 encoding for ASCII characters
  //   byte utf8Char = message[i];  // No actual encoding, as ASCII is a subset of UTF-8

  //   // Send the encoded character
  //   Serial.write(utf8Char);
  // }


  String a = get_pitch_for();
  Serial.println(a);
  //delay(500);
  
  // delay(300);
  // while(true){
  //   if (softSerial.available())
  // {
  //    ip=softSerial.read();
  //   if(ip == '&'){
  //      Serial.println(ip_full);
  //      ip_full = "";
  //   }
  //   else if(ip != '&') {
      
  //     ip_full = ip_full + ip;
  //   }
   
   
  // }
  // }
}
  // delay(300);
  
  // count=count+1;
  // Serial.println(count);

String get_pitch_for(){
  //Serial.println("entry");
  while(true){
    if (softSerial.available())
  {
    ip=softSerial.read();
    if(ip == '&'){
       //Serial.println(ip_full);
       String tmp = ip_full;
       ip_full = "";
       return tmp;
    }
    else if(ip != '&') {
      //Serial.println(ip_full);
      ip_full = ip_full + ip;
    }
   
   
  }
  }
  
  
}
