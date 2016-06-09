/////////////////////
/*
GY33----MINI
VCC----VCC
GND----GND
1:RX---TX,send A5 01 A6 to GY-33
2:TX---RX
3:MINI_TX---FT232_RX
*/
//////////////////
unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
byte rgb[3]={0};
byte lcc[3]={0};

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);  
  delay(1);    
  Serial2.write(0XA5); 
  Serial2.write(0X81);    //8 - bit7 = 1; 1 - bit0 = 1
  Serial2.write(0X26);    //Sum of A5 and 81 (for verification)
}

void loop() {
  unsigned char i=0,sum=0;
  SerialEvent();
  if(sign)
  {   
     sign=0;
     for(i=0;i<7;i++)
      sum+=Re_buf[i]; 
     if(sum==Re_buf[i] )        //检查帧头，帧尾
     {  	       
         /*lcc[0]=(Re_buf[4]<<8)|Re_buf[5];
         lcc[1]=(Re_buf[6]<<8)|Re_buf[7];
         lcc[2]=(Re_buf[8]<<8)|Re_buf[9];
          Serial2.print("Lux =");
          Serial2.print(lcc[0]);
          Serial2.print("CT =");
          Serial2.print(lcc[1]);
          Serial2.print("Color =");
          Serial2.println(lcc[2]);*/
          
          rgb[0]=Re_buf[4];
          rgb[1]=Re_buf[5];
          rgb[2]=Re_buf[6];
          Serial.print("r:");
          Serial.print( rgb[0]);
          Serial.print(",g:");
          Serial.print( rgb[1]);
          Serial.print(",b:");
          Serial.println( rgb[2]);
           
   }
  } 
}
void SerialEvent() {
  while (Serial2.available()) {   
    Re_buf[counter]=(unsigned char)Serial2.read();
    if(counter==0&&Re_buf[0]!=0x5A) return;      // 检查帧头         
    counter++;       
    if(counter==8)                //接收到数据
    {    
       counter=0;                 //重新赋值，准备下一帧数据的接收 
       sign=1;
    }      
  }
}
