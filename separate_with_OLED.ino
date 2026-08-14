#include <U8g2lib.h>
#include <Wire.h>
#define delay_time 1000
#define NEG_PIN   PA0
#define POS_PIN   PA1
#define PRE_PIN   PA2
#define LI_Voltage 3.65 //max_charge_voltage 3.65V
#define LI_Temp 60 //max_operating_temp 60 c

const float scaleFactors[4] = {2, 3, 4, 5}; //scale according to voltage divider
float Cell_value[4]; //เก็บแรงดันของแต่ละ cell
float Temp_value[4];
float Rntc_value[4];
float Voltage_value[4]; //เก็บค่าแรงดันที่วัดทั้งแผง
//float Calibrate[4] = {5.177, 5.2047, 5.1585, 5.1531}; //For connect PC
float Calibrate[4] = {3.1677, 3.1199, 3.069, 3.1055}; //For not connect PC 
float analogValues[6];

// Temperature (°C)
float tempTable[] = {
  23.0, 25.0, 25.7, 26.4, 27.0, 28.9, 29.2, 30.8,
  32.0, 33.8, 34.3, 35.8, 36.4, 37.1,
  38.0, 40.1
};
// Resistance (Ohm)
float resTable[] = {
  16459, 16312, 15645, 14672, 14623, 14354, 14252, 13859,
  13238, 13138, 13014, 12813, 12659, 12581,
  12491, 10804
};
const int tableSize = 16;

float Rntc;
float R_temporary;
float temperature;

bool negState = 1;
bool posState = 1;
bool preState = 1;


U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

byte stage = 0;  // 0 = รอปุ่ม, 1-3 = ขั้นตอนของ B0, 11-13 = ขั้นตอนของ B1
byte mode = 0; //standby, start, stop
byte select = 0; // 0 = waiting, 1 = Auto, 2 = Manual, 

float temp_f;
float TEMP_HYS = 2.0;
bool overTempFlag = false;
// filter
float temperature_filtered = 0;
bool tempInitialized = false;
// delay protection
unsigned long overTempStart = 0;
const unsigned long TEMP_DELAY = 2000; // 2 วินาที

float filterTemp(float temperature) {

  if (!tempInitialized) {
    temperature_filtered = temperature;
    tempInitialized = true;
  } else {
    temperature_filtered = 0.8 * temperature_filtered + 0.2 * temperature;
  }

  return temperature_filtered;
}

/////////////////////////////////////////////////////////////R to Temp
float getTemperatureFromR(float R_input) {

  // Serial.print("R_input = ");
  // Serial.println(R_input);

  if (R_input <= 0) return -999;

  // limit บน
  if (R_input >= resTable[0]) {
    // Serial.println("LIMIT LOW TEMP");
    return tempTable[0];
  }

  //  limit ล่าง
  if (R_input <= resTable[tableSize - 1]) {
    // Serial.println("LIMIT HIGH TEMP");
    return tempTable[tableSize - 1];
  }

  //  interpolation
  for (int i = 0; i < tableSize - 1; i++) {

    float R1 = resTable[i];
    float R2 = resTable[i + 1];

    // Serial.print("Check range: ");
    // Serial.print(R1);
    // Serial.print(" to ");
    // Serial.println(R2);

    if (R_input <= R1 && R_input >= R2) {

      float T1 = tempTable[i];
      float T2 = tempTable[i + 1];

      float T = T1 + (T2 - T1) * (R1 - R_input) / (R1 - R2);

      // Serial.print("FOUND T = ");
      // Serial.println(T);

      return T;
    }
  }

  Serial.println("NOT FOUND");
  return -999;
}

float averageArray(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

void showOverVoltage() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB08_tr);  

  int x1 = (128 - u8g2.getStrWidth("Warning")) / 2;
  u8g2.setCursor(x1, 25);
  u8g2.print("Warning");

  int x2 = (128 - u8g2.getStrWidth("Over Voltage!")) / 2;
  u8g2.setCursor(x2, 40);
  u8g2.print("Over Voltage!");

  u8g2.sendBuffer();
}

void showUnderVoltage() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB08_tr);  

  int x1 = (128 - u8g2.getStrWidth("Warning")) / 2;
  u8g2.setCursor(x1, 25);
  u8g2.print("Warning");

  int x2 = (128 - u8g2.getStrWidth("Under Voltage!")) / 2;
  u8g2.setCursor(x2, 40);
  u8g2.print("Over Voltage!");

  u8g2.sendBuffer();
}

void showOverTemp() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB08_tr);  

  int x1 = (128 - u8g2.getStrWidth("Warning")) / 2;
  u8g2.setCursor(x1, 25);
  u8g2.print("Warning");

  int x2 = (128 - u8g2.getStrWidth("Over Temperature!")) / 2;
  u8g2.setCursor(x2, 40);
  u8g2.print("Over Temperature!");

  u8g2.sendBuffer();
}


void showModeSelect() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB08_tr);  
  u8g2.setCursor(10, 15);
  u8g2.print("Mode Select");
  // ฟอนต์ตัวใหญ่ขึ้นสำหรับ Auto/Manual
  u8g2.setFont(u8g2_font_ncenB10_tr);  

  // Auto (ด้านซ้าย)
  u8g2.setCursor(10, 40);
  u8g2.print("Auto");
  // Manual (ด้านขวา)
  u8g2.setCursor(65, 40);
  u8g2.print("Manual");
  u8g2.sendBuffer();
}

void showModeAuto() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB08_tr);  
  u8g2.setCursor(10, 15);
  u8g2.print("Auto Mode");
  // ฟอนต์ตัวใหญ่ขึ้นสำหรับ Auto/Manual
  u8g2.setFont(u8g2_font_ncenB10_tr);  

  // Auto (ด้านซ้าย)
  u8g2.setCursor(10, 40);
  u8g2.print("Start");
  // Manual (ด้านขวา)
  u8g2.setCursor(70, 40);
  u8g2.print("Stop");
  u8g2.sendBuffer();
}

  //true / 1 → CLOSE
  //false / 0 → OPEN
void updatePortA() {

  // เคลียร์เฉพาะ A0-A2
  PORTA &= ~((1 << NEG_PIN) | (1 << POS_PIN) | (1 << PRE_PIN));

  // ตั้งค่าตาม state
  if (negState) PORTA |= (1 << NEG_PIN);
  if (posState) PORTA |= (1 << POS_PIN);
  if (preState) PORTA |= (1 << PRE_PIN);
}

// void updateContactorFromPORTA() {

//   negState = PORTA & (1 << 0);  // PA0
//   posState = PORTA & (1 << 1);  // PA1
//   preState = PORTA & (1 << 2);  // PA2

//   // เขียนออกไปยัง PORTL (contactor จริง)
//   updateContactors();
// }


void showVoltageInAuto() {  ////////////////////////////////////////////////แสดงผล/////////////////////////////////////////
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);

  u8g2.setCursor(0, 10);
  u8g2.print("Cell1: ");
  //u8g2.print(analogValues[0], 3);
  //u8g2.print(" and");
  // u8g2.print(Voltage_value[0], 3);
  u8g2.print(Cell_value[0], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 20);
  u8g2.print("Cell2: ");
  //u8g2.print(analogValues[1], 3);
  // u8g2.print(" and");
  // u8g2.print(Voltage_value[1], 3);
  u8g2.print(Cell_value[1], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 30);
  u8g2.print("Cell3: ");
  //u8g2.print(analogValues[2], 3);
  //u8g2.print(" and");
  // u8g2.print(Voltage_value[2], 3);
  u8g2.print(Cell_value[2], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 40);
  u8g2.print("Cell4: ");
  //u8g2.print(analogValues[3], 3);
  //u8g2.print(" and");
  // u8g2.print(Voltage_value[3], 3);
  u8g2.print(Cell_value[3], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 48);
  u8g2.print("Temperature: ");
  //u8g2.print(R_temporary, 3);
  u8g2.print(temperature, 3);
  u8g2.print(" C");

  /* ====== ฝั่งขวา : Contactor Status ====== */
  u8g2.setCursor(78, 10);
  u8g2.print("NEG:");
  u8g2.print(negState ? "Open" : "Close");

  u8g2.setCursor(78, 20);
  u8g2.print("POS:");
  u8g2.print(posState ? "Open" : "Close");

  u8g2.setCursor(78, 30);
  u8g2.print("PRE:");
  u8g2.print(preState ? "Open" : "Close");

  // =================== ด้านล่าง ================

  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.setCursor(0, 60);
  u8g2.print("<<< START");

  u8g2.setCursor(60, 60);
  u8g2.print("STOP >>>");

  u8g2.sendBuffer();
}


void showVoltage() {  ////////////////////////////////////////////////แสดงผล/////////////////////////////////////////
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);

  u8g2.setCursor(0, 10);
  u8g2.print("Cell1: ");
  //u8g2.print(Voltage_value[0], 3);
  u8g2.print(Cell_value[0], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 20);
  u8g2.print("Cell2: ");
  //u8g2.print(Voltage_value[1], 3);
  u8g2.print(Cell_value[1], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 30);
  u8g2.print("Cell3: ");
  //u8g2.print(Voltage_value[2], 3);
  u8g2.print(Cell_value[2], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 40);
  u8g2.print("Cell4: ");
  //u8g2.print(Voltage_value[3], 3);
  u8g2.print(Cell_value[3], 3);
  u8g2.print(" V");

  u8g2.setCursor(0, 48);
  u8g2.print("Temperature: ");
  //u8g2.print(R_temporary, 3);
  u8g2.print(temperature, 3);
  u8g2.print(" C");

  /* ====== ฝั่งขวา : Contactor Status ====== */
  u8g2.setCursor(78, 10);
  u8g2.print("NEG:");
  u8g2.print(negState ? "Open" : "Close");

  u8g2.setCursor(78, 20);
  u8g2.print("POS:");
  u8g2.print(posState ? "Open" : "Close");

  u8g2.setCursor(78, 30);
  u8g2.print("PRE:");
  u8g2.print(preState ? "Open" : "Close");

  u8g2.sendBuffer();
}

void clearSerial() {
  u8g2.clearBuffer();           // ล้าง buffer
  u8g2.setDrawColor(0);         // 0 = BLACK
  u8g2.drawBox(0, 0, 128, 64);  // วาดกรอบเต็มหน้าจอ
  u8g2.sendBuffer();             // ส่งไปจอ
  u8g2.setDrawColor(1);         // 1 = WHITE (กลับมาเป็นสีปกติ)
}


void setup() {
  DDRA = 0xFF;   // กำหนด PORTA เป็น Output
  DDRB = 0x00;   // กำหนด PORTB เป็น Input
  DDRL = 0x00;   // กำหนด PORTL เป็น Input
  PORTB = 0xFF;   // เปิด Pull-up ให้ปุ่ม B0,B1
  PORTL = 0xFF;
  PORTA = 0xFF; //active low

  Serial.begin(9600); //serial plot
  u8g2.begin(); //OLED
}

void loop() {
  /////////////////////////////เลือก auto/ manual
  while(select == 0){
    showModeSelect();
    //delay(10);

    if ((PINB & (1 << PB2)) == 0) { // ปุ่ม B2 Auto
      delay(50);
      if ((PINB & (1 << PB2)) == 0) {
        while ((PINB & (1 << PB2)) == 0); // รอปล่อย
        delay(50);
        clearSerial();
        mode = 0;
        stage = 0;
        select = 1;
      }
    }
    else if ((PINB & (1 << PB3)) == 0) { // ปุ่ม B3 Manual
      delay(50);
      if ((PINB & (1 << PB3)) == 0) {
        while ((PINB & (1 << PB3)) == 0); // รอปล่อย
        delay(50);
        clearSerial();
        mode = 01;
        stage = 0;
        select = 2;
      }
    }
  }

  /////////////////////////////////////////////////mode Manual///////////////////////////////////////////////////////////////
  while (mode == 01){
    
    for (int i = 0; i < 4; i++){
      analogValues[i]  = analogRead(A0 + i);
      Voltage_value[i] = analogValues[i] * (Calibrate[i]/1023.0) * scaleFactors[i];

      analogValues[4]  = analogRead(A4); ///////////////////////////////// Temperature Read /////////////////////////////////
      Temp_value[i] = analogValues[4] * (5/1023.0);
      //Serial.print(Temp_value[i], 3);

      Rntc = 46960 * (analogValues[4] * (5/1023.0))/(12+(analogValues[4] * (5/1023.0)))*1.1917; //Rfixed = 47kohms
      //Un-sub:k=1.1917,  Usb:k=1.6897

      Rntc_value[i] = Rntc;
      R_temporary = averageArray(Rntc_value,4);
      temperature = getTemperatureFromR(R_temporary);
      temp_f = filterTemp(temperature);
      if (!overTempFlag && temp_f >= LI_Temp) {
        overTempFlag = true;
      }
      else if (overTempFlag && temp_f <= (LI_Temp - TEMP_HYS)) {
        overTempFlag = false;
      }

      Serial.println(temperature, 3);

      if (i == 0){
        Cell_value[i] = Voltage_value[i];  // cell แรกไม่มีค่าก่อนหน้า
      }
      else{
        Cell_value[i] = Voltage_value[i] - Voltage_value[i - 1];
      }


      if (Cell_value[i] > LI_Voltage){ /*///////////////////////////// Emeregency Break เหลือตัดอุณหภูมิ///////////////////////////////*/
        showOverVoltage();
        delay(3000);
        PORTA = 0xff; //stage = 0
        select = 0; // กลับไปเลือกโหมดอีกครั้ง
        mode = 0;
        preState = 0;
        negState = 0;
        updatePortA();
        delay(delay_time);
        posState = 1;
        updatePortA();
        delay(delay_time);
        negState = 1;
        posState = 1;
        preState = 1;
        updatePortA();

        break;
        
      }

      if (temperature > LI_Temp){ /*///////////////////////////// Emeregency Break เหลือตัดอุณหภูมิ///////////////////////////////*/
        showOverTemp();
        delay(3000);
        PORTA = 0xff; //stage = 0
        select = 0; // กลับไปเลือกโหมดอีกครั้ง
        mode = 0;
        negState = 1;
        posState = 1;
        preState = 1;
        break;
      }

      // if (overTempFlag) {

      //   if (overTempStart == 0) {
      //    overTempStart = millis();
      //   }

      //   if (millis() - overTempStart >= TEMP_DELAY) {
      //     showOverTemp();
      //     delay(3000);
      //     PORTA = 0xff;
      //     select = 0;
      //     mode = 0;
      //     negState = 1;
      //     posState = 1;
      //     preState = 1;
      //     break;
      //     temperature = 25;

      //   }

      // } else {
      //     overTempStart = 0;
      // }

      




      Serial.print("Voltage cell ");
      Serial.print(i+1);
      Serial.print(" is: ");
      Serial.print(Voltage_value[i], 3);
      Serial.println(" V.");

      showVoltage(); // แสดงค่า OLED

      //delay(1000); //ตรงนี้ถ้าปรับไปเยอะจะมีปัญหาตอนออกลูป
    }

    //=======================รับปุ่มกด Negative
    if ((PINL & (1 << PL0)) == 0) {
      delay(50);
      if ((PINL & (1 << PL0)) == 0) {
        while ((PINL & (1 << PL0)) == 0); // รอปล่อย
        delay(50);
        negState = !negState;
      }
    }
    //=======================รับปุ่มกด Positive
    if ((PINL & (1 << PL1)) == 0) {
      delay(50);
      if ((PINL & (1 << PL1)) == 0) {
        while ((PINL & (1 << PL1)) == 0); // รอปล่อย
        delay(50);
        posState = !posState;
      }
    }
    //=======================รับปุ่มกด Pre-charge
    if ((PINL & (1 << PL2)) == 0) {
      delay(50);
      if ((PINL & (1 << PL2)) == 0) {
        while ((PINL & (1 << PL2)) == 0); // รอปล่อย
        delay(50);
        preState = !preState;
      }
    }

    updatePortA();

    if ((PINL & (1 << PL3)) == 0) { ////////////////////Back///////////////////
      delay(50);
      if ((PINL & (1 << PL3)) == 0) {
        while ((PINL & (1 << PL3)) == 0); // รอปล่อย
        delay(50);
        PORTA = 0xff; //stage = 0
        select = 0; // กลับไปเลือกโหมดอีกครั้ง
        mode = 0;
        negState = 1;
        posState = 1;
        preState = 1;
        break;
      }
    }
    
  }







  /////////////////////////////////////// "Auto mode" ///////////////////////////////
  // if (mode ==0 && stage == 0 ){
  //   PORTA = 0xff;
  //   showModeAuto();
  // }
  // โหมดรอปุ่ม
  while (mode ==0 && select == 1){

    for (int i = 0; i < 4; i++){
      analogValues[i]  = analogRead(A0 + i);
      Voltage_value[i] = analogValues[i] * (Calibrate[i]/1023.0) * scaleFactors[i];

      analogValues[4]  = analogRead(A4); ///////////////////////////////// Temperature Read /////////////////////////////////
      Temp_value[i] = analogValues[4] * (5/1023.0);
      ///////////plot/////////////////////
      //Serial.print(Temp_value[i], 3);

      Rntc = 46960 * (analogValues[4] * (5/1023.0))/(12+(analogValues[4] * (5/1023.0)))*1.1917; //Rfixed = 47kohms
      //Un-sub:k=1.1917,  Usb:k=1.6897
      
      Rntc_value[i] = Rntc;
      R_temporary = averageArray(Rntc_value,4);
      //Serial.println(R_temporary, 3);
      temperature = getTemperatureFromR(R_temporary);

      if (i == 0){
        Cell_value[i] = Voltage_value[i];  // cell แรกไม่มีค่าก่อนหน้า
      }
      else{
        Cell_value[i] = Voltage_value[i] - Voltage_value[i - 1];
      }
      

      // if (Cell_value[i] > LI_Voltage || Cell_value[i] < 0){ /*///////////////////////////// Emeregency Break เหลือตัดอุณหภูมิ///////////////////////////////*/
      //   if(Cell_value[i] > LI_Voltage){
      //     showOverVoltage();
      //   }
      //   if(Cell_value[i] < 0){
      //     showUnderVoltage();
      //   }
      //   delay(3000);
      //   PORTA = 0xff; //stage = 0
      //   select = 0; // กลับไปเลือกโหมดอีกครั้ง
      //   mode = 0;

      //   preState = 0;
      //   negState = 0;
      //   updatePortA();
      //   delay(delay_time);
      //   posState = 1;
      //   updatePortA();
      //   delay(delay_time);
      //   negState = 1;
      //   posState = 1;
      //   preState = 1;
      //   updatePortA();
      //   break;
      // }
      if (Cell_value[i] > LI_Voltage){ /*///////////////////////////// Emeregency Break เหลือตัดอุณหภูมิ///////////////////////////////*/
        showOverVoltage();
        delay(3000);
        PORTA = 0xff; //stage = 0
        select = 0; // กลับไปเลือกโหมดอีกครั้ง
        mode = 0;
        preState = 0;
        negState = 0;
        updatePortA();
        delay(delay_time);
        posState = 1;
        updatePortA();
        delay(delay_time);
        negState = 1;
        posState = 1;
        preState = 1;
        updatePortA();

        break;
        
      }

      if (temperature > LI_Temp){ /*///////////////////////////// Emeregency Break เหลือตัดอุณหภูมิ///////////////////////////////*/
        showOverTemp();
        delay(3000);
        PORTA = 0xff; //stage = 0
        select = 0; // กลับไปเลือกโหมดอีกครั้ง
        mode = 0;
        negState = 1;
        posState = 1;
        preState = 1;
        break;
      }

      //Serial.print("Voltage cell ");
      //Serial.print(i+1);
      //Serial.print(" is: ");
      //Serial.print(Voltage_value[i], 3);
      //Serial.println(" V.");

      //showVoltage(); //แสดงค่าแต่ละcell

      //Serial.print(analogValues[3], 3); //เก็บไว้ calibrate
      //Serial.print(Voltage_value[3], 3); 
      //delay(1000); //ตรงนี้ถ้าปรับไปเยอะจะมีปัญหาตอนออกลูป
      //รันรูปรับค่า 4 input เข้า array analogValues[i]
    }

    showVoltageInAuto();
    
    // Serial.print(temperature, 3);
    // Serial.print(" ");
    delay(1000);
    Serial.print(Cell_value[0], 3);
    Serial.print(" ");
    Serial.print(Cell_value[1], 3);
    Serial.print(" ");
    Serial.print(Cell_value[2], 3);
    Serial.print(" ");
    Serial.println(Cell_value[3], 3);
    
    // Serial.print(Voltage_value[0], 3);
    // Serial.print(" ");
    // Serial.print(Voltage_value[1], 3);
    // Serial.print(" ");
    // Serial.print(Voltage_value[2], 3);
    // Serial.print(" ");  
    // Serial.println(Voltage_value[3], 3);

    // Serial.print(" ");
    // Serial.print(analogValues[0], 3);
    // Serial.print(" ");
    // Serial.print(analogValues[1], 3);
    // Serial.print(" ");
    // Serial.print(analogValues[2], 3);
    // Serial.print(" ");
    // Serial.println(analogValues[3], 3);
    


    
    if (stage == 0 || stage == 3 ) { ///////////////////////////////////Stat/////////////////////
      if ((PINB & (1 << PB0)) == 0) { // ปุ่ม B0 Start
        delay(50);
        if ((PINB & (1 << PB0)) == 0) {
          while ((PINB & (1 << PB0)) == 0); // รอปล่อย
          delay(50);
          //mode = 1;
          //stage = 1;

          PORTA = 0b11111010;
          negState = 0;
          posState = 1;
          preState = 0;
          updatePortA();
          showVoltage();
          delay(delay_time);

          PORTA = 0x00;
          negState = 0;
          posState = 0;
          preState = 0;
          updatePortA();
          showVoltage();
          delay(delay_time);

          PORTA = 0b11111100;
          negState = 0;
          posState = 0;
          preState = 1;
          updatePortA();
          showVoltage();
          delay(delay_time);
        }
      }



     if ((PINB & (1 << PB1)) == 0) { ///////////////////////////////// ปุ่ม B1 Stop ///////////////////
        delay(50);
        if ((PINB & (1 << PB1)) == 0) {
          while ((PINB & (1 << PB1)) == 0); // รอปล่อย
          delay(50);
          //mode = 2;

          PORTA = 0x00;
          negState = 0;
          posState = 0;
          preState = 0;
          updatePortA();
          showVoltage();
          delay(delay_time);

          PORTA = 0b11111010;
          negState = 0;
          posState = 1;
          preState = 0;
          updatePortA();
          showVoltage();
          delay(delay_time);

          PORTA = 0xFF;
          negState = 1;
          posState = 1;
          preState = 1;
          updatePortA();
          showVoltage();
          delay(delay_time);

          mode = 0;
          select = 0;
        }
      }

      if ((PINL & (1 << PL3)) == 0) { ////////////////////Back///////////////////
        delay(50);
        if ((PINL & (1 << PL3)) == 0) {
          while ((PINL & (1 << PL3)) == 0); // รอปล่อย
          delay(50);
          PORTA = 0xFF; //stage = 0
          select = 0; // กลับไปเลือกโหมดอีกครั้ง
          mode = 0;
          negState = 1;
          posState = 1;
          preState = 1;
          break;
        }
      }
    }
  }
  delay(500);

  
}



