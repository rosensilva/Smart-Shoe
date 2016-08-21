#include "I2Cdev.h"
#include "MPU6050.h"
#include "HX711.h"
#include <EEPROM.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
HX711 scale(A1, A0);    // parameter "gain" is ommited; the default value 128 is used by the library


int16_t ax, ay, az;
int16_t gx, gy, gz;



// accelerometer values
int t=20,s=0,v=0;
int rest = 0;
int accel_reading;
int accel_corrected;
int accel_offset = 200;
float accel_angle;
float accel_scale = 1; // set to 0.01

// gyro values
int gyro_offset = 151; // 151
int gyro_corrected;
int gyro_reading;
float gyro_rate;
float gyro_scale = 0.02; // 0.02 by default - tweak as required
float gyro_angle;
float loop_time = 0.05; // 50ms loop
float angle = 0.00; // value to hold final calculated gyro angle

// time stamp variables
int last_update;
int cycle_time;
long last_cycle = 0;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

int i=0,low_counter=0,peak_counter=0,steps=0;

void ledSteps(){
  
  pinMode(8,OUTPUT);  //The following Pins are for LED outputs of the SHOE.
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(5,INPUT);

  
  if(steps>5)digitalWrite(8,HIGH);  //For Demonstration purposes we light up each light after 5 steps.
  if(steps>10)digitalWrite(9,HIGH); 
  if(steps>15)digitalWrite(10,HIGH);
  if(steps>20)digitalWrite(11,HIGH);
  if(steps>25)digitalWrite(12,HIGH);
}


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    
   

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided 
            // by the SCALE parameter (not set yet)  

  scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided 
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");
    
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(LED_PIN, OUTPUT);
}



void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    float aax= (((ax +1300)/16000.0)*9.81 -2.71); // Scaling X axis acceleration for use in our Application 
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
     //   Serial.print("a/g:\t");
     //   Serial.print(aax); Serial.println("");
     //   Serial.print(ax); Serial.print("\t");
     //   Serial.print(ay); Serial.print("\t");
     //   Serial.print(az); Serial.print("\t");
     //   Serial.print(gx); Serial.print("\t");
     //   Serial.print(gy); Serial.print("\t");
     //   Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    //delay(1000);


    
  /*Here is the foot-step detecting algorithm  
   * Analyzed the variation between Foot step and time while walking 
   * Lower counter is a counter where it stores how many lower accelerations occur
   * Peak counter stores peak accelerations only if there are more than 10 lower counters
   * if program gets 10 lower accelerations and more than 7 peak acceleration, it shows that step is taken,according to our analyzation
   */
  if(aax<-1)low_counter++;
  if(low_counter>10){
    if(aax>1)peak_counter++;

    if(peak_counter>7){
      steps++;
      Serial.print("Current Steps : \t");
      Serial.println(steps);
      digitalWrite(5,HIGH);
      digitalWrite(4,LOW);
      delay(50);
      digitalWrite(5,LOW);
      peak_counter=0;
      low_counter=0;
    }
  }
  
  ledSteps(); //This function which is defined above uses the number of steps variable and Light up LEDs accordingly
  
  //Serial.println(low_counter);
  //Serial.println(peak_counter);
  //Serial.println("");
  //EEPROM.write(i,y);

  i++;
  
  if(digitalRead(5)){
    Serial.println("Entering Calibrating MODE ");
    calibrate();
  }  
  delay(t);//each loop is delayed by 20 ms
  
}



void get_angle(){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // accelerometer_X_Axis angle calc
    accel_reading = ax;
    accel_corrected = accel_reading - accel_offset;
    accel_corrected = map(accel_corrected, -16800, 16800, -90, 90);
    accel_corrected = constrain(accel_corrected, -90, 90);
    accel_angle = (float)(accel_corrected * accel_scale)-10;
 //   Serial.print(accel_angle);
  //  Serial.print("\t");
   
  // gyro_Y_Axis angle calc  
    gyro_reading = gy;
    gyro_corrected = (float)((gyro_reading/131) - gyro_offset);  // 131 is sensivity of gyro from data sheet
    gyro_rate = (gyro_corrected * gyro_scale) * -loop_time;      // loop_time = 0.05 ie 50ms        
    gyro_angle = angle + gyro_rate;
  /*
  // print values to serial monitor for checking 
    Serial.print(gyro_reading);
    Serial.print("\t");
    Serial.print(gyro_corrected);
    Serial.print("\t");
    Serial.print(gyro_angle);
    Serial.print("\t");
    Serial.println(" ");
    */
  // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  
//timestamp
  time_stamp();
}



void time_stamp(){
  while ((millis() - last_cycle) < 50){
  delay(1);
  }
  // once loop cycle reaches 50ms, reset timer value and continue
  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}

void calibrate(){
   
  int i;
  float weight=0,temp_weight=0,foot_len=0;
  for(i=0;i<10;i++){
  digitalWrite(8,HIGH);  //For Demonstration purposes we light up each light after 5 steps.
  digitalWrite(9,HIGH); 
  digitalWrite(10,HIGH);
  digitalWrite(11,HIGH);
  digitalWrite(12,HIGH);
  delay(100);
  
  digitalWrite(8,LOW);  //For Demonstration purposes we light up each light after 5 steps.
  digitalWrite(9,LOW); 
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  digitalWrite(12,LOW);
  delay(100);
  
  }
  for(i=0;i<3;i++){
  scale.power_up();  
  /*
  Serial.print("one reading:\t");
  
  Serial.print("\t| average:\t");
  Serial.println(, 1);
  */
  //Serial.print(scale.get_units(), 1);
  temp_weight = scale.get_units(10);
  Serial.print("Weight Samples \t");
  Serial.println(temp_weight);
  if(temp_weight<0) temp_weight = -1.0*temp_weight;
  weight = weight +temp_weight;
  
  scale.power_down();
  delay(500);// put the ADC in sleep mode
  }
  Serial.println("Weight");
  Serial.println(weight/3);
  
  int cnt=0;
  while(cnt<10){
    
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float aax= (((ax +1300)/16000.0)*9.81 -2.71); // Scaling X axis acceleration for use in our Application 
  get_angle(); //update the angle

  
  v=v+aax*t;
  s=v+(0.5)*aax*t*t;

  
  
  
  if(aax>-1.5 && aax<1.5){
   rest++;
  }
  //Serial.println(rest);

  if(rest>20 && (aax<-1.5 || aax>1.5)){
     
     //Serial.println(s);
     if(s<0)s=-1*s;  
     foot_len = s + foot_len;
    v = 0;
    s = 0;
    rest=0;
    cnt++;
    if(cnt>0)digitalWrite(8,HIGH);
    if(cnt>2)digitalWrite(9,HIGH);
    if(cnt>4)digitalWrite(10,HIGH);
    if(cnt>6)digitalWrite(11,HIGH);
    if(cnt>8)digitalWrite(12,HIGH);
  }

  }
  Serial.print("Foot Length :\t");
  Serial.println(foot_len/10);

  float final_height,final_weight,BMI;
  
  final_height = ((foot_len/10)/1280)*1.76;
  final_weight = (weight/3)/55 *75;
  BMI = final_weight/final_height*final_height;

  Serial.print("Person's Height :\t");
  Serial.println(final_height);
  Serial.print("Person's Weight :\t");
  Serial.println(final_weight);
  Serial.print("Person's BMI :\t");
  Serial.println(BMI);
  
  digitalWrite(8,LOW);  //For Demonstration purposes we light up each light after 5 steps.
  digitalWrite(9,LOW); 
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  digitalWrite(12,LOW);

    EEPROM.write(0, BMI);
    EEPROM.write(1, foot_len/100);
}

  
