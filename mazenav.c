#include <Keypad.h>
#include <Wire.h>
//#include <Adafruit_SSD1306.h> don't know if needed?
#include <math.h> //idk
#include <MPU6050_tockn.h>

#define I2C_SLAVE_ADDR  0x04 //Define the I2C address of the Arduino Nano
// Define the pins connected to the keypad
#define ROWS 4
#define COLS 3

MPU6050 mpu6050(Wire);

const float pi = 3.14;
const float D = 5.9;
const float N = 25;

char keys[ROWS][COLS] = {
  {'3', '6', '9'},
  {'2', '5', '8'},
  {'1', '4', '7'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {2, 0, 4, 16};
byte colPins[COLS] = {18, 5, 17};

//variables to store the motor speeds and servo angle
int leftMotor_speed;
int rightMotor_speed;
int servoAngle;

// Define the dynamic array to store the input sequence
int* input_sequence;
int sequence_length = 0;

// Initialize the keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Forward function
void go_forward()
{
  leftMotor_speed = 150;
  rightMotor_speed = 150;
  servoAngle = 90;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
}

// Backward function
void go_backward()
{
  leftMotor_speed = -150;
  rightMotor_speed = -150;
  servoAngle = 90;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
}

void stop()
{ // Function transmits motor & steering values to slave arduino to make EEEBot stop moving
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 90;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
}

// Turn left function
void turn_left()
{
  leftMotor_speed = 100;
  rightMotor_speed = 250;
  servoAngle = -20;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
}

// Turn right function
void turn_right()
{
  leftMotor_speed = 250;
  rightMotor_speed = 100;
  servoAngle = 120;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
}

void turn(char *turn_type)
{
  int angle = 0;
  float angle_change = 0;

  Wire.beginTransmission(I2C_SLAVE_ADDR);
  if (turn_type == "Left")
  {                                  
    turn_left();
  }
  else if (turn_type == "Right")
  {
    turn_right();  
  }
  mpu6050.begin();
  angle = (mpu6050.getAngleZ());
  while (angle_change < 90)
  {
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    if (turn_type == "Left")
    {
      angle_change = ((mpu6050.getAngleZ()) - angle);
    }
    else if (turn_type == "Right")
    {
      angle_change = (angle - (mpu6050.getAngleZ()));
    }
  }
}

// Function to execute the input sequence
void execute_sequence()
{
  for (int i = 0; i < sequence_length; i++) {
    if (input_sequence[i] == 2)
    {
      Serial.println("Enter a value between 1 and 9:");
      while (1) {
        char key = keypad.getKey();
        if (key >= '1' && key <= '9')
        {
          go_forward();
          delay(10);
          measure_distance("forward", 10);
          delay(10);
          stop();
          break;
        }
      }
    } else if (input_sequence[i] == 4)
    {
      turn("Left");
      delay(10);
      stop();
    } else if (input_sequence[i] == 6)
    {
      turn("Right");
      delay(10);
      stop();
    } else if (input_sequence[i] == 8)
    {
  Serial.println("Enter a value between 1 and 9:");
  while (1) {
    char key = keypad.getKey();
    if (key >= '1' && key <= '9')
    {
      go_backward();
      measure_distance("backward", 10);
      delay(10);
      stop();
      break;
    }
  }
}

if (input_sequence[i] == 8) {
  Serial.println("Enter a value between 1 and 9:");
  while (1) {
    char key = keypad.getKey();
    if (key >= '1' && key <= '9')
    {
      go_forward();
      measure_distance("forward", 10);
      delay(10);
      stop();
      break;
        }
      }
    }
  }
}

void setup() {
// Initialize the serial port
input_sequence = new int[100];
Serial.begin(9600);
Wire.begin();

// Initialize the input sequence array


// Print the instructions
Serial.println("Please input a combination of keypresses using the numpad.");
Serial.println("Press 2 for forward, 4 for left, 6 for right, 8 for backward.");
Serial.println("If 2 or 8 is pressed, you will be asked to enter a value between 1 and 9.");
Serial.println("Press 5 to execute the input sequence.");
}

void loop() {
  // Get the next key from the keypad
  char key = keypad.getKey();

  // If a key was pressed
  if (key != NO_KEY)
  {
    // Convert the key to an integer
    int key_num = key - '0';
    // If the key is 2, 4, 6, or 8
    if (key_num == 2 || key_num == 4 || key_num == 6 || key_num == 8)
    {
      input_sequence[sequence_length] = key_num;
      sequence_length++;
    }
    // If the key is 5
    else if (key_num == 5)
    {
      execute_sequence();
      sequence_length = 0;
    }
    // If the key is 7
    else if (key_num == 7)
    {
      sequence_length = 0;
    }
  }
}

void measure_distance(char *direction, int distance)
{
  long enc1_count, enc2_count;
  long enc1_count_new, enc2_count_new;
  int enc_change = 0;

  Wire.requestFrom(I2C_SLAVE_ADDR,2);
  if (Wire.available () >= 2)
  {
    enc1_count = Wire.read();
    enc2_count = Wire.read();
  }
  if (direction == "backward");
  {
    while (enc_change < distance)
    {
      Wire.requestFrom(I2C_SLAVE_ADDR,2);
      if (Wire.available () >= 2)
      {
        enc1_count_new = Wire.read();
        enc2_count_new = Wire.read();
        if (enc1_count <= enc1_count_new)
        {                            
          enc_change = (enc1_count_new - enc1_count);
          enc_change = enc_change * ((D*pi)/N);
        }
        else if (enc1_count > enc1_count_new)
        {                                  
          enc_change = ((enc1_count_new + 254) - enc1_count);
          enc_change = enc_change * ((D*pi)/N);
        }    
        if (enc_change > (distance + 10))
        {
          enc_change = 0;
          enc1_count = enc1_count_new;
        }
        Serial.print("\enc1 : ");
        Serial.println(enc1_count_new);
        Serial.print("\enc1 og : ");
        Serial.println(enc1_count);
        Serial.print("\enc1 change : ");
        Serial.println(enc_change);
      }
    }
  }
  else if (direction == "forward");
  {
    while (enc_change < distance)
    {
      Wire.requestFrom(I2C_SLAVE_ADDR,2);
      if (Wire.available () >= 2)
      {
        enc1_count_new = Wire.read();
        enc2_count_new = Wire.read();
        if (enc1_count >= enc1_count_new)
        {                            
          enc_change = (enc1_count - enc1_count_new);
          enc_change = enc_change * ((D*pi)/N);
        }
        else if (enc1_count < enc1_count_new)
        {                                  
          enc_change = ((enc1_count + 254) - enc1_count_new);
          enc_change = enc_change * ((D*pi)/N);
        }    
        if (enc_change < (distance - 10)) // done
        {
          enc_change = 0;
          enc1_count = enc1_count_new;
        }
        Serial.print("\enc1 : ");
        Serial.println(enc1_count_new);
        Serial.print("\enc1 og : ");
        Serial.println(enc1_count);
        Serial.print("\enc1 change : ");
        Serial.println(enc_change);
      }
    }
  }
}

void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)
{
Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of leftMotor_speed, containing bits 16 to 9
Wire.write((byte)(leftMotor_speed & 0x000000FF));           //leftMotor_speed, containing the 8 LSB - bits 8 to 1
Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of rightMotor_speed, containing bits 16 to 9
Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1
Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));         // first byte of servoAngle, containing bits 16 to 9
Wire.write((byte)(servoAngle & 0x000000FF)); // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
Wire.endTransmission();   // stop transmitting
}