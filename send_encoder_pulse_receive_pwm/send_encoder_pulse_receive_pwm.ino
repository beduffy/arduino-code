#include <Encoder.h>

boolean motor_direction = true;  // true is forward
const int m1_in1 = 10;
const int m1_in2 = 11;

const int m2_in1 = 5;
const int m2_in2 = 7;

const int en1_pwm = 9;
const int en2_pwm = 6;

long oldPositionLeft  = 0;
long oldPositionRight  = 0;
const int check_rpm_period = 20;
const int lowest_power_value = 50;

// 8423 pulses per revolution by eye
int pulses_per_revolution = 8423;

Encoder myEncLeft(2, 4);
Encoder myEncRight(3, 12);
long oldPosition  = -999;
unsigned long start_time = millis();
unsigned long last_time_for_revolution = millis();
unsigned long prev_time = start_time;

void setup() {
  //Serial.begin(9600);
  //Serial.begin(19200);
  //Serial.begin(38400);
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }
  
  Serial.println("Began serial");

  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(en1_pwm, OUTPUT);
  pinMode(en2_pwm, OUTPUT);

  analogWrite(en1_pwm, 0);
  analogWrite(en2_pwm, 0);

  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_in2, LOW);
  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_in2, LOW);

  if (motor_direction == true) {
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    digitalWrite(m2_in2, LOW);
  }
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, HIGH);
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, HIGH);
  }

  Serial.println("Started motor");

  Serial.print("l");
  Serial.print(" ");
  Serial.println(is_str("l"));
  Serial.print("100");
  Serial.print(" ");
  Serial.println(is_str("100"));

  start_time = millis();
}

void loop() {
  //boolean 
  if (Serial.available() > 0) {
    /*int value = Serial.parseInt();
    Serial.println(value);*/
    String input_string = Serial.readString();
    Serial.println(input_string);

    /*int motor_value_left = getValue(input_string, ':', 0).toInt();
    int motor_value_right = getValue(input_string, ':', 1).toInt();*/
    String left_part = getValue(input_string, ':', 0);
    String right_part = getValue(input_string, ':', 1);
    
    Serial.println("left_part:" + left_part);
    Serial.println("right_part:" + right_part);

    // if the left value is a string, only change left or right wheel (given by string)
    if (is_str(left_part)) {
      Serial.println("Specific motor chosen");
      // Change specific motor's PWM
      int motor_value = right_part.toInt();
      Serial.println("motor_value:");
      Serial.println(motor_value);
      if (left_part == "l") {
        //Serial.println("Within left wheel if statement");
        if(motor_value > 0) {
          // todo delete or duplicate
          //motor_value = constrain(motor_value, 60, 255);
          MotorCounterClockwiseLeft(abs(motor_value));
        }
        else {
          //motor_value = constrain(motor_value, -255, -60);
          MotorClockwiseLeft(abs(motor_value));
        }
      }
      else if (left_part == "r") {
        Serial.println("Within right wheel if statement");
        if(motor_value > 0) {
          MotorCounterClockwiseRight(abs(motor_value));
        }
        else {
          MotorClockwiseRight(abs(motor_value));
        }
      }
    }

    else {
      // Change both motor PWMs at the same time
      int motor_value_left_int = left_part.toInt();
      int motor_value_right_int = right_part.toInt();
  
      Serial.println("motor_value_left_int:");
      Serial.println(motor_value_left_int);
      Serial.println("motor_value_right_int:");
      Serial.println(motor_value_right_int);

      //if (motor_value_left != 0) {
      if(motor_value_left_int > 0) {
        // todo delete or duplicate
        motor_value_left_int = constrain(motor_value_left_int, lowest_power_value + 10, 255);
        MotorCounterClockwiseLeft(abs(motor_value_left_int));
      }
      else {
        motor_value_left_int = constrain(motor_value_left_int, -255, - lowest_power_value - 10);
        MotorClockwiseLeft(abs(motor_value_left_int));
      }
  
      if(motor_value_right_int > 0) {
        motor_value_right_int = constrain(motor_value_right_int, lowest_power_value + 10, 255);
        MotorCounterClockwiseRight(abs(motor_value_right_int));
      }
      else {
        motor_value_right_int = constrain(motor_value_right_int, -255, - lowest_power_value - 10);
        MotorClockwiseRight(abs(motor_value_right_int));
      }
      //}
    }
  }

  long curr_time = millis() - start_time;
  long time_difference = curr_time - prev_time;
  if (time_difference > check_rpm_period) {
    long newPositionLeft = myEncLeft.read();
    long newPositionRight = myEncRight.read();

    prev_time = curr_time;

    Serial.print("newPositionLeft: ");
    Serial.print(newPositionLeft);
    Serial.print("  ");

    Serial.print("newPositionRight: ");
    Serial.println(newPositionRight);

    /*Serial.write("newPositionLeft: ");
    Serial.write(newPositionLeft);
    Serial.write("  ");

    Serial.write("newPositionRight: ");
    Serial.write(newPositionRight);
    Serial.write("\n");*/

    oldPositionLeft = newPositionLeft;
    oldPositionRight = newPositionRight;
  }
}

void MotorClockwiseLeft(int power) {
  //if(power > lowest_power_value) {
    analogWrite(en1_pwm, power);
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);
  /*}
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, LOW);
  }*/
}

void MotorCounterClockwiseLeft(int power) {
  if(power > lowest_power_value) {
    analogWrite(en1_pwm, power);
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, HIGH);
  }
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, LOW);
    

    // todo try below?
    /*analogWrite(en1_pwm, lowest_power_value);
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);*/
  }
}

void MotorClockwiseRight(int power) {
  if(power > lowest_power_value) {
    analogWrite(en2_pwm, power);
    digitalWrite(m2_in1, HIGH);
    digitalWrite(m2_in2, LOW);
  }
  else {
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, LOW);
  }
}

void MotorCounterClockwiseRight(int power) {
  if(power > lowest_power_value) {
    analogWrite(en2_pwm, power);
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, HIGH);
  }
  else {
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, LOW);
  }
}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

boolean is_str(String input_string) {
  boolean is_str = false;
  for(byte i=0; i< input_string.length(); i++)
  {
    if(!isDigit(input_string.charAt(i))) {
      is_str = true;
      // todo could check for all to not be a digit to confirm it's a string?
      break;
    }
  }

  return is_str;
}
