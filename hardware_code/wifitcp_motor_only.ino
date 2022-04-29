
#include <SoftwareSerial.h>

#define INITIAL_STATUS 0
#define WIFI_CONNECTED 1
#define WIFI_INFO_PRINT 2
#define WIFI_TCP_CONNECT 3
#define WIFI_TCP_SENT 4
#define WIFI_TCP_NOTSENT 5

// robot hardware params
#define MOTOR_ULIM 250
#define MOTOR_LLIM 0

#define SERIAL_FLAG 0
#define MOTOR_FLAG 1

SoftwareSerial EspSerial(2, 3); // RX, TX
int stat = INITIAL_STATUS;
String ip_addr = "";

int v_l = 0; int v_r = 0;
int lm_value = 0; int rm_value = 0;
int prev_l = 0; int prev_r = 0;
const float s_coeff = 0.5;

int right_fr = 10; int right_bk = 11;
int left_fr = 5; int left_bk = 6;
int sleep_pin = 8;
float l_coeff = 1; float r_coeff = 1;

bool tmp = 1;

unsigned long stime = 0;

void setup() {
  stat = INITIAL_STATUS;
  // set up pwm
  pinMode(left_fr, OUTPUT);
  pinMode(left_bk, OUTPUT);
  pinMode(right_fr, OUTPUT);
  pinMode(right_bk, OUTPUT);
  pinMode(sleep_pin, OUTPUT);
    
  digitalWrite(sleep_pin, HIGH);
  delay(10);
  
  if(SERIAL_FLAG) {
    Serial.begin(9600);
    delay(1000);
    Serial.println("System started");
  }
  // your esp's baud rate might be different 9600, 57600, 76800 or 115200
  EspSerial.begin(9600); 
  delay(2000);

  // check if it's already connected
  if(EspSerial.available()) {
    String s = EspSerial.readString();
    if(SERIAL_FLAG)
      Serial.print(s);
    if(s.indexOf("WIFI CONNECTED") >= 0) {
      stat = WIFI_CONNECTED;
    } else if(s.indexOf("WIFI GOT IP") >= 0) {
      stat = WIFI_CONNECTED;
    }
  } else {
//    if(SERIAL_FLAG)
//      Serial.println("Trying to send AT+RST ...");
//    EspSerial.print("AT+RST\r\n");
//    while(!EspSerial.available()) {delay(10);}
//    checkEspMsg();
//    EspSerial.print("ATE0\r\n"); 
//    while(!EspSerial.available()) {delay(10);}
//    checkEspMsg();
    stat = INITIAL_STATUS;
  }
  checkEspMsg();
  if(SERIAL_FLAG) {
    Serial.print("state: "); Serial.println(stat);
  }
  delay(1000);

  while(stat != WIFI_CONNECTED) {
    if(SERIAL_FLAG)
      Serial.println("Trying to connect to internet...");
      EspSerial.print("AT+CWJAP=\"your wifi name\",\"your wifi password\"\r\n");
    while(!EspSerial.available()) {delay(10);}

    while(EspSerial.available())
    {
      String s = EspSerial.readString();
      if(SERIAL_FLAG)
        Serial.print(s);
      if(s.indexOf("WIFI CONNECTED") >= 0) {
        stat = WIFI_CONNECTED;
      } else if(s.indexOf("WIFI GOT IP") >= 0) {
        stat = WIFI_CONNECTED;
      }
    }
    delay(100);
  }
  checkEspMsg();
  if(SERIAL_FLAG)
    Serial.println("Connected to network");
  delay(100);

  while(stat != WIFI_TCP_CONNECT) {
    EspSerial.print("AT+CIPSTART=\"TCP\",\"your IP address\",8080\r\n");
    while(!EspSerial.available()) {delay(10);}
    while(EspSerial.available()) {
      String s = EspSerial.readString();
      if(SERIAL_FLAG)
        Serial.print(s);
      if(s.indexOf("\nOK") >= 0 || s.indexOf("CONNECT") >= 0) {
        stat = WIFI_TCP_CONNECT;
      }
      if(s.indexOf("WIFI GOT IP") >= 0) {
        stat = WIFI_CONNECTED;
      }
    }
    delay(50);
  }
  checkEspMsg();
  if(SERIAL_FLAG)
    Serial.println("Connected to TCP port");
  delay(500);
}
 
void loop() {
  stime = micros();
  while(!getValueFromMsg(v_l, v_r)) { tmp = !tmp; }
  if(SERIAL_FLAG) {
    Serial.print("wait msg time: "); Serial.println(micros() - stime);
  }
  stime = micros();
  computeMotor(v_l, v_r);
  if(SERIAL_FLAG) {
    Serial.print("motor time: "); Serial.println(micros() - stime);
  }
  stime = micros();
  while(!sendMotorData()) { tmp = !tmp; }
  if(SERIAL_FLAG) {
    Serial.print("send time: "); Serial.println(micros() - stime);
  }
}

void computeMotor(int v_l, int v_r) {
  if(v_l == 0 && v_r == 0) {
    writeMotor(0, -1);
  }
  else {
    writeMotor(v_l, 0); writeMotor(v_r, 1);
  }
}

void writeMotor(int m_vel, int lr_flag) {
  // lr_flag = 0 (left), = 1 (right), = -1 (stop)
  if(m_vel > MOTOR_ULIM) {
    m_vel = MOTOR_ULIM;
  } else if(m_vel < -MOTOR_ULIM) {
    m_vel = -MOTOR_ULIM;
  } 

  if(lr_flag == 0) {
    lm_value = m_vel;
  } else {
    rm_value = m_vel;
  }
  stat = WIFI_TCP_NOTSENT;

  if(!MOTOR_FLAG) return;

  if(lr_flag == -1 && m_vel == 0) {
    int stop_pwm = 100;
    analogWrite(left_fr, stop_pwm);
    analogWrite(left_bk, stop_pwm);
    analogWrite(right_fr, stop_pwm);
    analogWrite(right_bk, stop_pwm);
    return;
  }
  
  if(lr_flag == 0 && m_vel >= 0) {
    analogWrite(left_fr, m_vel);
    analogWrite(left_bk, 0);
  } else if(lr_flag == 0 && m_vel < 0) {
    analogWrite(left_bk, - m_vel);
    analogWrite(left_fr, 0);
  } else if(lr_flag == 1 && m_vel >= 0) {
    analogWrite(right_fr, m_vel);
    analogWrite(right_bk, 0);
  } else if(lr_flag == 1 && m_vel < 0) {
    analogWrite(right_bk, - m_vel);
    analogWrite(right_fr, 0);
  }
}

bool getValueFromMsg(int& v_l, int& v_r) {
  // expect input msg format to be "[v left],[v right]", integer
  
  String value_str = "";
  String s = ""; char c;
  while(EspSerial.available()) {
    // The esp has data so display its output to the serial window 
    c = EspSerial.read(); // read the next character.
    if(c == '\n') break;
    s += c;
  }

  if(s.length() < 10) return false;
  
  if(s.indexOf("+IPD") < 0 || s.indexOf(':') < 0) {
    if(SERIAL_FLAG) {
      Serial.print("wrong msg: "); Serial.println(s);
    }
    return false;
  }

  value_str = s.substring(s.indexOf(':')+1);
  
  if(value_str.length() < 2) return false;

  int idx_com = value_str.indexOf(',');
  if(idx_com < 0) return false;

  String s_vec[2] = {value_str.substring(0, idx_com), value_str.substring(idx_com+1)};

  // pass by reference
  v_l = s_vec[0].toInt(); v_r = s_vec[1].toInt();
  return true;
}

void checkEspMsg() {
  if(EspSerial.available()) // check if the ESP module is sending a message 
  {
    while(EspSerial.available())
    {
      // The esp has data so display its output to the serial window 
      char c = EspSerial.read(); // read the next character.
      if(SERIAL_FLAG)
        Serial.write(c);
    }  
  }
}

bool sendMotorData() {
  if(stat != WIFI_TCP_NOTSENT) return false;
  EspSerial.flush();

  EspSerial.print("AT+CIPSEND=9\r\n"); EspSerial.flush();

  String s = ""; char c;
  while(!EspSerial.available()) {tmp = !tmp;}

  while(EspSerial.available()) {
    c = EspSerial.read(); // read the next character.
    s += c;
  }
//  Serial.print("send:"); Serial.println(s);

  if(s.indexOf("OK") < 0) {
    return false;
  }

  // esp is now able to send data
  char buf[10] = {};
  sprintf(buf, "%04d,%04d\n", lm_value, rm_value);
  EspSerial.print(buf);
  stat = WIFI_TCP_SENT;

  EspSerial.flush();
  return true;
}
