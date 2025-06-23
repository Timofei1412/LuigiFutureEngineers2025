#include <Arduino.h>
#include "HardwareSerial.h"
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#include "Defines.h"


// extern BluetoothSerial SerialBT;

Servo serva_claw_forward;
Servo serva_upper_forward;
Servo serva_claw_back;
Servo serva_upper_back;

#define ll long long

int in_1 = 17, in_2 = 22, in_3 = 19, in_4 = 18, but = 32, line_sensor_l = 4, line_sensor_r = 14, sharp = 13;  //порты
int serva_delay = 450, x = 1, time_break = 10, time_cross = 470;                                              //время
// bool go = false, flag = false; //флаги
int speeds = 150, turn_speeds = 100;  //скорости

int min_r = 49, max_r = 1190, min_l = 62, max_l = 1474, grey = 60, grey_turn = 50, grey_turn_black = 30, black_grey = 90;  //датчики линии
//int min_l = 4000, max_l = 0, min_r = 4000, max_r = 0, grey = 40; //датчики линии калибровка

int err, u, l, r;  //регулятор линии
int s_left, s_right;
ll timer;
float kp = 0.25, kp_fast = 0.8;  //коэффиценты

int start_speed = 50, final_speed = 230, a_uskorenie = 3, a_tormozenie = 3, local_speed = 0;
ll last_change_speed = 0;


void close_forward() {
  serva_claw_forward.write(29);
  delay(serva_delay);
}

void open_forward() {
  serva_claw_forward.write(90);
  delay(serva_delay);
}

void down_forward() {
  serva_upper_forward.write(25);
  delay(serva_delay);
}

void up_forward() {
  serva_upper_forward.write(115);
  delay(serva_delay);
}

void close_back() {
  serva_claw_back.write(47);
  delay(serva_delay);
}

void open_back() {
  serva_claw_back.write(110);
  delay(serva_delay);
}

void down_back() {
  serva_upper_back.write(25);
  delay(serva_delay);
}

void up_back() {
  serva_upper_back.write(115);
  delay(serva_delay);
}

void servo_spusk() {
  serva_upper_back.write(15);
  delay(serva_delay);
}

void servo_podem() {
  serva_upper_forward.write(15);
  delay(serva_delay);
}

void move(int left, int right) {
  if (left < 0) {
    delay(x);
    analogWrite(in_1, 255);
    delay(x);
    analogWrite(in_2, constrain(255 - abs(left), 0, 255));
  } else {
    delay(x);
    analogWrite(in_2, 255);
    delay(x);
    analogWrite(in_1, constrain(255 - abs(left), 0, 255));
  }

  if (right > 0) {
    delay(x);
    analogWrite(in_3, 255);
    delay(x);
    analogWrite(in_4, constrain(255 - abs(right), 0, 255));
  } else {
    delay(x);
    analogWrite(in_4, 255);
    delay(x);
    analogWrite(in_3, constrain(255 - abs(right), 0, 255));
  }
}

void motor_stop() {
  delay(x);
  analogWrite(in_1, 0);
  delay(x);
  analogWrite(in_2, 0);
  delay(x);
  analogWrite(in_3, 0);
  delay(x);
  analogWrite(in_4, 0);
}

void motor_break() {
  move(-255, -255);
  delay(time_break);
  move(0, 0);
}

void move_time(int data, int dir) {
  ll t = millis();
  while (millis() - t < data) {
    move(speeds * dir, speeds * dir);
  }
  if (dir) {
    motor_break();
  } else {
    move(255, 255);
    delay(time_break);
    move(0, 0);
  }
}

int read_l() {
  return map(constrain(int(analogRead(line_sensor_l)), min_l, max_l), min_l, max_l, 0, 100);
}

int read_r() {
  return map(constrain(int(analogRead(line_sensor_r)), min_r, max_r), min_r, max_r, 0, 100);
}

void sink(int black = 1) {
  ll t = millis();
  while (millis() - t < 450) {
    err = read_l() - read_r();
    u = err * 0.3;
    move(-u * black, u * black);
  }
  move(0, 0);
}

void follow_line(int speed, int black = 1) {
  l = read_l();
  r = read_r();
  err = l - r;
  u = err * kp;
  move(speed - (u * black), speed + (u * black));
}

void follow_line_fast(int speed, int black = 1) {
  l = read_l();
  r = read_r();
  err = l - r;
  u = err * kp_fast;
  move(speed - (u * black), speed + (u * black));
}

void move_to_cross() {
  move(140, 140);
  delay(time_cross);
  motor_break();
}

void follow_line_to_cross(int cross = 3, int black = 1) {
  timer = millis();
  while (millis() - timer < 300) {
    follow_line(speeds, black);
  }
  s_left = read_l();
  s_right = read_r();
  if (black == 1) {
    while (s_left < grey or s_right < grey) {
      follow_line(speeds, 1);
      s_left = read_l();
      s_right = read_r();
    }
  } else {
    while (s_left > black_grey or s_right > black_grey) {
      follow_line(speeds, -1);
      s_left = read_l();
      s_right = read_r();
      // delay(10);
      // SerialBT.print(read_l());
      // SerialBT.print(' ');
      // delay(10);
      // SerialBT.println(read_r());
    }
  }
  if (cross == 1) {
    move_to_cross();
  } else if (cross == 2) {
    motor_break();
  } else {
    move(0, 0);
  }
}

void follow_cross(int data, int black = 1) {
  for (int i = 0; i < data - 1; ++i) {
    follow_line_to_cross(3, black);
  }
  follow_line_to_cross(1, black);
}

void follow_line_time(int b, int time, int black = 1) {
  ll t = millis();
  while (millis() - t < time) {
    follow_line(speeds, black);
  }
  if (b == 1) {
    move_to_cross();
  } else if (b == 2) {
    motor_break();
  } else {
    move(0, 0);
  }
}

void turn_left(int black = 1) {
  move(-turn_speeds, turn_speeds);
  delay(200);
  if (black == 1) {
    while (read_l() < grey_turn) {
      move(-turn_speeds, turn_speeds + 10);
    }
    while (read_r() < grey_turn) {
      move(-turn_speeds, turn_speeds + 10);
    }
  } else {
    while (read_l() > grey_turn_black) {
      move(-turn_speeds, turn_speeds + 10);
    }
    while (read_r() > grey_turn_black) {
      move(-turn_speeds, turn_speeds + 10);
    }
  }

  move(255, -255);
  delay(time_break);
  move(0, 0);
  sink(black);
}

void turn_right(int black = 1) {
  move(turn_speeds, -turn_speeds);
  delay(200);
  if (black == 1) {
    while (read_r() < grey_turn) {
      move(turn_speeds, -turn_speeds - 10);
    }
    while (read_l() < grey_turn) {
      move(turn_speeds, -turn_speeds - 10);
    }
  } else {
    while (read_r() > grey_turn_black) {
      move(turn_speeds, -turn_speeds - 10);
    }
    while (read_l() > grey_turn_black) {
      move(turn_speeds, -turn_speeds - 10);
    }
  }

  move(-255, 255);
  delay(time_break);
  move(0, 0);
  sink(black);
}

void turn_right_scvos_line(int black) {
  if (black == 1) {
    move(turn_speeds, -turn_speeds);
    delay(200);
    while (read_r() < grey_turn) {
      move(turn_speeds, -turn_speeds);
    }
    while (read_l() < grey_turn) {
      move(turn_speeds, -turn_speeds);
    }

    move(turn_speeds, -turn_speeds);
    delay(200);
    while (read_r() < grey_turn) {
      move(turn_speeds, -turn_speeds);
    }
    while (read_l() < grey_turn) {
      move(turn_speeds, -turn_speeds);
    }
  } else {
    move(turn_speeds, -turn_speeds);
    delay(200);
    while (read_r() > grey_turn_black) {
      move(turn_speeds, -turn_speeds - 10);
    }
    while (read_l() > grey_turn_black) {
      move(turn_speeds, -turn_speeds - 10);
    }
    move(turn_speeds, -turn_speeds);
    delay(200);
    while (read_r() > grey_turn_black) {
      move(turn_speeds, -turn_speeds - 10);
    }
    while (read_l() > grey_turn_black) {
      move(turn_speeds, -turn_speeds - 10);
    }
  }
  move(-255, 255);
  delay(time_break);
  move(0, 0);
  sink(black);
}

void take(bool f_side, int black = 1) {
  if (f_side == true) {
    down_forward();
    open_forward();
    follow_line_time(2, 600, black);
    close_forward();
    up_forward();
    turn_right(black);
    move_time(300, -1);
    delay(500);
    follow_cross(1, black);
  } else {
    follow_line_time(2, 550, black);
    turn_right(black);
    down_back();
    open_back();
    move_time(350, -1);
    close_back();
    up_back();
    follow_cross(1, black);
  }
}

void put(bool f_side) {
  if (f_side == true) {
    follow_line_to_cross(2);
    move_time(250, 1);
    down_forward();
    open_forward();
    move_time(320, -1);
    turn_right();
    close_forward();
    up_forward();
    move_time(200, -1);
    delay(500);
    follow_cross(1);
  } else {
    follow_line_to_cross(2);
    move_time(200, 1);
    turn_right_scvos_line(1);
    move_time(400, -1);
    down_back();
    open_back();
    follow_cross(1);
    close_back();
    up_back();
  }
}

void up_level() {
  follow_line_time(2, 250, 1);
  follow_line_time(2, 1600, -1);
  follow_line_to_cross(1, -1);
}

void down_level() {
  follow_line_time(2, 1250, -1);
  follow_line_to_cross(1, 1);
}

void follow_line_fast_to_cross(bool start = true, bool tormoz = true) {
  s_left = read_l();
  s_right = read_r();
  if (start) {
    local_speed = start_speed;
  } else {
    local_speed = final_speed;
  }
  last_change_speed = millis();
  while (local_speed < final_speed) {
    follow_line_fast(local_speed);
    s_left = read_l();
    s_right = read_r();
    if (millis() - last_change_speed >= 1) {
      local_speed += a_uskorenie;
      last_change_speed = millis();
    }
  }
  while (s_left < grey or s_right < grey) {
    follow_line_fast(local_speed);
    s_left = read_l();
    s_right = read_r();
  }
  timer = millis();
  if (tormoz) {
    timer = millis();
    while (millis() - timer < 100) {
      follow_line_fast(local_speed);
      s_left = read_l();
      s_right = read_r();
    }

    last_change_speed = millis();
    while (local_speed > start_speed) {
      follow_line_fast(local_speed);
      s_left = read_l();
      s_right = read_r();
      if (millis() - last_change_speed >= 1) {
        local_speed -= a_tormozenie;
        last_change_speed = millis();
      }
    }
    motor_stop();
  } else {
    delay(50);
  }
  
}

void follow_cross_fast(int data) {
  if (data == 1) {
    follow_line_fast_to_cross(1, 1);
  } else {
    follow_line_fast_to_cross(1, 0);
    for (int i = 0; i < data - 2; ++i) {
      follow_line_fast_to_cross(0, 0);
    }
    follow_line_fast_to_cross(0, 1);
  }
  
}
void follow_line_up_level() {
  s_left = read_l();
  s_right = read_r();
  if (1) {
    local_speed = start_speed;
  } else {
    local_speed = final_speed;
  }
  timer = millis();
  last_change_speed = millis();
  while (local_speed < final_speed) {
    follow_line_fast(local_speed, -1);
    s_left = read_l();
    s_right = read_r();
    if (millis() - last_change_speed >= 1) {
      local_speed += a_uskorenie;
      last_change_speed = millis();
    }
  }
  
  while (millis() - timer < 600) {
    follow_line_fast(local_speed, -1);
    s_left = read_l();
    s_right = read_r();
  }

  while (s_left > black_grey or s_right > black_grey) {
    follow_line_fast(local_speed, -1);
    s_left = read_l();
    s_right = read_r();
  }
  timer = millis();
  while (millis() - timer < 80) {
    follow_line_fast(local_speed, -1);
    s_left = read_l();
    s_right = read_r();
  }

  last_change_speed = millis();
  while (local_speed > start_speed) {
    follow_line_fast(local_speed, -1);
    s_left = read_l();
    s_right = read_r();
    if (millis() - last_change_speed >= 1) {
      local_speed -= a_tormozenie;
      last_change_speed = millis();
    }
  }
  motor_break();
}

int check_level() {
  move(turn_speeds, -turn_speeds);
  delay(300);
  move(-255, 255);
  delay(time_break);
  delay(200);
  if (read_r() < grey) {
    turn_left();
    return 1;
  } else {
    turn_left(-1);
    return -1;
  }
}


bool receive(CameraInfo* table) {
  CameraSerial.print("!");

  bool state = CameraSerial.readBytes((char*)table, sizeof(CameraInfo)) == sizeof(CameraInfo);

  while (CameraSerial.available()) {
    CameraSerial.read();
  }
  return state;
}

void RobotSetup() {
  CameraSerial.println("Started Setup");
  pinMode(but, INPUT);
  pinMode(line_sensor_r, INPUT);
  pinMode(line_sensor_l, INPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(in_3, OUTPUT);
  pinMode(in_4, OUTPUT);
  serva_claw_forward.attach(16);
  serva_upper_forward.attach(27);
  serva_claw_back.attach(33);
  serva_upper_back.attach(25);
  pinMode(sharp, INPUT);

  close_forward();
  up_forward();
  close_back();
  up_back();
  while (CameraSerial.available()) {
    CameraSerial.read();
  }

  // while (true) DebugSerial.println("Setup Finished");
}

void turnToAngle(uint8_t startDir, uint8_t targetDir, uint8_t floor) {
  while (startDir != targetDir) {  // Match needed rotation
    if ((uint8_t)(targetDir - startDir) % 4 == 3) {
      turn_left(floor ? -1 : 1);
      startDir = (startDir + 3) % 4;
    } else if ((uint8_t)(targetDir - startDir) % 4 == 2) {
      turn_right_scvos_line(floor ? -1 : 1);
      startDir = (startDir + 6) % 4;
    } else {
      turn_right(floor ? -1 : 1);
      startDir = (startDir + 1) % 4;
    }
  }
}
