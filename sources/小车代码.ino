//信号列表：
//P 停车
//A 前进
//B 后退
//L 左转
//R 右转
//C 设置运行模式，默认为60，后跟 int型整数（0~255）转弯时内轮速度设置。
//K  设置转弯信号发送周期数，后跟int 1-20，默认为5，每个周期2ms。
//S 设置速度，默认为255
//D 校准舵机正方向
#include <Servo.h> 
#define LEFT_AHEAD 10
#define LEFT_BACK 9
#define RIGHT_AHEAD 13
#define RIGHT_BACK 12
#define STEER 14

Servo myservo;  
int TURN = 150;
int RUN1 = 100;
void turnLeft();
void turnRight();
void goAhead();
void park();
void goBack();
void stopBack();

void setup(){
    Serial.begin(9600);
    pinMode(LEFT_AHEAD,OUTPUT);
    pinMode(LEFT_BACK, OUTPUT);
    pinMode(RIGHT_AHEAD, OUTPUT);
    pinMode(RIGHT_BACK, OUTPUT);
    digitalWrite(LEFT_AHEAD, LOW);
    digitalWrite(LEFT_BACK, LOW);
    digitalWrite(RIGHT_AHEAD, LOW);
    digitalWrite(RIGHT_BACK, LOW);;

    pinMode(STEER, OUTPUT);
    digitalWrite(STEER, LOW);
    myservo.attach(14);
    myservo.write(85.5); 
    delay(1000);
}

char incomingByte = ' ';

void loop(){
    if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        if (incomingByte == 'L') {
            Serial.println("TURN LEFT");
            turnLeft();
            Serial.read();
        } else if (incomingByte == 'R'){
            Serial.println("TURN RIGHT");
            turnRight();
            Serial.read();
        } else if (incomingByte == 'A'){
            Serial.println("GO AHEAD");
            goAhead();
            Serial.read();
        } else if (incomingByte == 'B'){
            Serial.println("GO BACK");
            goBack();
            Serial.read();
        } else if (incomingByte == 'P'){
            Serial.println("PARK");
            park();
            Serial.read();
        }
    }

}

void steer(float x)
{
  if (x<0.9){
    myservo.write(77);
  }
  if (x>1.1){
    myservo.write(93);
  }
  if (x==1){
    myservo.write(85.5);
  }
}  

void goAhead(){
    stopBack();
    analogWrite(LEFT_AHEAD,RUN1);
    analogWrite(RIGHT_AHEAD,RUN1);
    steer(1);
}

void turnLeft(){
    stopBack();
    digitalWrite(LEFT_AHEAD,LOW);
    analogWrite(RIGHT_AHEAD,100);
    analogWrite(LEFT_BACK,100);
    steer(1.2); 
}

void turnRight(){
    stopBack();
    digitalWrite(RIGHT_AHEAD,LOW);
    analogWrite(LEFT_AHEAD,100);
    analogWrite(RIGHT_BACK,100);
    steer(0.7);

}

void park(){
    stopBack();
    digitalWrite(LEFT_AHEAD, LOW);
    digitalWrite(RIGHT_AHEAD, LOW);
    steer(1);
}

void goBack(){
    //    stopBack();
    digitalWrite(LEFT_AHEAD,LOW);
    digitalWrite(RIGHT_AHEAD,LOW);
    analogWrite(LEFT_BACK,RUN1);
    analogWrite(RIGHT_BACK,RUN1);
    steer(1);
}

void stopBack(){
    digitalWrite(LEFT_BACK, LOW);
    digitalWrite(RIGHT_BACK, LOW);
}
