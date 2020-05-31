#include <Arduino.h>
#include <BluetoothSerial.h>
// ESP32
#define AIN1 13
#define AIN2 14
#define PWMA 12
#define BIN1 26
#define BIN2 27
#define PWMB 25
#define ECHO 32               // HC-SR04のECHO
#define TRIG 33               // HC-SR04のTRIG
#define LEDC_CHANNEL_A 6
#define LEDC_CHANNEL_B 7
#define LEDC_BASE_FREQ 10000
#define MAX_SPEED 3
#define MIN_SPEED -1
#define ANGLE_STEP 6
#define ANGLE_MAX 18
#define MAX_WAIT 20000
#define MIN_DIST 0.3
#define LED_PIN 5
// Bluetoothシリアルに付ける名前
const char *btname = "ESP32Car";
// 速度・角度・距離を保存するグローバル変数
int speed = 0;
int angle = 0;
double dist;
// Bluetoothシリアルに対応するグローバル変数
BluetoothSerial SerialBT;

void read_bt(){
  // Bluetoothシリアルから文字を受信しているかどうかを判断する
  if (SerialBT.available()){
    digitalWrite(LED_PIN, HIGH);
    // Bluetoothシリアルから1文字読み込む
    int ch = SerialBT.read();
    Serial.print("Bluetooth :");
    Serial.println(ch);
    if (ch == 'e'){
      // speedがMAX_SPEEDより小さければ1増やす
      if (speed < MAX_SPEED){
        speed++;
      }
    }
    // 文字がxのとき
    else if (ch == 'x'){
      // speedがMIN_SPEEDより大きければ1減らす
      if (speed > MIN_SPEED){
        speed--;
      }
    }
    // 文字がdのとき
    else if (ch == 'd'){
      // angleがANGLE_MAXより小さければANGLE_STEPだけ増やす(ハンドルを右にきる)
      if (angle < ANGLE_MAX){
        angle += ANGLE_STEP;
      }
    }
    // 文字がsのとき
    else if (ch == 's'){
      // angleがANGLE_MAXより大きければANGLE_STEPだけ減らす(ハンドルを左にきる)
      if (angle > -ANGLE_MAX){
        angle -= ANGLE_STEP;
      }
    }
    // 文字がiのとき
    else if (ch == 'i'){
      // モーターを止め，ハンドルを中心に戻す
      speed = 0;
      angle = 0;
    }
  }
  digitalWrite(LED_PIN, LOW);
}

void read_distance(){
  // 超音波を10マイクロ秒出力
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  // 反射した超音波を受信するまでの時間を図る
  double time = pulseIn(ECHO, HIGH, MAX_WAIT);
  // タイム・アウトした場合はMAX_wAITだけ時間がかかったことにする
  if (time == 0){
    time = MAX_WAIT;
  }
  // 距離を求める
  dist = (time / 1000 / 1000) / 2 * 340;
  Serial.print("Distance :");
  Serial.println(dist);
}

void operate_motor(){
  // speedがプラスで，障害物までの距離がMIN_DISTより大きい場合
  if (speed > 0 && dist > MIN_DIST){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }
  // speedがマイナスの場合
  else if(speed < 0){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
  }
  else{
    // モーターを止める
    speed = 0;
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);
  }
  // モーターの回転速度を決める
  ledcWrite(LEDC_CHANNEL_A, 30 + 40 * abs(speed));
  ledcWrite(LEDC_CHANNEL_B, 30 + 40 * abs(speed));
}
void setup() {
  // Bluetoothシリアルの初期化
  SerialBT.begin(btname);
  Serial.begin(9600);
  // PWMの初期化
  ledcSetup(LEDC_CHANNEL_A, LEDC_BASE_FREQ, 8);
  ledcSetup(LEDC_CHANNEL_B, LEDC_BASE_FREQ, 8);
  ledcAttachPin(PWMA, LEDC_CHANNEL_A);
  ledcAttachPin(PWMB, LEDC_CHANNEL_B);
  // GPIOピンの初期化
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LED_PIN, OUTPUT);
}
void loop() {
  // Bluetoothシリアルから文字を読み込んで処理する
  read_bt();
  // 障害物までの距離を測定する
  read_distance();
  // モーターを操作する
  operate_motor();
}