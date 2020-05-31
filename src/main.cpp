#include <Arduino.h>
#include <BluetoothSerial.h>
// ESP32
#define AIN1 13               // TB6612のAIN1
#define AIN2 14               // TB6612のAIN2
#define PWMA 12               // TB6612のPWMA
#define BIN1 26               // TB6612のBIN1
#define BIN2 27               // TB6612のBIN2
#define PWMB 25               // TB6612のPWMB
#define ECHO 32               // HC-SR04のECHO
#define TRIG 33               // HC-SR04のTRIG
#define LEDC_CHANNEL_A 6      // PWMのチャンネルA
#define LEDC_CHANNEL_B 7      // PWMのチャンネルB
#define LEDC_BASE_FREQ 10000  // PWMの周波数
#define MAX_SPEED 3           // 速度の最大値
#define MIN_SPEED -1          // 速度の最小値
#define ANGLE_MAX 1           // 角度の最大値
#define ANGLE_MIN -1          // 角度の最小値
#define MAX_WAIT 20000        // pulseIn関数のタイムアウト値
#define MIN_DIST 0.2          // 障害物までの最小距離
#define LED_PIN_BT 5          // Bluetooth接続確認用LED
#define LED_PIN_Right 16      // ロボカーの右LED
#define LED_PIN_Left 17       // ロボカーの左LED
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
    digitalWrite(LED_PIN_BT, HIGH);
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
      // angleがANGLE_MAXより小さければ1増やす(ハンドルを右にきる)
      if (angle < ANGLE_MAX){
        angle++;
      }
    }
    // 文字がsのとき
    else if (ch == 's'){
      // angleがANGLE_MAXより大きければ1減らす(ハンドルを左にきる)
      if (angle > ANGLE_MIN){
        angle--;
      }
    }
    // 文字がiのとき
    else if (ch == 'i'){
      // モーターを止め，ハンドルを中心に戻す
      speed = 0;
      angle = 0;
    }
    Serial.print("speed: ");
    Serial.print(speed);
    Serial.print(" ,");
    Serial.print("angle: ");
    Serial.println(angle);
  }
  digitalWrite(LED_PIN_BT, LOW);
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
  //Serial.print("Distance: ");
  //Serial.println(dist);
}

void operate_motor(){
  // speedがプラスで，障害物までの距離がMIN_DISTより大きい場合
  if (speed > 0 && dist > MIN_DIST){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
    digitalWrite(LED_PIN_Right,LOW);
    digitalWrite(LED_PIN_Left,LOW);
  }
  // speedがマイナスの場合
  else if(speed < 0){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
    digitalWrite(LED_PIN_Right,HIGH);
    digitalWrite(LED_PIN_Left,HIGH);
  }
  // angleがプラスの場合
  else if(angle > 0 && dist > MIN_DIST - 0.1){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
    digitalWrite(LED_PIN_Right,HIGH);
    digitalWrite(LED_PIN_Left,LOW);
  }
  // angleがマイナスの場合
  else if(angle < 0 && dist > MIN_DIST  - 0.1){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
    digitalWrite(LED_PIN_Right,LOW);
    digitalWrite(LED_PIN_Left,HIGH);
  }
  else{
    // モーターを止める
    speed = 0;
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);
    digitalWrite(LED_PIN_Right,LOW);
    digitalWrite(LED_PIN_Left,LOW);
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
  pinMode(LED_PIN_BT, OUTPUT);
  pinMode(LED_PIN_Right, OUTPUT);
  pinMode(LED_PIN_Left, OUTPUT);
}
void loop() {
  // Bluetoothシリアルから文字を読み込んで処理する
  read_bt();
  // 障害物までの距離を測定する
  read_distance();
  // モーターを操作する
  operate_motor();
}
