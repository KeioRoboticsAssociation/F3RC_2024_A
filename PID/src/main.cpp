#include <mbed.h>
#include <PID.hpp>

// I2C通信のためのピン設定　例
#define I2C1_SDA PC_7
#define I2C1_SCL PB_6

// センサー用のI2C
I2C i2c(I2C1_SDA, I2C1_SCL);

// PIDコントローラの初期化 (例としてKp,Ki,Kdの値を設定)
PID pid1(1.0, 0.1, 0.05, 0.1);
PID pid2(1.0, 0.1, 0.05, 0.1);
PID pid3(1.0, 0.1, 0.05, 0.1);

// 目標距離
const float setpoint1 = 100.0; // センサー1の目標値
const float setpoint2 = 100.0; // センサー2の目標値
const float setpoint3 = 100.0; // センサー3の目標値

// 制御出力の最大値と最小値
const float max_output = 1000.0;
const float min_output = -1000.0;

// DT35センサーから距離を読み取る関数のプロトタイプ
float read_distance(int sensor_id);

int main(){
    // PIDコントローラの初期化
    pid1.setOutputLimits(min_output, max_output);
    pid1.setSetPoint(setpoint1);

    pid2.setOutputLimits(min_output, max_output);
    pid2.setSetPoint(setpoint2);

    pid3.setOutputLimits(min_output, max_output);
    pid3.setSetPoint(setpoint3);

    while (true){
        // センサーからの距離測定
        float distance1 = read_distance(1);
        float distance2 = read_distance(2);
        float distance3 = read_distance(3);

        // PIDコントローラの更新
        pid1.setProcessValue(distance1);
        pid2.setProcessValue(distance2);
        pid3.setProcessValue(distance3);

        // 制御出力の計算
        float control_output1 = pid1.compute();
        float control_output2 = pid2.compute();
        float control_output3 = pid3.compute();

        // 制御出力の表示（または、制御対象に出力）
        printf("Control Output 1: %f\n", control_output1);
        printf("Control Output 2: %f\n", control_output2);
        printf("Control Output 3: %f\n", control_output3);

        // 適切なタイミングでの繰り返し
        wait_us(100000); // 0.1秒待つ
    }
}

// DT35センサーから距離を読み取る関数（モックアップ）
float read_distance(int sensor_id) {
    // センサーIDに応じたI2C通信を行い、距離を取得する
    // 実際のセンサー通信のコードを書く必要があります
    // ここではモックアップとして、センサーIDに基づいて異なる値を返す
    if (sensor_id == 1) {
        return 100.0; // センサー1の値
    } else if (sensor_id == 2) {
        return 150.0; // センサー2の値
    } else {
        return 200.0; // センサー3の値
    }
}