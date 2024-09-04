#include <math.h>
#include <mbed.h>

// オムニホイールのパラメータ
const float robot_radius = 5.00;  //ロボットの重心からホイール中心までの距離[m]
const float wheel_radius = 1.00;  //ホイール半径[m]

// ロボットのｘ軸方向の速度 V_x
// ロボットのｙ軸方向の速度 V_y
// ロボットの回転角度 theta
// ロボットの角速度 omega
// 目標地点の座標 (target_x, target_y)
// ロボットの現在位置 (robot_x, robot_y)
// ロボットの現在の向き (robot_theta) 

// 各ホイールの速度
float wheel1_speed, wheel2_speed, wheel3_speed, wheel4_speed;

// ホイールの速度を計算する関数
void calculate_wheel_speeds(float V_x, float V_y,float omega,float theta){
    wheel1_speed = (-sin(theta + M_PI/4) * V_x + cos(theta + M_PI/4) * V_y + robot_radius * omega) / wheel_radius;
    wheel2_speed = (-sin(theta + 3*M_PI/4) * V_x + cos(theta + 3*M_PI/4) * V_y + robot_radius * omega) / wheel_radius;
    wheel3_speed = (-sin(theta + 5*M_PI/4) * V_x + cos(theta + 5*M_PI/4) * V_y + robot_radius * omega) / wheel_radius;
    wheel4_speed = (-sin(theta + 7*M_PI/4) * V_x + cos(theta + 7*M_PI/4) * V_y + robot_radius * omega) / wheel_radius;
}

void calculate_speed_to_target(float target_x, float target_y, float robot_x, float robot_y, float robot_theta, float &V_x, float &V_y, float &omega){
    // 目標地点への相対的な位置を計算
    float delta_x = target_x - robot_x;
    float delta_y = target_y - robot_y;
    
    // 極座標に変換
    float rho = sqrt(delta_x * delta_x + delta_y * delta_y);  // 距離
    float alpha = atan2(delta_y, delta_x);  // 角度
    
    // ロボットの向きと目標地点の角度の差
    float angle_to_target = alpha - robot_theta;
    
    // ロボットの速度と角速度を計算
    V_x = rho * cos(angle_to_target);  // ロボットの進行方向への速度
    V_y = rho * sin(angle_to_target);  // ロボットの横方向への速度
    omega = angle_to_target;           // ロボットの回転速度
}

int main() {
    // 目標地点の座標
    float target_x = 10.0;
    float target_y = 5.0;

    // ロボットの現在の位置と向き
    float robot_x = 0.0;
    float robot_y = 0.0;
    float robot_theta = 0.0;

    // ロボットの速度と角速度
    float V_x, V_y, omega;

    // 目標地点に向かうための速度と角速度を計算
    calculate_speed_to_target(target_x, target_y, robot_x, robot_y, robot_theta, V_x, V_y, omega);

    // 計算された速度と角速度を基にホイールの速度を計算
    calculate_wheel_speeds(V_x, V_y, omega, robot_theta);

    // 結果を表示
    printf("Wheel 1 Speed: %f\n", wheel1_speed);
    printf("Wheel 2 Speed: %f\n", wheel2_speed);
    printf("Wheel 3 Speed: %f\n", wheel3_speed);
    printf("Wheel 4 Speed: %f\n", wheel4_speed);

    while (true) {
        // メインループ
    }
}