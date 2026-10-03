# mini-shirasu の状態 (Status メッセージを物理単位に直したもの)
std_msgs/Header header

# 0 = 無効, 1 = 電流, 2 = 速度, 3 = 位置, 4 = 異常
uint8 MODE_DISABLED = 0
uint8 MODE_CURRENT = 1
uint8 MODE_VELOCITY = 2
uint8 MODE_POSITION = 3
uint8 MODE_FAULT = 4
uint8 mode

bool saturated_upper   # 制御出力が上側に飽和
bool saturated_lower   # 制御出力が下側に飽和
bool vdc_low           # 母線電圧が低く、測定値を更新できていない

float64 current        # [A]
float64 velocity       # [rad/s] encoder_cpr を数える軸
float64 position       # [回転] encoder_cpr を数える軸
float64 vdc            # [V] 母線電圧
uint16 temperature_raw # 温度の ADC 生値
