/*
 * ============================================================
 * pathfinder_commented.ino —— 循迹小车比赛程序（带注释工作版）
 * ============================================================
 *
 * 【本文件说明】
 *   - 本文件是 src/pathfinder/pathfinder.ino 的注释增强副本。
 *   - 代码语句与原件 100% 一致（仅增加注释），行为完全不变。
 *   - pathfinder.ino 保留为只读基准，请勿修改；后续改动在本文件进行。
 *
 * 【赛事背景】智能芯片与自动驾驶专项赛·高中组决赛
 *   - 规则文件: D:\dev\Robot\小车\高中组比赛规则.docx
 *   - 基础任务100分: 开始工作5 / 岗位巡查30 / 重点消防15 /
 *     货品管理20 / 检查仓门20 / 返回充电10
 *   - 拓展任务30分: 设备巡检(6台光刻机+3锥桶)
 *   - 每轮4分钟, 两轮取最高分; 鸣笛2声或闪灯2次 = 常见"动作完成"信号
 *
 * 【硬件接线】(Arduino Nano)
 *   ┌──────────┬──────────┬──────────────────────────┐
 *   │ 引脚      │ 外设      │ 说明                      │
 *   ├──────────┼──────────┼──────────────────────────┤
 *   │ D8       │ 蜂鸣器    │ 低电平有效(LOW=响)          │
 *   │ D11      │ RGB绿灯   │ 低电平有效(LOW=亮)          │
 *   │ D12      │ RGB红灯   │ 低电平有效(LOW=亮)          │
 *   │ D13      │ RGB蓝灯   │ 低电平有效(LOW=亮)          │
 *   │ D9       │ 舵机      │ 机械臂/门锁推拉杆 (0~180°)  │
 *   │ D10      │ 按键      │ 校准时按三次确认白/黑采样    │
 *   │ D2/D4/D3│ 左电机    │ TB6612 AIN1/AIN2 + PWMA    │
 *   │ D5/D7/D6│ 右电机    │ TB6612 BIN1/BIN2 + PWMB    │
 *   │ A0~A3   │ 光电传感器 │ A0左外 A1中左 A2中右 A3右外 │
 *   └──────────┴──────────┴──────────────────────────┘
 *   TB6612 要点: VM接电池、VCC=5V、STBY必须拉高;
 *   转动 = 方向脚一高一低 + 对应PWM脚给速度, 单拉一脚无效。
 *
 * 【传感器判线逻辑】
 *   每个传感器先标定: 白场采样值(white) + 黑线采样值(black),
 *   阈值 = (白+黑)/2。运行中 analogRead > 阈值 = 压到黑线。
 *   A1/A2 是循迹主力(中两路), A0/A3 是左右边界路口检测。
 *
 * 【电机逻辑速查】以左电机(D2/D4/D3)为例:
 *   digitalWrite(2, dir);   → AIN1
 *   digitalWrite(4, !dir);  → AIN2
 *   analogWrite(3, 速度);   → PWMA (0~255)
 *   两方向脚取反即反转; speed 是基础速度, correction 是右轮
 *   直线补偿(直行时右轮 = speed+correction, 抵消电机差异跑偏)。
 *
 * 【调参入口】全部在 loop() 开头的 getThresholds(正反, 速度, 校正):
 *   - 正反: 0=正向, 1=反向(与电机接线匹配)
 *   - 速度: 循迹基准速度 (现150)
 *   - 校正: 右轮补偿 -20~+20 (现7)
 * ============================================================
 */
#include <Servo.h>

Servo servo_9;                  // 舵机对象: 机械臂/门锁推拉杆 (D9)

/* ---- 4路光电传感器: 白场/黑线/阈值 三组采样值 ----
 *   A0=左外  A1=中左  A2=中右  A3=右外 */
volatile int my_1_white;        // A0 白场采样
volatile int my_2_white;        // A1 白场采样
volatile int my_3_white;        // A2 白场采样
volatile int my_4_white;        // A3 白场采样
volatile int my_1_black;        // A0 黑线采样
volatile int my_2_black;        // A1 黑线采样
volatile int my_3_black;        // A2 黑线采样
volatile int my_4_black;        // A3 黑线采样
volatile int my_1_threshold;    // A0 判定阈值 (白+黑)/2
volatile int my_2_threshold;    // A1 判定阈值
volatile int my_3_threshold;    // A2 判定阈值
volatile int my_4_threshold;    // A3 判定阈值

/* ---- 全局控制参数(在 getThresholds 里赋值) ---- */
volatile int speed;             // 循迹基准速度 (0~255)
volatile int correction;        // 右轮直线补偿, 直行时右轮=speed+correction
volatile bool direction;        // 电机正反: 与接线匹配的"前进"方向标志
volatile float systemTime;      // 巡线计时起点 (millis快照, 用于超时退出)

/* ============================================================
 * 校准: 上电后按提示按键采样白/黑, 计算4路阈值
 *   forward = 正反标志   initSpeed = 基准速度   straightCorrection = 右轮补偿
 *   流程: 按键(按住采样白) → 红灯闪3次 → 按键(按住采样黑) → 红灯闪3次
 *         → 算阈值 → 再按一次键开始比赛
 * ============================================================ */
void getThresholds(bool forward, int initSpeed, int straightCorrection) {
  direction = forward;
  speed = initSpeed;
  correction = straightCorrection;
  while (digitalRead(10) == 1) {          // 按键按下期间采样白场
    my_1_white = analogRead(A0);
    my_2_white = analogRead(A1);
    my_3_white = analogRead(A2);
    my_4_white = analogRead(A3);
  }
  for (int i = 1; i <= 3; i = i + (1)) {  // 红灯闪3次: "白场已采完, 请移到黑线"
    digitalWrite(12, LOW);
    delay(75);
    digitalWrite(12, HIGH);
    delay(75);
  }
  while (digitalRead(10) == 1) {          // 按键按下期间采样黑线
    my_1_black = analogRead(A0);
    my_2_black = analogRead(A1);
    my_3_black = analogRead(A2);
    my_4_black = analogRead(A3);
  }
  for (int i = 1; i <= 3; i = i + (1)) {  // 红灯闪3次: "黑线已采完"
    digitalWrite(12, LOW);
    delay(75);
    digitalWrite(12, HIGH);
    delay(75);
  }
  my_1_threshold = (my_1_white + my_1_black) / 2;   // 阈值=白黑中点
  my_2_threshold = (my_2_white + my_2_black) / 2;
  my_3_threshold = (my_3_white + my_3_black) / 2;
  my_4_threshold = (my_4_white + my_4_black) / 2;
  while (digitalRead(10) == 1) {          // 最后按一次: 松手即开始比赛
  }
}

/* ============================================================
 * 停止程序: 两电机立即停转, 然后死循环挂起(不再执行任何动作)
 * ============================================================ */
void stopProgram() {
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  analogWrite(3, 0);                      // 左电机: AIN1=H AIN2=L PWMA=0
  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  analogWrite(6, 0);                      // 右电机: BIN1=L BIN2=H PWMB=0
  while(true);                            // 原地待命, 防失控
}

/* ============================================================
 * RGB红灯闪烁 blinkCount 次 (D12, 低有效)
 *   规则对应: 2次闪灯 = 任务完成/需要充电/设备故障等信号
 * ============================================================ */
void RGB_Red(int blinkCount, int interval) {
  for (int i = (1); i <= (blinkCount); i = i + (1)) {
    digitalWrite(12, LOW);                // 亮
    delay(interval);
    digitalWrite(12, HIGH);               // 灭
    delay(100);
  }
}

/* ============================================================
 * RGB绿灯闪烁 (D11, 低有效) —— 规则: 绿=正常/满电, 通常闪1次
 * ============================================================ */
void RGB_Green(int blinkCount, int interval) {
  for (int i = (1); i <= (blinkCount); i = i + (1)) {
    digitalWrite(11, LOW);
    delay(interval);
    digitalWrite(11, HIGH);
    delay(interval);
  }
}

/* ============================================================
 * RGB蓝灯闪烁 (D13, 低有效)
 * ============================================================ */
void RGB_Blue(int blinkCount, int interval) {
  for (int i = (1); i <= (blinkCount); i = i + (1)) {
    digitalWrite(13, LOW);
    delay(interval);
    digitalWrite(13, HIGH);
    delay(interval);
  }
}

/* ============================================================
 * 舵机控制 (D9): 转到指定角度, 用于"检查仓门"任务拉出门锁推拉杆
 *   angle_0_180 = 0~180°
 * ============================================================ */
void servoControl_D9(int angle_0_180) {
  servo_9.write(angle_0_180);             // 转到位
  delay(500);                             // 保持0.5s
  delay(800);                             // 再保持0.8s
}

/* ============================================================
 * 蜂鸣器报警 (D8, 低有效): 响 alarmCount 次
 *   规则对应: 2声 = 男技术员/蓝消防箱/红标牌/设备故障;
 *             1声 = 绿标牌/设备正常
 * ============================================================ */
void buzzerAlarm_D8(int alarmCount, int interval) {
  for (int i = (1); i <= (alarmCount); i = i + (1)) {
    digitalWrite(8, HIGH);                // 响
    delay(interval);
    digitalWrite(8, LOW);                 // 停
    delay(interval);
  }
}

/* ============================================================
 * 直行 timeMs 毫秒 (两轮同向, 右轮带校正补偿)
 *   例: goStraight(300) = 前进约300ms
 * ============================================================ */
void goStraight(int timeMs) {
  digitalWrite(2, direction);             // 左电机 AIN1=方向
  digitalWrite(4, (!direction));          //       AIN2=反向
  analogWrite(3, speed);                  //       PWMA=基准速度
  digitalWrite(5, (!direction));          // 右电机 BIN1=反向
  digitalWrite(7, direction);             //       BIN2=方向
  analogWrite(6, (speed + correction));   //       PWMB=速度+右轮补偿
  delay(timeMs);                          // 持续行进
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 后退 timeMs 毫秒 (两轮反向, 与直行相反的极性)
 * ============================================================ */
void goBackward(int timeMs) {
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, speed);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, (speed + correction));
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 右转 timeMs: 右轮停、左轮转 → 走弧线(非原地)
 * ============================================================ */
void turnRight(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, speed);                  // 左轮转
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, 0);                      // 右轮停
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 原地右转(右转圈) timeMs: 两轮反向 → 绕车中心自转
 * ============================================================ */
void spinRight(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, 120);                    // 左轮前进
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 120);                    // 右轮后退
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 左转 timeMs: 左轮停、右轮转 → 走弧线(非原地)
 * ============================================================ */
void turnLeft(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, 0);                      // 左轮停
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, speed);                  // 右轮转
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 原地左转(左转圈) timeMs: 两轮反向 → 绕车中心自转
 * ============================================================ */
void spinLeft(int timeMs) {
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 120);                    // 左轮后退
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, 120);                    // 右轮前进
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 转弯遇线停 turnDir: 低速(80)行进, 依次等传感器压线后停止
 *   用于路口/弯道对线对齐。
 *   turnDir=1(左): 等 A0(左外)压线 → 等 A2(中右)压线 → 停
 *   turnDir=2(右): 等 A3(右外)压线 → 等 A1(中左)压线 → 停
 *   注意: 电机极性与 goBackward 相同, 具体行进方向需实测确认
 * ============================================================ */
void turnUntilLine(int turnDir) {
  if (turnDir == 1) {
    while (analogRead(A0) > my_1_threshold) {   // A0 仍压线
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, 80);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 80);
    }
    while (analogRead(A2) > my_3_threshold) {   // A2 仍压线
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, 80);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 80);
    }
    digitalWrite(2, (!direction));              // —— 停车 ——
    digitalWrite(4, direction);
    analogWrite(3, 0);
    digitalWrite(5, direction);
    digitalWrite(7, (!direction));
    analogWrite(6, 0);

  }
  if (turnDir == 2) {
    while (analogRead(A3) > my_4_threshold) {   // A3 仍压线
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, 80);
      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, 80);
    }
    while (analogRead(A1) > my_2_threshold) {   // A1 仍压线
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, 80);
      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, 80);
    }
    digitalWrite(2, (!direction));              // —— 停车 ——
    digitalWrite(4, direction);
    analogWrite(3, 0);
    digitalWrite(5, direction);
    digitalWrite(7, (!direction));
    analogWrite(6, 0);

  }
}

/* ============================================================
 * 巡线找左右路口 turnDir: 循迹行驶直到外侧传感器脱线/路口
 *   turnDir=1(左): 以 A0(左外)为停止条件, 期间用 A1/A2 转向循迹
 *   turnDir=2(右): 以 A3(右外)为停止条件, 期间用 A1/A2 转向循迹
 *   转向逻辑(A1/A2 中两路):
 *     两路都压线      → 直行
 *     两路都脱线      → 直行(可能过路口)
 *     A1脱/A2压(偏右) → 左轮停、右轮转 → 向右修正
 *     A1压/A2脱(偏左) → 左轮转、右轮停 → 向左修正
 *   结束后两电机停转
 * ============================================================ */
void lineFollowJunction(int turnDir) {
  if (turnDir == 1) {
    while (analogRead(A0) > my_1_threshold) {   // 左外路未到, 继续巡线
      if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 都在线上: 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 都脱线(路口): 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 偏右: 左停右转
        digitalWrite(4, (!direction));
        analogWrite(3, 0);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, speed);

      }
      if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 偏左: 左转右停
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, 0);

      }
    }

  }
  if (turnDir == 2) {
    while (analogRead(A3) > my_4_threshold) {   // 右外路未到, 继续巡线
      if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 都在线上: 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 都脱线(路口): 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 偏右: 左停右转
        digitalWrite(4, (!direction));
        analogWrite(3, 0);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, speed);

      }
      if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 偏左: 左转右停
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, 0);

      }
    }

  }
  digitalWrite(2, LOW);                         // —— 结束: 停车 ——
  digitalWrite(4, HIGH);
  analogWrite(3, 0);
  digitalWrite(5, HIGH);
  digitalWrite(7, LOW);
  analogWrite(6, 0);
}

/* ============================================================
 * 定时巡线 timeMs 毫秒: 用 A1/A2 中两路循迹, millis 超时退出
 *   转向逻辑同 lineFollowJunction (直行/修正两档)
 *   结束后停车
 * ============================================================ */
void lineFollowTime(int timeMs) {
  systemTime = millis();                        // 计时起点
  while (millis() - systemTime < timeMs) {      // 未超时则继续
    if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
      digitalWrite(2, direction);               // 都在线上: 直行
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, (speed + correction));

    }
    if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
      digitalWrite(2, direction);               // 都脱线(路口): 直行
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, (speed + correction));

    }
    if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
      digitalWrite(2, direction);               // 偏右: 左停右转
      digitalWrite(4, (!direction));
      analogWrite(3, 0);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, speed);

    }
    if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
      digitalWrite(2, direction);               // 偏左: 左转右停
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 0);

    }
  }
  digitalWrite(2, HIGH);                        // —— 超时: 停车 ——
  digitalWrite(4, LOW);
  analogWrite(3, 0);
  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  analogWrite(6, 0);
}

/* ============================================================
 * setup: 初始化所有引脚与变量
 *   - 蜂鸣器D8、RGB D11/D12/D13 输出并置为"灭"(高电平)
 *   - 舵机D9 挂载, 初始角度180°
 *   - D10 按键输入; 电机6脚 D2/D3/D4/D5/D6/D7 输出
 * ============================================================ */
void setup() {
  pinMode(8, OUTPUT);                           // 蜂鸣器
  pinMode(11, OUTPUT);                          // RGB绿
  pinMode(12, OUTPUT);                          // RGB红
  pinMode(13, OUTPUT);                          // RGB蓝
  servo_9.attach(9);                            // 舵机
  my_1_white = 0;                               // —— 变量清零 ——
  my_2_white = 0;
  my_3_white = 0;
  my_4_white = 0;
  my_1_black = 0;
  my_2_black = 0;
  my_3_black = 0;
  my_4_black = 0;
  my_1_threshold = 0;
  my_2_threshold = 0;
  my_3_threshold = 0;
  my_4_threshold = 0;
  speed = 150;                                  // 默认基准速度
  correction = 0;                               // 默认无补偿
  direction = 0;                                // 默认正向
  systemTime = 0;
  digitalWrite(8, LOW);                         // 蜂鸣器先响一下(上电提示)
  digitalWrite(11, HIGH);                       // RGB 熄灭(高=灭)
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  servo_9.write(180);                           // 舵机到180°
  delay(0);                                     // (Mixly空延时块, 无实际作用)
  pinMode(10, INPUT);                           // 校准按键
  pinMode(2, OUTPUT);                           // 左电机 AIN1
  pinMode(4, OUTPUT);                           // 左电机 AIN2
  pinMode(3, OUTPUT);                           // 左电机 PWMA
  pinMode(5, OUTPUT);                           // 右电机 BIN1
  pinMode(7, OUTPUT);                           // 右电机 BIN2
  pinMode(6, OUTPUT);                           // 右电机 PWMB
}

/* ============================================================
 * loop: 整场固定路线(预先编排, 无视觉判断)
 *   段1 校准: getThresholds(正反=0, 速度=150, 右轮校正=7)
 *   之后是"开始工作→巡线→转弯→信号→…"的固定序列。
 *   注: 规则里男/女技术员、红/蓝箱、红/绿标牌等随机摆放,
 *       本程序(纯光电循迹)按固定路线+固定信号执行,
 *       具体哪段对应哪个任务以现场实测为准。
 * ============================================================ */
void loop() {
  /* ===== 段1: 阈值校准(上电操作, 见 getThresholds) ===== */
  getThresholds(0, 150, 7);         // 正反=0 速度=150 右轮校正=7
  /* ===== 段2: 开始工作(规则任务1: 鸣笛2声后驶离启动区) ===== */
  buzzerAlarm_D8(2, 150);           // 鸣笛2声
  goStraight(300);                  // 驶离启动区
  /* ===== 段3: 第一段巡线 ===== */
  lineFollowTime(770);              // 循迹700ms
  spinRight(650);                   // 原地右转(转弯)
  delay(3500);                      // 等待3.5s(推测: 等车身摆正/现场节奏)
  turnUntilLine(1);                 // 左向对线
  lineFollowJunction(1);            // 循迹找左侧路口
  goStraight(500);
  turnUntilLine(2);                 // 右向对线
  lineFollowJunction(2);            // 循迹找右侧路口
  goStraight(200);
  turnUntilLine(2);
  lineFollowTime(1000);              // 循迹700ms
  delay(1500);                      // 等待1.5s
  /* ===== 段4: 信号+转向(推测: 岗位巡查/重点消防响应) ===== */
  buzzerAlarm_D8(2, 150);           // 鸣笛2声(男技术员?/蓝消防箱?)
  turnUntilLine(1);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  delay(1500);
  buzzerAlarm_D8(2, 150);           // 鸣笛2声
  spinRight(200);                   // 原地右转
  turnUntilLine(2);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  goStraight(300);
  /* ===== 段5: 大转弯+长等待(推测: 跨区) ===== */
  spinLeft(800);                    // 原地左转
  delay(3500);                      // 等待3.5s
  turnUntilLine(2);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  delay(2000);                      // 等待2s
  /* ===== 段6: 急促长鸣+后退(推测: 任务完成/返回) ===== */
  buzzerAlarm_D8(100, 10);          // 连续急促鸣笛100次(结束/提示信号?)
  goBackward(400);                  // 后退400ms
  turnUntilLine(2);
  lineFollowJunction(2);
  delay(2000);
  goBackward(500);                  // 再后退500ms
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(2);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(1);
  /* ===== 段7: 冲刺+收尾 ===== */
  spinRight(300);                   // 原地右转
  goStraight(900);                  // 长直行900ms(冲刺?)
  delay(2000);                      // 等待2s
  buzzerAlarm_D8(2, 150);           // 最后鸣笛2声(推测: 结束信号)

}
