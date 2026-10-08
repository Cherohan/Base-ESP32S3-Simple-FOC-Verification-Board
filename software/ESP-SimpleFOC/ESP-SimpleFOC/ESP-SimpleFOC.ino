#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//==== LCD ST7735S 引脚定义 ====
#define TFT_CS     1
#define TFT_DC     41
#define TFT_RST    40
#define TFT_SCLK   2
#define TFT_MOSI   42
#define BL_PIN     39

//==== DRV8313 驱动引脚 ====
#define DRV_INA    11
#define DRV_INB    10
#define DRV_INC    9
#define DRV_FAULT  12
#define DRV_SLEEP  13

//==== AS5600 I2C引脚 ====
#define I2C_SDA    5
#define I2C_SCL    4

//==== WS2812 RGB ====
#define RGB_PIN 21
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

//==== SimpleFOC 对象 ====
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);   // 极对数=7（12槽14极）
BLDCDriver3PWM driver(DRV_INA, DRV_INB, DRV_INC, DRV_SLEEP);

//==== 硬件SPI ====
SPIClass hspi(HSPI);
Adafruit_ST7735 tft = Adafruit_ST7735(&hspi, TFT_CS, TFT_DC, TFT_RST);

//==== RGB 状态函数 ====
void setRGB(uint8_t r, uint8_t g, uint8_t b){
  pixels.setPixelColor(0, pixels.Color(r,g,b));
  pixels.show();
}

//==== FOC 任务句柄（用于跨任务控制）====
TaskHandle_t focTaskHandle = NULL;
volatile bool focEnabled = true;   // 全局使能标志


//=====================================================================
//  ★ FOC 实时任务：固定在核心 0，优先级 5，1kHz 控制频率
//=====================================================================
void focTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);   // 1ms → 1000Hz

    for (;;) {
        if (focEnabled) {
            motor.loopFOC();
            motor.move();
        }

        // 驱动故障检测（放在 FOC 任务里，响应更快）
        if (digitalRead(DRV_FAULT) == LOW) {
            driver.disable();
            focEnabled = false;   // 停止 FOC 输出
            setRGB(2, 0, 0);      // 红灯常亮表示故障
        }

        // 精确 1ms 周期延时，保证控制频率稳定
        vTaskDelayUntil(&lastWake, period);
    }
}


//=====================================================================
//  setup()
//=====================================================================
void setup() {
    // ---- 基础外设 ----
    pixels.begin();
    setRGB(2, 0, 0);   // 红色：上电等待

    Serial.begin(115200);
    delay(100);   // 等串口稳定

    // 打印 tick rate 确认配置
    Serial.print("Tick Rate: ");
    Serial.println(configTICK_RATE_HZ);

    pinMode(BL_PIN, OUTPUT);
    digitalWrite(BL_PIN, HIGH);

    // ---- LCD 初始化 ----
    hspi.begin(TFT_SCLK, -1, TFT_MOSI);
    hspi.setFrequency(40000000);   // 40MHz，加快刷新
    tft.initR(INITR_MINI160x80);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.println("SimpleFOC RTOS");
    tft.println("initFOC...");

    // ---- AS5600 传感器 ----
    Wire.begin(I2C_SDA, I2C_SCL);
    sensor.init();
    sensor.min_elapsed_time = 0.001f;   // 1ms 采样间隔
    motor.linkSensor(&sensor);

    // ---- DRV8313 驱动 ----
    pinMode(DRV_FAULT, INPUT_PULLUP);
    pinMode(DRV_SLEEP, OUTPUT);
    digitalWrite(DRV_SLEEP, HIGH);

    driver.voltage_power_supply = 12.0f;
    driver.pwm_frequency = 20000;
    driver.init();
    motor.linkDriver(&driver);

    // ---- FOC 参数 ----
    motor.voltage_limit = 6.0f;           // 保守值，保护电机
    motor.velocity_limit = 20.0f;
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 0.02f;          // 先关 I，排除积分锁死
    motor.PID_velocity.D = 0;
    motor.LPF_velocity.Tf = 0.02f;        // 20ms 速度滤波

    // ★ 关键：手动指定传感器方向（根据之前调试结果）
    // 如果之前 initFOC 自动找的方向不对，取消下面这行注释
    // motor.sensor_direction = Direction::CW;

    // ---- 初始化 ----
    setRGB(0, 0, 2);   // 蓝色：标定中
    motor.init();
    motor.initFOC();

    // 打印标定结果
    Serial.print("Zero Angle: ");
    Serial.println(motor.zero_electric_angle, 4);
    Serial.print("Sensor Dir: ");
    Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");

    motor.controller = MotionControlType::velocity;
    motor.target = 15.0f;   // 低速起步

    // ---- 创建 FOC 任务：核心 0，优先级 5 ----
    xTaskCreatePinnedToCore(
        focTask,          // 任务函数
        "FOC",            // 任务名
        8192,             // 栈大小（字节）
        NULL,             // 参数
        5,                // 优先级（高于 loop 任务）
        &focTaskHandle,   // 任务句柄
        0                 // 绑定到核心 0
    );

    tft.println("FOC Ready!");
    setRGB(0, 2, 0);   // 绿色：正常运行
}


//=====================================================================
//  loop()：运行在核心 1，只负责 LCD 刷新和串口
//=====================================================================
void loop() {
    static uint32_t tmr = 0;

    if (millis() - tmr > 200) {
        tmr = millis();

        // 整屏刷也没关系了，因为 FOC 跑在另一个核心
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, 0);
        tft.print("Vel:");
        tft.println(motor.shaft_velocity, 2);
        tft.print("Tgt:");
        tft.println(motor.target, 2);
        tft.print("Ang:");
        tft.println(sensor.getAngle(), 2);

        // 串口同步输出关键变量
        Serial.print("Vel=");
        Serial.print(motor.shaft_velocity, 2);
        Serial.print("  Tgt=");
        Serial.print(motor.target, 2);
        Serial.print("  Ang=");
        Serial.println(sensor.getAngle(), 2);
    }

    vTaskDelay(pdMS_TO_TICKS(10));   // 让出 CPU，避免占用核心 1 满载
}
