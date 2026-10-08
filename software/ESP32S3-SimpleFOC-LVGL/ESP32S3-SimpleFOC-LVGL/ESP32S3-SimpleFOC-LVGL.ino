#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>

//=====================================================================
//  相电压采样模式
//=====================================================================
#define USE_ADC_VOLTAGE_SAMPLING  0

//=====================================================================
//  引脚定义
//=====================================================================
#define TFT_CS     1
#define TFT_DC     41
#define TFT_RST    40
#define TFT_SCLK   2
#define TFT_MOSI   42
#define BL_PIN     39

#define DRV_INA    11
#define DRV_INB    10
#define DRV_INC    9
#define DRV_FAULT  12
#define DRV_SLEEP  13

#define I2C_SDA    5
#define I2C_SCL    4

#define RGB_PIN    21
#define NUMPIXELS  1

#define VOLT_UA_PIN   18
#define VOLT_UB_PIN    8

#define ENC_A      38
#define ENC_B      17

#define BTN_MODE    47
#define BTN_RUN     48

//=====================================================================
//  电压采样标定
//=====================================================================
#define VOLT_ADC_VREF  3.3f
#define VOLT_ADC_MAX   4095.0f
#define VOLT_DIVIDER   4.3f
#define VOLT_BIAS      0.0f
#define VOLT_SAMPLES   8

//=====================================================================
//  编码器 & 模式参数
//=====================================================================
#define ENC_RANGE        200
#define ANGLE_RANGE_RAD  (2.0f * PI * 10.0f)

#define TARGET_SMOOTH_ALPHA  0.05f

//=====================================================================
//  ★ 速度环指令限制（三级保护）
//=====================================================================
#define SPEED_MAX_CMD     30.0f    // 编码器到端点对应的目标速度 (rad/s)
#define SPEED_ABS_LIMIT   40.0f    // ★ 目标速度绝对上限，永不超过
#define SPEED_RAMP        60.0f    // ★ 斜率限制 (rad/s²)，60 → 40rad/s 需 0.67s

#define UQ_MAX            3.0f
#define MOTOR_R_PHASE     5.0f
#define MOTOR_KT          0.05f

//=====================================================================
//  模式定义
//=====================================================================
enum Mode { MODE_SPEED = 0, MODE_ANGLE, MODE_TORQUE };
volatile Mode currentMode  = MODE_SPEED;
volatile bool motorRunning = false;

//=====================================================================
//  硬件对象
//=====================================================================
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
SPIClass hspi(HSPI);
Adafruit_ST7735 tft = Adafruit_ST7735(&hspi, TFT_CS, TFT_DC, TFT_RST);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver(DRV_INA, DRV_INB, DRV_INC, DRV_SLEEP);

//=====================================================================
//  LVGL v9 缓冲
//=====================================================================
#define LV_BUF_LINES 20
static lv_color_t lv_buf[160 * LV_BUF_LINES];
static lv_display_t *disp = NULL;

//=====================================================================
//  UI 句柄
//=====================================================================
static lv_obj_t *ui_dot;
static lv_obj_t *ui_status;
static lv_obj_t *ui_l1, *ui_l2, *ui_l3;
static lv_obj_t *ui_uv, *ui_uc;

//=====================================================================
//  全局状态
//=====================================================================
TaskHandle_t focTaskHandle = NULL;
volatile float g_bus_v = 12.0f;
volatile bool  g_fault = false;

volatile float g_Ua = 0.0f;
volatile float g_Ub = 0.0f;
volatile float g_Uc = 0.0f;

volatile float g_Ud_est = 0.0f;
volatile float g_Uq_est = 0.0f;
volatile float g_Iq_est = 0.0f;
volatile float g_T_est  = 0.0f;

volatile int32_t  enc_count   = 0;
volatile uint32_t enc_last_us = 0;

float zero_angle    = 0.0f;
float target_smooth = 0.0f;
float spd_cmd       = 0.0f;    // ★ 速度环平滑后的指令

//=====================================================================
//  RGB 状态
//=====================================================================
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
}

void updateRGB() {
    if (g_fault) {
        setRGB(255, 0, 0);
        return;
    }
    if (!motorRunning) {
        setRGB(0, 0, 255);
        return;
    }
    switch (currentMode) {
        case MODE_SPEED:  setRGB(0, 255, 0);     break;
        case MODE_ANGLE:  setRGB(255, 255, 0);   break;
        case MODE_TORQUE: setRGB(255, 80, 150);  break;
    }
}

//=====================================================================
//  模式名
//=====================================================================
const char* modeLong() {
    switch (currentMode) {
        case MODE_SPEED:  return "SPEED";
        case MODE_ANGLE:  return "ANGLE";
        case MODE_TORQUE: return "TORQUE";
    }
    return "?";
}

const char* modeShort() {
    switch (currentMode) {
        case MODE_SPEED:  return "SPD";
        case MODE_ANGLE:  return "ANG";
        case MODE_TORQUE: return "TRQ";
    }
    return "?";
}

//=====================================================================
//  编码器 ISR
//=====================================================================
void IRAM_ATTR encISR() {
    uint32_t now = micros();
    if (now - enc_last_us < 1500) return;
    enc_last_us = now;

    if (digitalRead(ENC_B) == HIGH) {
        if (enc_count < ENC_RANGE) enc_count++;
    } else {
        if (enc_count > -ENC_RANGE) enc_count--;
    }
}

//=====================================================================
//  相电压 → dq
//=====================================================================
static inline void abcToDq(float Ua, float Ub, float Uc, float theta,
                           float &Ud, float &Uq) {
    float half = motor.driver->voltage_power_supply * 0.5f;
    float ua = Ua - half;
    float ub = Ub - half;

    float Ualpha = ua;
    float Ubeta  = (ua + 2.0f * ub) * 0.57735027f;

    float ct = cosf(theta);
    float st = sinf(theta);

    Ud =  Ualpha * ct + Ubeta * st;
    Uq = -Ualpha * st + Ubeta * ct;
}

//=====================================================================
//  LVGL 显示刷新回调
//=====================================================================
static void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.writePixels((uint16_t *)px_map, w * h, true);
    tft.endWrite();

    lv_display_flush_ready(disp);
}

//=====================================================================
//  UI 辅助
//=====================================================================
static lv_obj_t* mk_label(lv_obj_t *parent, const char *txt, int x, int y) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font (l, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(l, lv_color_black(), 0);
    lv_obj_set_style_pad_all   (l, 0, 0);
    lv_obj_set_pos(l, x, y);
    return l;
}

static lv_obj_t* mk_sep(lv_obj_t *parent, int y) {
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, 160, 1);
    lv_obj_set_pos (o, 0, y);
    lv_obj_set_style_bg_color(o, lv_color_black(), 0);
    lv_obj_set_style_bg_opa  (o, LV_OPA_COVER, 0);
    return o;
}

//=====================================================================
//  构建 UI
//=====================================================================
static void build_ui(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
    lv_obj_set_style_bg_opa  (scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all (scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    const int LH = 10;

    ui_dot = lv_obj_create(scr);
    lv_obj_remove_style_all(ui_dot);
    lv_obj_set_size(ui_dot, 6, 6);
    lv_obj_set_pos (ui_dot, 1, 2);
    lv_obj_set_style_bg_color(ui_dot, lv_color_black(), 0);
    lv_obj_set_style_bg_opa  (ui_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_radius  (ui_dot, 3, 0);

    ui_status = mk_label(scr, "Simple FOC 12.0V", 8, 0);
    mk_sep(scr, LH * 1);

    ui_l1 = mk_label(scr, "---", 0, LH * 2);
    ui_l2 = mk_label(scr, "---", 0, LH * 3);
    ui_l3 = mk_label(scr, "---", 0, LH * 4);

    mk_sep(scr, LH * 5);

    ui_uv = mk_label(scr, "Ua  0.0  Ub  0.0", 0, LH * 6);
    ui_uc = mk_label(scr, "Uc  0.0 V   SPD",  0, LH * 7);
}

//=====================================================================
//  相电压采样
//=====================================================================
static void samplePhaseVoltages() {
#if USE_ADC_VOLTAGE_SAMPLING
    uint32_t sumA = 0, sumB = 0;
    for (int i = 0; i < VOLT_SAMPLES; i++) {
        sumA += analogRead(VOLT_UA_PIN);
        sumB += analogRead(VOLT_UB_PIN);
    }
    float adcA = sumA / (float)VOLT_SAMPLES;
    float adcB = sumB / (float)VOLT_SAMPLES;

    float vA = adcA * (VOLT_ADC_VREF / VOLT_ADC_MAX);
    float vB = adcB * (VOLT_ADC_VREF / VOLT_ADC_MAX);

    float ua = (vA - VOLT_BIAS) * VOLT_DIVIDER;
    float ub = (vB - VOLT_BIAS) * VOLT_DIVIDER;

    static float fUa = 6.0f, fUb = 6.0f;
    fUa = fUa * 0.7f + ua * 0.3f;
    fUb = fUb * 0.7f + ub * 0.3f;

    g_Ua = fUa;
    g_Ub = fUb;
    g_Uc = -(fUa - 6.0f + fUb - 6.0f) + 6.0f;
#else
    float theta  = motor.electrical_angle;
    float Ud     = motor.voltage.d;
    float Uq     = motor.voltage.q;

    float ct = cosf(theta);
    float st = sinf(theta);

    float Ualpha = Ud * ct - Uq * st;
    float Ubeta  = Ud * st + Uq * ct;

    const float SQRT3_2 = 0.8660254f;
    float half = motor.driver->voltage_power_supply * 0.5f;

    float ua =  Ualpha                           + half;
    float ub = -0.5f * Ualpha + SQRT3_2 * Ubeta + half;
    float uc = -0.5f * Ualpha - SQRT3_2 * Ubeta + half;

    static float fUa = 6.0f, fUb = 6.0f, fUc = 6.0f;
    fUa = fUa * 0.6f + ua * 0.4f;
    fUb = fUb * 0.6f + ub * 0.4f;
    fUc = fUc * 0.6f + uc * 0.4f;

    g_Ua = fUa;
    g_Ub = fUb;
    g_Uc = fUc;
#endif
}

//=====================================================================
//  相电压 → dq → Iq → 力矩
//=====================================================================
static void updateTorqueEstimate(void) {
    float Ud, Uq;
    abcToDq(g_Ua, g_Ub, g_Uc, motor.electrical_angle, Ud, Uq);
    g_Ud_est = Ud;
    g_Uq_est = Uq;

    float iq = Uq / MOTOR_R_PHASE;
    float tq = MOTOR_KT * iq;

    static float fIq = 0.0f, fTq = 0.0f;
    fIq = fIq * 0.8f + iq * 0.2f;
    fTq = fTq * 0.8f + tq * 0.2f;

    g_Iq_est = fIq;
    g_T_est  = fTq;
}

//=====================================================================
//  周期性刷新 UI
//=====================================================================
static void update_ui(void) {
    char buf[32];

    snprintf(buf, sizeof(buf), "Simple FOC %.1fV", g_bus_v);
    lv_label_set_text(ui_status, buf);

    if (!motorRunning) {
        lv_label_set_text(ui_l1, "STOPPED");
        lv_label_set_text(ui_l2, "BTN1: mode");
        lv_label_set_text(ui_l3, "BTN2: run");
    } else {
        switch (currentMode) {
            case MODE_SPEED: {
                float spd = motor.shaft_velocity;
                float tgt = spd_cmd;                // ★ 显示平滑后的目标
                snprintf(buf, sizeof(buf), "SPD %6.1f rad/s", spd);
                lv_label_set_text(ui_l1, buf);
                snprintf(buf, sizeof(buf), "TRG %6.1f rad/s", tgt);
                lv_label_set_text(ui_l2, buf);
                snprintf(buf, sizeof(buf), "ERR %6.1f rad/s", tgt - spd);
                lv_label_set_text(ui_l3, buf);
                break;
            }
            case MODE_ANGLE: {
                float pos = (sensor.getAngle() - zero_angle) * RAD_TO_DEG;
                float tgt = (target_smooth    - zero_angle) * RAD_TO_DEG;
                snprintf(buf, sizeof(buf), "POS %6.1f deg", pos);
                lv_label_set_text(ui_l1, buf);
                snprintf(buf, sizeof(buf), "TRG %6.1f deg", tgt);
                lv_label_set_text(ui_l2, buf);
                snprintf(buf, sizeof(buf), "ERR %6.1f deg", tgt - pos);
                lv_label_set_text(ui_l3, buf);
                break;
            }
            case MODE_TORQUE: {
                snprintf(buf, sizeof(buf), "Uq %6.2f V", g_Uq_est);
                lv_label_set_text(ui_l1, buf);
                snprintf(buf, sizeof(buf), "Iq %6.2f A", g_Iq_est);
                lv_label_set_text(ui_l2, buf);
                snprintf(buf, sizeof(buf), "TRQ %6.3fNm", g_T_est);
                lv_label_set_text(ui_l3, buf);
                break;
            }
        }
    }

    snprintf(buf, sizeof(buf), "Ua%5.1f  Ub%5.1f", g_Ua, g_Ub);
    lv_label_set_text(ui_uv, buf);
    snprintf(buf, sizeof(buf), "Uc%5.1f V   %s", g_Uc, modeShort());
    lv_label_set_text(ui_uc, buf);
}

//=====================================================================
//  进入模式前的准备
//=====================================================================
void prepareModeStart() {
    switch (currentMode) {
        case MODE_SPEED:
            motor.controller = MotionControlType::velocity;
            enc_count        = 0;
            spd_cmd          = 0.0f;            // ★ 复位平滑指令
            motor.target     = 0.0f;
            break;

        case MODE_ANGLE:
            motor.controller = MotionControlType::angle;
            zero_angle       = sensor.getAngle();
            target_smooth    = zero_angle;
            motor.target     = zero_angle;
            enc_count        = 0;
            break;

        case MODE_TORQUE:
            motor.controller        = MotionControlType::torque;
            motor.torque_controller = TorqueControlType::voltage;
            motor.target            = 0.0f;
            enc_count               = 0;
            break;
    }
    Serial.printf("=== Mode: %s ===\n", modeLong());
}

//=====================================================================
//  按键处理
//=====================================================================
void handleButtons() {
    static bool lastMode = HIGH, lastRun = HIGH;
    static uint32_t lastModeT = 0, lastRunT = 0;
    uint32_t now = millis();

    bool curMode = digitalRead(BTN_MODE);
    bool curRun  = digitalRead(BTN_RUN);

    if (lastMode == HIGH && curMode == LOW && now - lastModeT > 200) {
        lastModeT = now;
        motorRunning = false;
        currentMode  = (Mode)((currentMode + 1) % 3);
        updateRGB();
        Serial.printf("BTN1 -> stop & switch to %s\n", modeLong());
    }

    if (lastRun == HIGH && curRun == LOW && now - lastRunT > 200) {
        lastRunT = now;
        if (!motorRunning) {
            g_fault = false;
            prepareModeStart();
            motorRunning = true;
            updateRGB();
            Serial.printf("BTN2 -> start in %s\n", modeLong());
        }
    }

    lastMode = curMode;
    lastRun  = curRun;
}

//=====================================================================
//  FOC 实时任务（核心 0，1kHz）
//=====================================================================
void focTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);
    bool lastRunning = false;

    for (;;) {
        if (motorRunning != lastRunning) {
            if (motorRunning) {
                motor.enable();
                Serial.println("Motor ENABLED");
            } else {
                motor.disable();
                Serial.println("Motor DISABLED");
            }
            lastRunning = motorRunning;
        }

        if (motorRunning) {
            switch (currentMode) {
                // ★★★ 速度环：编码器 → 目标速度（含绝对上限 + 斜率限制）
                case MODE_SPEED: {
                    // ① 编码器映射
                    float t   = (float)enc_count / (float)ENC_RANGE;
                    float raw = t * SPEED_MAX_CMD;

                    // ② 绝对上限（编码器到端点也不会突破 SPEED_ABS_LIMIT）
                    if (raw >  SPEED_ABS_LIMIT) raw =  SPEED_ABS_LIMIT;
                    if (raw < -SPEED_ABS_LIMIT) raw = -SPEED_ABS_LIMIT;

                    // ③ 斜率限制：每 1ms 最多变化 SPEED_RAMP × 0.001
                    const float step = SPEED_RAMP * 0.001f;   // rad/s per tick
                    float d = raw - spd_cmd;
                    if (d >  step) d =  step;
                    if (d < -step) d = -step;
                    spd_cmd += d;

                    motor.target = spd_cmd;
                    break;
                }

                // 位置环
                case MODE_ANGLE: {
                    float offset = (float)enc_count / (float)ENC_RANGE
                                   * ANGLE_RANGE_RAD;
                    float raw = zero_angle + offset;
                    target_smooth += TARGET_SMOOTH_ALPHA * (raw - target_smooth);
                    motor.target = target_smooth;
                    break;
                }

                // 转矩环
                case MODE_TORQUE: {
                    float t = (float)enc_count / (float)ENC_RANGE;
                    motor.target = t * UQ_MAX;
                    break;
                }
            }

            motor.loopFOC();
            motor.move();
        }

        if (digitalRead(DRV_FAULT) == LOW && !g_fault) {
            motor.disable();
            motorRunning = false;
            g_fault      = true;
            updateRGB();
            Serial.println("!! DRV FAULT !!");
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

//=====================================================================
//  setup()
//=====================================================================
void setup() {
    pixels.begin();
    pixels.setBrightness(30);
    setRGB(0, 0, 255);

    Serial.begin(115200);
    delay(100);
    Serial.printf("Tick Rate: %d\n", configTICK_RATE_HZ);
    Serial.println("=== FOC 3-Mode UI ===");
    Serial.println("BTN1 = stop & switch mode");
    Serial.println("BTN2 = start motor");
    Serial.printf("Speed: MAX=%.1f ABS=%.1f RAMP=%.1f rad/s^2\n",
                  SPEED_MAX_CMD, SPEED_ABS_LIMIT, SPEED_RAMP);

    pinMode(BL_PIN, OUTPUT);
    digitalWrite(BL_PIN, HIGH);

    pinMode(BTN_MODE, INPUT_PULLUP);
    pinMode(BTN_RUN,  INPUT_PULLUP);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);

#if USE_ADC_VOLTAGE_SAMPLING
    analogReadResolution(12);
    pinMode(VOLT_UA_PIN, INPUT);
    pinMode(VOLT_UB_PIN, INPUT);
#endif

    hspi.begin(TFT_SCLK, -1, TFT_MOSI);
    hspi.setFrequency(40000000);
    tft.initR(INITR_MINI160x80);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_WHITE);

    lv_init();
    disp = lv_display_create(160, 80);
    lv_display_set_buffers(disp, lv_buf, NULL,
                           sizeof(lv_buf),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, my_disp_flush);

    build_ui();
    lv_timer_handler();

    Wire.begin(I2C_SDA, I2C_SCL);
    sensor.init();
    sensor.min_elapsed_time = 0.001f;
    motor.linkSensor(&sensor);

    pinMode(DRV_FAULT, INPUT_PULLUP);
    pinMode(DRV_SLEEP, OUTPUT);
    digitalWrite(DRV_SLEEP, HIGH);

    driver.voltage_power_supply = 12.0f;
    driver.pwm_frequency = 20000;
    driver.init();
    motor.linkDriver(&driver);

    motor.voltage_limit  = 6.0f;
    motor.velocity_limit = SPEED_ABS_LIMIT * 1.2f;   // ★ 内部限速略高于指令上限

    motor.PID_velocity.P   = 0.20f;
    motor.PID_velocity.I   = 0.0f;
    motor.PID_velocity.D   = 0.0f;
    motor.LPF_velocity.Tf  = 0.03f;

    motor.P_angle.P            = 10.0f;
    motor.P_angle.I            = 0.0f;
    motor.P_angle.D            = 0.0f;
    motor.P_angle.output_ramp  = 100.0f;
    motor.LPF_angle.Tf         = 0.05f;

    motor.torque_controller = TorqueControlType::voltage;

    setRGB(255, 128, 0);
    motor.init();
    motor.initFOC();

    Serial.printf("Zero Angle: %.4f\n", motor.zero_electric_angle);
    Serial.printf("Sensor Dir: %s\n",
                  motor.sensor_direction == Direction::CW ? "CW" : "CCW");

    motor.controller = MotionControlType::velocity;
    motor.target     = 0.0f;
    currentMode      = MODE_SPEED;
    motorRunning     = false;
    zero_angle       = sensor.getAngle();
    target_smooth    = zero_angle;
    spd_cmd          = 0.0f;

    motor.disable();

    xTaskCreatePinnedToCore(focTask, "FOC", 8192, NULL, 5, &focTaskHandle, 0);

    updateRGB();
    Serial.println("System ready. Press BTN2 to start.");
}

//=====================================================================
//  loop()  核心 1
//=====================================================================
void loop() {
    static uint32_t last_tick = 0;
    uint32_t now = millis();
    if (now != last_tick) {
        lv_tick_inc(now - last_tick);
        last_tick = now;
    }

    handleButtons();
    lv_timer_handler();

    static uint32_t ui_tmr = 0;
    if (now - ui_tmr >= 200) {
        ui_tmr = now;

        samplePhaseVoltages();
        updateTorqueEstimate();
        update_ui();

        Serial.printf("[%s] run=%d enc=%4d  cmd=%.2f tgt=%.2f  Uq=%.2f Iq=%.2f T=%.4f\n",
                      modeShort(), motorRunning, enc_count,
                      spd_cmd, motor.target,
                      g_Uq_est, g_Iq_est, g_T_est);
    }

    delay(5);
}
