/* Ethernet TCP server + PCNT encoder + deterministic motor control loop. */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "ethernet_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "sdkconfig.h"

static const char *TAG = "eth_rt_motor";

#define PORT 5000
#define FRAME_LEN 12
#define BOOT_BUTTON_PIN 0 // Chân mặc định của nút BOOT
#define CMD_READ_ADC 0x01
#define CMD_WRITE_PWM 0x02
#define CMD_LED 0x03
#define CMD_STATUS 0x04
#define CMD_IDN 0x05
#define CMD_PING 0x06
#define CMD_READ_ENCODER 0x07
#define CMD_SET_TARGET_RPM 0x08
#define CMD_SET_KP 0x09
#define CMD_SET_KI 0x0A
#define CMD_PI_ENABLE 0x0B
#define CMD_READ_PWM 0x0C

#define STATUS_SEL_FLAGS 0x00
#define STATUS_SEL_TARGET_RPM 0x01
#define STATUS_SEL_DEADLINE_MISSES 0x02
#define STATUS_SEL_MAX_RT_EXEC_US 0x03
#define STATUS_SEL_RX_FRAMES 0x04
#define STATUS_SEL_CRC_ERRORS 0x05

#define STATUS_OK 0x00
#define STATUS_ERROR 0x01

#define ERR_BAD_CMD 0x01
#define ERR_BAD_FRAME 0x02
#define ERR_BAD_CRC 0x03

#define PULSES_PER_REV 250

#define MOTOR_IN1_PIN 26
#define MOTOR_IN2_PIN 25
#define MOTOR_ENA_PIN 27

#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR_PWM_DUTY_RES LEDC_TIMER_8_BIT
#define MOTOR_PWM_FREQ_HZ 5000
#define MOTOR_PWM_MAX_DUTY 255
#define MOTOR_PWM_MIN_RUN 80

#define MOTOR_KICK_START_PWM 150
#define MOTOR_KICK_START_MS 300

#define RT_PERIOD_MS 5
#define RT_DEADLINE_BUDGET_US 500
#define PI_DIVIDER 10
#define PI_PERIOD_MS (RT_PERIOD_MS * PI_DIVIDER)
#define COMMAND_TIMEOUT_MS 1000
#define INTEGRAL_LIMIT 20000

#define CONTROL_FLAG_PI_ENABLED 0x0001

#define RT_TIMER_SETUP_PRIORITY 23
#define TCP_TASK_PRIORITY 6

#if CONFIG_FREERTOS_UNICORE
#define RT_CORE_ID 0
#define TCP_CORE_ID 0
#else
#define RT_CORE_ID APP_CPU_NUM
#define TCP_CORE_ID PRO_CPU_NUM
#endif

typedef struct {
    int rpm;
    int pwm;
    int target_rpm;
    int kp_x100;
    int ki_x100;
    bool pi_enabled;
    uint32_t rx_frames;
    uint32_t deadline_misses;
    int64_t last_command_time_us;
    int64_t max_rt_exec_us;
} motor_state_t;

static pcnt_unit_handle_t pcnt_unit = NULL;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static motor_state_t g_state = {
    .kp_x100 = 50,
    .ki_x100 = 10,
};
static volatile int64_t last_exec_us = 0;
static int s_pi_integral = 0;
static gptimer_handle_t s_rt_timer = NULL;

static uint8_t calc_crc(const uint8_t *frame)
{
    uint8_t crc = 0;
    for (int i = 0; i < 10; i++) {
        crc ^= frame[i];
    }
    return crc;
}

static uint16_t frame_value(const uint8_t *frame)
{
    return ((uint16_t)frame[5] << 8) | frame[6];
}

static uint8_t frame_opt1(const uint8_t *frame)
{
    return frame[7];
}

static void state_snapshot(motor_state_t *out)
{
    portENTER_CRITICAL(&state_lock);
    *out = g_state;
    portEXIT_CRITICAL(&state_lock);
}

static void state_set_pwm(int pwm)
{
    portENTER_CRITICAL(&state_lock);
    g_state.pwm = pwm;
    portEXIT_CRITICAL(&state_lock);
}

static void state_note_command(void)
{
    portENTER_CRITICAL(&state_lock);
    g_state.rx_frames++;
    g_state.last_command_time_us = esp_timer_get_time();
    portEXIT_CRITICAL(&state_lock);
}


static void set_pi_enabled(bool enabled)
{
    portENTER_CRITICAL(&state_lock);
    g_state.pi_enabled = enabled;
    portEXIT_CRITICAL(&state_lock);
    if (!enabled) {
        s_pi_integral = 0;
    }
}

static void set_target_rpm(int target)
{
    portENTER_CRITICAL(&state_lock);
    g_state.target_rpm = target;
    portEXIT_CRITICAL(&state_lock);
    s_pi_integral = 0;
}

static void set_gains(int kp_x100, int ki_x100)
{
    portENTER_CRITICAL(&state_lock);
    g_state.kp_x100 = kp_x100;
    g_state.ki_x100 = ki_x100;
    portEXIT_CRITICAL(&state_lock);
    s_pi_integral = 0;
}

static void set_motor_output(int pwm)
{
    if (pwm < 0) {
        pwm = 0;
    } else if (pwm > MOTOR_PWM_MAX_DUTY) {
        pwm = MOTOR_PWM_MAX_DUTY;
    }

    if (pwm == 0) {
        gpio_set_level(MOTOR_IN1_PIN, 0);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    } else {
        gpio_set_level(MOTOR_IN1_PIN, 1);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    }

    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL));
    state_set_pwm(pwm);
}

static void stop_motor_safely(void)
{
    set_pi_enabled(false);
    set_motor_output(0);
}

static void init_encoder(void)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = 32,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_HOLD,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

static void init_motor(void)
{
    gpio_config_t motor_gpio_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_PIN) | (1ULL << MOTOR_IN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&motor_gpio_conf));

    ledc_timer_config_t ledc_timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .timer_num = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_DUTY_RES,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_ENA_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    set_motor_output(0);
}

static int IRAM_ATTR sample_rpm_isr(int period_ms)
{
    int pulse_count = 0;
    if (pcnt_unit_get_count(pcnt_unit, &pulse_count) != ESP_OK) {
        return 0;
    }
    (void)pcnt_unit_clear_count(pcnt_unit);

    // Tính RPM dựa trên thời gian truyền vào
    int rpm = (pulse_count * 60000) / (PULSES_PER_REV * period_ms);
    if (rpm < 0) {
        rpm = 0;
    }
    return rpm;
}

static int IRAM_ATTR clamp_pwm(int pwm)
{
    if (pwm < 0) {
        return 0;
    }
    if (pwm > MOTOR_PWM_MAX_DUTY) {
        return MOTOR_PWM_MAX_DUTY;
    }
    return pwm;
}

static void IRAM_ATTR set_motor_output_isr(int pwm)
{
    pwm = clamp_pwm(pwm);

    if (pwm == 0) {
        gpio_set_level(MOTOR_IN1_PIN, 0);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    } else {
        gpio_set_level(MOTOR_IN1_PIN, 1);
        gpio_set_level(MOTOR_IN2_PIN, 0);
    }

    (void)ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, pwm);
    (void)ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL);

    portENTER_CRITICAL_ISR(&state_lock);
    g_state.pwm = pwm;
    portEXIT_CRITICAL_ISR(&state_lock);
}

static int IRAM_ATTR pi_step_common(int target, int rpm, int kp_x100, int ki_x100)
{
    int error = target - rpm;
    s_pi_integral += error;

    if (s_pi_integral > INTEGRAL_LIMIT) {
        s_pi_integral = INTEGRAL_LIMIT;
    } else if (s_pi_integral < -INTEGRAL_LIMIT) {
        s_pi_integral = -INTEGRAL_LIMIT;
    }

    int proportional = kp_x100 * error;
    int integral = (ki_x100 * s_pi_integral * PI_PERIOD_MS) / 1000;
    int pwm = (proportional + integral) / 100;

    if (pwm > MOTOR_PWM_MAX_DUTY) {
        pwm = MOTOR_PWM_MAX_DUTY;
    } else if (pwm < 0) {
        pwm = 0;
    }

    if (pwm > 0 && pwm < MOTOR_PWM_MIN_RUN) {
        pwm = MOTOR_PWM_MIN_RUN;
    }
    return pwm;
}

static bool IRAM_ATTR rt_timer_on_alarm(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata,
                                        void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    static int pi_countdown = PI_DIVIDER;
    static int kick_ticks_remaining = 0;
    static int current_rpm = 0; // Biến lưu RPM mượt hơn

    int64_t start_us = esp_timer_get_time();
    if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
        
        esp_rom_delay_us(600); 
    }

    // CHỈ ĐỌC VÀ CLEAR ENCODER MỖI 50ms (PI_PERIOD_MS)
    pi_countdown--;
    if (pi_countdown <= 0) {
        current_rpm = sample_rpm_isr(PI_PERIOD_MS);
    }

    motor_state_t st;
    portENTER_CRITICAL_ISR(&state_lock);
    g_state.rpm = current_rpm;
    st = g_state;
    portEXIT_CRITICAL_ISR(&state_lock);

    bool command_timeout = false;
    if (st.pi_enabled && st.last_command_time_us > 0) {
        command_timeout = (start_us - st.last_command_time_us) > (COMMAND_TIMEOUT_MS * 1000LL);
    }

    if (command_timeout) {
        s_pi_integral = 0;
        kick_ticks_remaining = 0;
        pi_countdown = PI_DIVIDER; 
        portENTER_CRITICAL_ISR(&state_lock);
        g_state.pi_enabled = false;
        portEXIT_CRITICAL_ISR(&state_lock);
        set_motor_output_isr(0);
    } else if (pi_countdown <= 0) {
        pi_countdown = PI_DIVIDER; // Reset cho chu kỳ 50ms tiếp theo

        // Chạy vòng lặp PI
        if (st.pi_enabled) {
            if (st.target_rpm <= 0) {
                s_pi_integral = 0;
                kick_ticks_remaining = 0;
                set_motor_output_isr(0);
            } else if (current_rpm == 0 && st.pwm == 0 && kick_ticks_remaining == 0) {
                kick_ticks_remaining = MOTOR_KICK_START_MS / PI_PERIOD_MS;
                if (kick_ticks_remaining < 1) {
                    kick_ticks_remaining = 1;
                }
                set_motor_output_isr(MOTOR_KICK_START_PWM);
            } else if (kick_ticks_remaining > 0) {
                kick_ticks_remaining--;
                set_motor_output_isr(MOTOR_KICK_START_PWM);
            } else {
                // Đưa current_rpm (chính xác hơn) vào tính toán PI
                set_motor_output_isr(pi_step_common(st.target_rpm, current_rpm, st.kp_x100, st.ki_x100));
            }
        }
    }

    // Tính toán thời gian thực thi (giữ nguyên)
    int64_t exec_us = esp_timer_get_time() - start_us;
    last_exec_us = exec_us;
   portENTER_CRITICAL_ISR(&state_lock);
    if (exec_us > g_state.max_rt_exec_us) {
        g_state.max_rt_exec_us = exec_us;
    }
    if (exec_us > RT_DEADLINE_BUDGET_US) {
        g_state.deadline_misses++;
    }
    portEXIT_CRITICAL_ISR(&state_lock);

    return false;
}

static void rt_timer_setup_task(void *pvParameters)
{
    (void)pvParameters;

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_rt_timer));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = rt_timer_on_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_rt_timer, &callbacks, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = RT_PERIOD_MS * 1000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_rt_timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(s_rt_timer));
    ESP_ERROR_CHECK(gptimer_start(s_rt_timer));

    vTaskDelete(NULL);
}

static int recv_exact(int sock, uint8_t *buffer, size_t len)
{
    size_t got = 0;
    while (got < len) {
        int ret = recv(sock, buffer + got, len - got, 0);
        if (ret <= 0) {
            return ret;
        }
        got += ret;
    }
    return (int)got;
}

static bool validate_request(const uint8_t *rx, uint8_t *error)
{
    if (rx[0] != '@' || rx[1] != 'S' || rx[2] != 'E' || rx[3] != 'N' || rx[11] != '#') {
        *error = ERR_BAD_FRAME;
        return false;
    }
    if (calc_crc(rx) != rx[10]) {
        *error = ERR_BAD_CRC;
        return false;
    }
    return true;
}

static void build_response(uint8_t *tx, uint8_t cmd, uint16_t value, uint8_t status, uint8_t error, uint8_t seq)
{
    tx[0] = '@';
    tx[1] = 'R';
    tx[2] = 'E';
    tx[3] = 'C';
    tx[4] = cmd;
    tx[5] = (value >> 8) & 0xFF;
    tx[6] = value & 0xFF;
    tx[7] = status;
    tx[8] = error;
    tx[9] = seq;
    tx[10] = calc_crc(tx);
    tx[11] = '#';
}

static uint16_t status_value(uint8_t selector, const motor_state_t *st)
{
    switch (selector) {
    case STATUS_SEL_FLAGS: {
        uint16_t flags = 0;
        if (st->pi_enabled) {
            flags |= CONTROL_FLAG_PI_ENABLED;
        }
        return flags;
    }
    case STATUS_SEL_TARGET_RPM:
        return st->target_rpm < 0 ? 0 : (st->target_rpm > 65535 ? 65535 : st->target_rpm);
    case STATUS_SEL_DEADLINE_MISSES:
        return st->deadline_misses > 65535 ? 65535 : (uint16_t)st->deadline_misses;
    case STATUS_SEL_MAX_RT_EXEC_US:
        return st->max_rt_exec_us > 65535 ? 65535 : (uint16_t)st->max_rt_exec_us;
    case STATUS_SEL_RX_FRAMES:
        return st->rx_frames > 65535 ? 65535 : (uint16_t)st->rx_frames;
    default:
        return 0;
    }
}

static void handle_command(const uint8_t *rx, uint16_t *value, uint8_t *status, uint8_t *error)
{
    const uint8_t cmd = rx[4];
    const uint16_t request_value = frame_value(rx);
    const uint8_t opt1 = frame_opt1(rx);
    motor_state_t st;

    *value = request_value;
    *status = STATUS_OK;
    *error = 0;

    state_note_command();

    switch (cmd) {
    case CMD_WRITE_PWM: {
        int pwm = request_value > MOTOR_PWM_MAX_DUTY ? MOTOR_PWM_MAX_DUTY : request_value;
        set_pi_enabled(false);
        set_motor_output(pwm);
        *value = pwm;
        break;
    }
    case CMD_SET_TARGET_RPM:
        set_target_rpm(request_value);
        break;
    case CMD_SET_KP:
        state_snapshot(&st);
        set_gains(request_value, st.ki_x100);
        break;
    case CMD_SET_KI:
        state_snapshot(&st);
        set_gains(st.kp_x100, request_value);
        break;
    case CMD_PI_ENABLE:
        set_pi_enabled(request_value != 0);
        if (request_value == 0) {
            set_motor_output(0);
        }
        *value = request_value != 0 ? 1 : 0;
        break;
    case CMD_READ_PWM:
        state_snapshot(&st);
        *value = st.pwm;
        break;
    case CMD_READ_ENCODER:
        state_snapshot(&st);
        *value = st.rpm < 0 ? 0 : (st.rpm > 65535 ? 65535 : st.rpm);
        break;
    case CMD_STATUS:
        state_snapshot(&st);
        *value = status_value(opt1, &st);
        break;
    case CMD_READ_ADC:
    case CMD_LED:
    case CMD_PING:
    case CMD_IDN:
        break;
    default:
        *status = STATUS_ERROR;
        *error = ERR_BAD_CMD;
        *value = 0;
        break;
    }
}

static void tcp_server_task(void *pvParameters)
{
    (void)pvParameters;

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() failed errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed errno=%d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "listen() failed errno=%d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ESP_LOGI(TAG, "TCP command server listening on port %d", PORT);
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "accept() failed errno=%d", errno);
            continue;
        }

        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        ESP_LOGI(TAG, "Supervisor connected");

        while (1) {
            uint8_t rx[FRAME_LEN] = {0};
            uint8_t tx[FRAME_LEN] = {0};
            int len = recv_exact(sock, rx, sizeof(rx));
            if (len <= 0) {
                ESP_LOGW(TAG, "Supervisor disconnected");
                stop_motor_safely();
                break;
            }

            uint16_t value = 0;
            uint8_t status = STATUS_OK;
            uint8_t error = 0;
            uint8_t cmd = rx[4];
            uint8_t seq = rx[9];

            if (validate_request(rx, &error)) {
                handle_command(rx, &value, &status, &error);
            } else {
                status = STATUS_ERROR;
                value = 0;
            }

            build_response(tx, cmd, value, status, error, seq);
            if (send(sock, tx, sizeof(tx), 0) < 0) {
                ESP_LOGE(TAG, "send() failed errno=%d", errno);
                stop_motor_safely();
                break;
            }
        }

        shutdown(sock, 0);
        close(sock);
    }
}

static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet link up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet link down");
        stop_motor_safely();
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet stopped");
        stop_motor_safely();
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;
    ESP_LOGI(TAG, "Ethernet IP: " IPSTR, IP2STR(&ip_info->ip));
}

void app_main(void)
{
    gpio_reset_pin(BOOT_BUTTON_PIN);
    gpio_set_direction(BOOT_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOOT_BUTTON_PIN, GPIO_PULLUP_ONLY);

    init_encoder();
    init_motor();

    portENTER_CRITICAL(&state_lock);
    g_state.last_command_time_us = esp_timer_get_time();
    portEXIT_CRITICAL(&state_lock);

    xTaskCreatePinnedToCore(
        rt_timer_setup_task,
        "rt_timer_setup",
        4096,
        NULL,
        RT_TIMER_SETUP_PRIORITY,
        NULL,
        RT_CORE_ID);

    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles = NULL;
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_port_cnt));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *eth_netifs[eth_port_cnt];
    esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];

    if (eth_port_cnt == 1) {
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        eth_netifs[0] = esp_netif_new(&cfg);
        eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
        ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netifs[0]));

        esp_netif_ip_info_t ip_info;
        IP4_ADDR(&ip_info.ip, 192, 168, 2, 177);
        IP4_ADDR(&ip_info.gw, 192, 168, 2, 1);
        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
        ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netifs[0], &ip_info));
        ESP_LOGI(TAG, "Static IP set to 192.168.2.177");
    } else {
        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
        esp_netif_config_t cfg_spi = {
            .base = &esp_netif_config,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
        };
        char if_key_str[10];
        char if_desc_str[10];
        for (int i = 0; i < eth_port_cnt; i++) {
            snprintf(if_key_str, sizeof(if_key_str), "ETH_%d", i);
            snprintf(if_desc_str, sizeof(if_desc_str), "eth%d", i);
            esp_netif_config.if_key = if_key_str;
            esp_netif_config.if_desc = if_desc_str;
            esp_netif_config.route_prio -= i * 5;
            eth_netifs[i] = esp_netif_new(&cfg_spi);
            eth_netif_glues[i] = esp_eth_new_netif_glue(eth_handles[i]);
            ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[i], eth_netif_glues[i]));
        }
    }

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
    }

    xTaskCreatePinnedToCore(
        tcp_server_task,
        "tcp_server",
        4096,
        NULL,
        TCP_TASK_PRIORITY,
        NULL,
        TCP_CORE_ID);
}
