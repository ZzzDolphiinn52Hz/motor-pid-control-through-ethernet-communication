# Hướng sửa hệ thống ESP32 Ethernet theo hard realtime công nghiệp

Nếu đi theo hướng **hard realtime công nghiệp**, thì phải sửa theo nguyên tắc này:

> **ESP32 phải tự điều khiển hoàn toàn theo chu kỳ cố định. Ethernet/TCP/GUI chỉ được dùng để cấu hình, giám sát, ghi log, không được nằm trong vòng điều khiển.**

Code hiện tại đã đi đúng hướng một phần, vì vòng điều khiển nằm trên ESP32, còn GUI Python chỉ gửi lệnh và đọc trạng thái. Nhưng để tiệm cận “hard realtime công nghiệp”, cần sửa khá nhiều.

---

## 1. Trước hết: TCP Ethernet không nên dùng làm vòng điều khiển hard realtime

Với công nghiệp thật, không nên làm kiểu:

```text
PC đọc RPM → PC tính PI → PC gửi PWM qua TCP → ESP32 xuất PWM
```

Cách đó không hard realtime vì:

- TCP có jitter.
- Hệ điều hành PC không đảm bảo chu kỳ chính xác.
- Python/Tkinter càng không phù hợp để đóng vòng điều khiển.
- Mỗi lần monitor trong GUI hiện tại còn gửi nhiều request riêng: đọc encoder, PWM, flags, target, deadline miss, max ISR, RX frames.

Hướng đúng là:

```text
ESP32 tự chạy timer realtime
ESP32 tự đọc encoder
ESP32 tự tính PI
ESP32 tự xuất PWM
PC/GUI chỉ gửi target, Kp, Ki, enable/disable, đọc trạng thái
```

Nếu là hệ công nghiệp nghiêm ngặt hơn nữa, thay TCP bằng:

```text
EtherCAT / PROFINET IRT / CANopen / Modbus RTU công nghiệp / PLC local control
```

Còn TCP Ethernet chỉ nên dùng cho HMI, SCADA, cấu hình và giám sát.

---

## 2. Sửa kiến trúc realtime trong ESP32

Hiện tại file C đang dùng:

```c
#define RT_PERIOD_MS 5
#define RT_DEADLINE_BUDGET_US 500
#define PI_DIVIDER 10
#define PI_PERIOD_MS (RT_PERIOD_MS * PI_DIVIDER)
```

Tức là timer 5 ms, nhưng PI chạy mỗi 50 ms.

Với hard realtime hơn, nên tổ chức lại như sau:

```text
GPTimer ISR
    ↓
Chỉ đánh thức task realtime
    ↓
Realtime control task ưu tiên cao
    ↓
Đọc encoder → tính RPM → PI → cập nhật PWM
    ↓
Ghi snapshot telemetry
```

Không nên để callback timer làm quá nhiều việc.

---

## 3. Sửa quan trọng nhất: làm ISR thật ngắn

Hiện tại trong `rt_timer_on_alarm()` có các thao tác nặng:

- Gọi `esp_timer_get_time()`.
- Đọc GPIO.
- Có `esp_rom_delay_us(600)`.
- Đọc/clear PCNT.
- Tính PI.
- Cập nhật LEDC.
- Vào critical section.
- Cập nhật nhiều biến trạng thái.

Đặc biệt đoạn delay:

```c
if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
    esp_rom_delay_us(600); 
}
```

Trong khi budget realtime là 500 µs, chỉ riêng delay 600 µs đã vượt deadline.

Nên sửa thành:

```c
static TaskHandle_t rt_task_handle = NULL;

static bool IRAM_ATTR rt_timer_on_alarm(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    BaseType_t high_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(rt_task_handle, &high_task_woken);

    return high_task_woken == pdTRUE;
}
```

Sau đó tạo task realtime riêng:

```c
static void rt_control_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t start_us = esp_timer_get_time();

        rt_control_step();

        int64_t exec_us = esp_timer_get_time() - start_us;

        portENTER_CRITICAL(&state_lock);
        if (exec_us > g_state.max_rt_exec_us) {
            g_state.max_rt_exec_us = exec_us;
        }
        if (exec_us > RT_DEADLINE_BUDGET_US) {
            g_state.deadline_misses++;
        }
        portEXIT_CRITICAL(&state_lock);
    }
}
```

Trong `app_main()` tạo task này trước hoặc ngay sau timer setup:

```c
xTaskCreatePinnedToCore(
    rt_control_task,
    "rt_control",
    4096,
    NULL,
    24,
    &rt_task_handle,
    RT_CORE_ID
);
```

---

## 4. Tách rõ realtime task và TCP task

Hiện tại bạn đã có:

```c
#define RT_TIMER_SETUP_PRIORITY 23
#define TCP_TASK_PRIORITY 6
```

Đây là hướng đúng: realtime ưu tiên cao, TCP ưu tiên thấp.

Nhưng nên chỉnh thành rõ hơn:

```c
#define RT_CONTROL_PRIORITY 24
#define TCP_TASK_PRIORITY   6
```

Với ESP32 2 core, nên ghim như sau:

```text
Core 1: realtime control task
Core 0: TCP/IP, Ethernet, GUI command task
```

Bạn đang có hướng ghim task theo core rồi:

```c
#define RT_CORE_ID APP_CPU_NUM
#define TCP_CORE_ID PRO_CPU_NUM
```

Cách này tốt. Giữ lại.

---

## 5. Không cho TCP ghi trực tiếp vào motor trong đường realtime

Hiện tại khi nhận `CMD_WRITE_PWM`, TCP task gọi:

```c
set_pi_enabled(false);
set_motor_output(pwm);
```

Tức là TCP task có thể trực tiếp cập nhật PWM. Với hard realtime, không nên để TCP task đụng trực tiếp output motor.

Nên đổi thành:

```text
TCP task chỉ ghi command/request
Realtime task mới là nơi duy nhất cập nhật PWM thật
```

Ví dụ tạo struct command:

```c
typedef struct {
    volatile int manual_pwm;
    volatile int target_rpm;
    volatile int kp_x100;
    volatile int ki_x100;
    volatile bool pi_enabled;
    volatile bool manual_pwm_pending;
    volatile int64_t last_command_time_us;
} control_command_t;

static control_command_t g_cmd = {
    .kp_x100 = 50,
    .ki_x100 = 10,
};
```

Khi TCP nhận lệnh PWM:

```c
case CMD_WRITE_PWM:
    portENTER_CRITICAL(&state_lock);
    g_cmd.pi_enabled = false;
    g_cmd.manual_pwm = pwm;
    g_cmd.manual_pwm_pending = true;
    g_cmd.last_command_time_us = esp_timer_get_time();
    portEXIT_CRITICAL(&state_lock);
    *value = pwm;
    break;
```

Trong realtime task:

```c
if (cmd.manual_pwm_pending) {
    cmd.manual_pwm_pending = false;
    set_motor_output_rt(cmd.manual_pwm);
}
```

Ý nghĩa: **chỉ realtime task mới được xuất PWM thật**.

---

## 6. Watchdog phải áp dụng cho cả Manual PWM và PI

Hiện tại timeout chủ yếu đang kiểm tra khi PI enabled. Nhưng trong công nghiệp, nếu mất truyền thông thì dù đang manual PWM cũng phải dừng motor.

Nên sửa logic thành:

```c
bool command_timeout = false;

if (st.last_command_time_us > 0) {
    command_timeout = (now_us - st.last_command_time_us) > COMMAND_TIMEOUT_MS * 1000LL;
}

if (command_timeout) {
    disable_pi();
    set_motor_output_rt(0);
    set_fault(COMM_TIMEOUT_FAULT);
}
```

Không nên chỉ timeout khi `pi_enabled == true`.

Nên có thêm trạng thái fault:

```c
#define FAULT_NONE          0x0000
#define FAULT_COMM_TIMEOUT  0x0001
#define FAULT_OVERSPEED     0x0002
#define FAULT_ENCODER_LOST  0x0004
#define FAULT_DEADLINE_MISS 0x0008
```

Khi có fault nặng:

```c
motor_pwm = 0;
pi_enabled = false;
fault_latched = true;
```

Muốn chạy lại thì GUI phải gửi lệnh reset fault riêng.

---

## 7. Không dùng `ESP_ERROR_CHECK()` trong đường realtime

Trong hàm thường thì được. Nhưng trong realtime path không nên dùng:

```c
ESP_ERROR_CHECK(...)
```

Vì nếu lỗi xảy ra có thể abort chương trình, log, hoặc làm mất tính xác định thời gian.

Trong realtime path nên dùng kiểu:

```c
esp_err_t err = ledc_set_duty(...);
if (err != ESP_OK) {
    fault_flags |= FAULT_PWM_DRIVER;
    return;
}
```

Nhưng tốt hơn nữa là cấu hình PWM một lần lúc init, sau đó realtime task chỉ cập nhật duty.

---

## 8. Giảm hoặc bỏ critical section dài

Hiện tại nhiều hàm dùng:

```c
portENTER_CRITICAL(&state_lock);
...
portEXIT_CRITICAL(&state_lock);
```

Nếu TCP task và realtime task cùng tranh lock, có thể gây jitter.

Nên dùng mô hình:

```text
Realtime task:
    ghi telemetry vào buffer A

TCP task:
    đọc snapshot từ buffer B

Định kỳ swap buffer rất ngắn
```

Ví dụ đơn giản:

```c
typedef struct {
    int rpm;
    int pwm;
    int target_rpm;
    uint32_t deadline_misses;
    int64_t max_exec_us;
    uint32_t fault_flags;
} telemetry_t;

static telemetry_t g_tel;
```

Realtime task chỉ ghi nhanh:

```c
portENTER_CRITICAL(&state_lock);
g_tel.rpm = rpm;
g_tel.pwm = pwm;
g_tel.target_rpm = target;
g_tel.deadline_misses = deadline_misses;
g_tel.max_exec_us = max_exec_us;
g_tel.fault_flags = fault_flags;
portEXIT_CRITICAL(&state_lock);
```

TCP task chỉ snapshot một lần:

```c
telemetry_t tel;

portENTER_CRITICAL(&state_lock);
tel = g_tel;
portEXIT_CRITICAL(&state_lock);
```

Không nên để TCP task vừa giữ lock vừa xử lý logic dài.

---

## 9. Gộp telemetry thành một lệnh duy nhất

GUI hiện tại mỗi lần đọc monitor gửi nhiều lệnh riêng. Cách này ổn cho demo, nhưng không tối ưu cho công nghiệp.

Nên thêm lệnh:

```c
#define CMD_READ_TELEMETRY 0x20
```

Thay vì GUI gửi 7 frame:

```text
READ_ENCODER
READ_PWM
STATUS flags
STATUS target
STATUS misses
STATUS max ISR
STATUS rx frames
```

Nên gộp thành 1 frame trả về nhiều dữ liệu:

```text
RPM
PWM
Target RPM
Mode
Fault flags
Deadline misses
Max exec us
```

Do frame hiện tại chỉ 12 byte, bạn có 2 lựa chọn:

### Cách 1: giữ frame 12 byte, đọc từng nhóm

```c
CMD_READ_TELEMETRY_1: rpm + pwm
CMD_READ_TELEMETRY_2: target + flags
CMD_READ_TELEMETRY_3: deadline + max_exec
```

### Cách 2: đổi sang frame dài hơn

Ví dụ:

```c
#define FRAME_LEN 32
```

Frame response:

```text
@REC
cmd
seq
rpm_H rpm_L
pwm_H pwm_L
target_H target_L
flags_H flags_L
fault_H fault_L
deadline_H deadline_L
max_exec_H max_exec_L
crc16_H crc16_L
#
```

Cách 2 chuyên nghiệp hơn.

---

## 10. Đổi CRC XOR thành CRC-16

Hiện tại CRC chỉ là XOR 10 byte đầu. Trong hệ demo thì được, nhưng công nghiệp nên dùng CRC-16.

Hiện tại:

```c
static uint8_t calc_crc(const uint8_t *frame)
{
    uint8_t crc = 0;
    for (int i = 0; i < 10; i++) {
        crc ^= frame[i];
    }
    return crc;
}
```

Nên đổi sang CRC-16/MODBUS hoặc CRC-16/CCITT.

Ví dụ:

```c
static uint16_t crc16_modbus(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];

        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}
```

---

## 11. Thêm timestamp và sequence cho dữ liệu realtime

Hiện tại frame đã có `seq`, đây là điểm tốt. Cả C và Python đều dùng frame 12 byte, port 5000, sequence, CRC và header `@SEN`/`@REC`.

Nhưng với công nghiệp nên thêm:

```text
sequence
timestamp_us
cycle_counter
fault_flags
deadline_misses
```

Ví dụ telemetry nên có:

```c
uint32_t rt_cycle_count;
uint32_t timestamp_ms;
uint16_t rpm;
uint16_t pwm;
uint16_t target;
uint16_t fault_flags;
uint16_t deadline_misses;
```

Mục đích:

- Biết dữ liệu có mới không.
- Biết có mất chu kỳ không.
- Biết vòng realtime có bị trễ không.
- Biết PC có đang đọc dữ liệu cũ không.

---

## 12. Thêm trạng thái fault công nghiệp

Nên thêm enum trạng thái hệ thống:

```c
typedef enum {
    SYS_INIT = 0,
    SYS_READY,
    SYS_RUNNING_MANUAL,
    SYS_RUNNING_PI,
    SYS_FAULT,
    SYS_ESTOP
} system_state_t;
```

Không nên chỉ có `pi_enabled`.

Luồng công nghiệp hơn:

```text
INIT
 ↓
READY
 ↓
RUNNING_MANUAL hoặc RUNNING_PI
 ↓
FAULT nếu lỗi
 ↓
RESET_FAULT
 ↓
READY
```

Các lỗi nên có:

```c
#define FAULT_COMM_TIMEOUT     0x0001
#define FAULT_ENCODER_TIMEOUT  0x0002
#define FAULT_OVERSPEED        0x0004
#define FAULT_PWM_ERROR        0x0008
#define FAULT_DEADLINE_MISS    0x0010
#define FAULT_ETH_LINK_DOWN    0x0020
#define FAULT_ESTOP            0x0040
```

---

## 13. Chu kỳ PI nên chọn rõ theo yêu cầu động cơ

Hiện tại:

```text
Timer: 5 ms
PI: 50 ms
```

Nếu động cơ cần đáp ứng nhanh hơn, nên dùng:

```c
#define RT_PERIOD_MS 1
#define PI_DIVIDER 10
#define PI_PERIOD_MS 10
```

hoặc:

```c
#define RT_PERIOD_MS 2
#define PI_DIVIDER 5
#define PI_PERIOD_MS 10
```

Nhưng không nên giảm bừa. Cần đo:

```text
max_rt_exec_us < 20% đến 30% chu kỳ realtime
deadline_misses = 0
RPM không nhiễu quá nhiều
PWM không dao động mạnh
```

Ví dụ nếu PI chạy mỗi 10 ms, thì thời gian xử lý nên thấp hơn khoảng 1–2 ms, tốt nhất dưới vài trăm µs.

---

## 14. Không đọc nút BOOT trong ISR

Đọc nút BOOT nên đưa sang task riêng:

```c
static void button_task(void *arg)
{
    while (1) {
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            request_stop_motor();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

Không được delay trong timer ISR.

---

## 15. Sửa GUI Python: chỉ là HMI, không phải realtime controller

GUI hiện tại có:

```python
POLL_PERIOD_MS = 100
GUI_EVENT_PERIOD_MS = 16
SOCKET_TIMEOUT_S = 0.25
```

Tức là GUI đọc monitor 100 ms/lần, xử lý event GUI khoảng 16 ms/lần.

Với công nghiệp, GUI nên giữ vai trò:

```text
- Set target RPM
- Set Kp/Ki
- Enable/disable
- Reset fault
- Read telemetry
- Log dữ liệu
```

Không nên để GUI quyết định PWM theo từng chu kỳ.

Nên thêm các nút:

```text
Reset Fault
Emergency Stop
Read Telemetry Packet
Save Log
```

Và hiển thị thêm:

```text
System state
Fault flags
Cycle counter
Last command age
Deadline miss count
Max control execution time
Ethernet link status
```

---

## 16. Phiên bản sửa đề xuất cho bài của bạn

Nếu muốn nâng cấp vừa đủ mạnh để ghi trong báo cáo là “theo hướng hard realtime”, tôi khuyên sửa theo thứ tự này:

### Mức 1 — bắt buộc sửa

```text
1. Bỏ esp_rom_delay_us(600) khỏi ISR.
2. Không xử lý nút BOOT trong ISR.
3. Timer ISR chỉ notify realtime task.
4. Realtime task ưu tiên cao, pinned core riêng.
5. TCP task không được gọi set_motor_output trực tiếp.
6. Watchdog timeout áp dụng cho cả manual PWM và PI.
7. Thêm fault state.
```

### Mức 2 — nên sửa

```text
8. Gộp telemetry thành một lệnh đọc duy nhất.
9. Thêm CRC-16.
10. Thêm timestamp/cycle counter.
11. Thêm fault_flags.
12. Giảm lock/critical section.
13. Log deadline_misses và max_exec_us rõ ràng.
```

### Mức 3 — công nghiệp hơn

```text
14. Dùng watchdog phần cứng.
15. Có E-stop phần cứng.
16. Có relay/circuit cắt công suất motor khi fault.
17. Dùng protocol realtime công nghiệp thay TCP nếu cần đồng bộ nhiều node.
18. Test worst-case: TCP flood, rút dây mạng, encoder lỗi, motor kẹt, CPU load cao.
```

---

## 17. Cấu trúc cuối cùng nên hướng tới

```text
                  ┌────────────────────┐
                  │      GUI PC         │
                  │  HMI / Monitor      │
                  └─────────┬──────────┘
                            │ TCP Ethernet
                            │ không realtime
                            ▼
┌────────────────────────────────────────────────┐
│                    ESP32                        │
│                                                │
│  Core 0                                        │
│  ┌────────────────────┐                        │
│  │ TCP Server Task     │                        │
│  │ Parse command       │                        │
│  │ Update command buf  │                        │
│  └─────────┬──────────┘                        │
│            │ shared command/telemetry           │
│            ▼                                    │
│  Core 1                                        │
│  ┌────────────────────┐                        │
│  │ GPTimer ISR         │  mỗi 1/2/5 ms          │
│  │ notify only         │                        │
│  └─────────┬──────────┘                        │
│            ▼                                    │
│  ┌────────────────────┐                        │
│  │ RT Control Task     │                        │
│  │ Read encoder        │                        │
│  │ Compute PI          │                        │
│  │ Update PWM          │                        │
│  │ Check fault         │                        │
│  └────────────────────┘                        │
└────────────────────────────────────────────────┘
```

---

## Kết luận

Nếu muốn theo hướng **hard realtime công nghiệp**, không phải sửa GUI là chính, mà phải sửa firmware ESP32 theo hướng:

```text
ISR cực ngắn
Realtime task ưu tiên cao
TCP chỉ cấu hình/giám sát
PWM chỉ được cập nhật bởi realtime task
Có watchdog + fault state + timeout + telemetry chuẩn
Không delay, không log, không blocking trong realtime path
```

Với bài của bạn, chỉ cần sửa được 5 điểm quan trọng nhất này là hệ thống đã thuyết phục hơn rất nhiều:

```text
1. Bỏ delay khỏi ISR.
2. Timer ISR chỉ notify task.
3. Tạo realtime control task riêng.
4. TCP không set PWM trực tiếp.
5. Thêm watchdog/fault cho cả manual và PI.
```

Khi đó có thể mô tả trong báo cáo là:

> Hệ thống được thiết kế theo kiến trúc realtime cục bộ. Vòng điều khiển tốc độ được thực thi độc lập trên ESP32 bằng timer phần cứng và task ưu tiên cao, trong khi Ethernet TCP chỉ đảm nhiệm vai trò giám sát và cấu hình. Nhờ đó, jitter của mạng không ảnh hưởng trực tiếp đến chu kỳ điều khiển động cơ.
