#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// --- Configurações Wi-Fi ---
#define WIFI_SSID "Navega+ Highlander 2G"
#define WIFI_PASSWORD "Blessedhr10@"

// --- Configurações MQTT ---
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC "embarca/batimentos"
#define MQTT_FALL_TOPIC "embarca/quedas"

// --- Configuração MAX30102 ---
#define I2C_MAX_PORT i2c1
#define SDA_MAX_PIN 2
#define SCL_MAX_PIN 3
#define MAX30102_ADDR 0x57
#define FS_HZ 100.0f
#define LED_PA 0x1F

// --- Configuração MPU6050 ---
#define I2C_MPU_PORT i2c0
#define SCL_MPU_PIN 0
#define SDA_MPU_PIN 1
#define MPU6050_ADDR 0x68
#define MPU_BAUD_RATE 400000

// --- Parâmetros de Detecção de Queda ---
#define FREE_FALL_THRESHOLD   0.3f    // Aceleração < 0.3g = queda livre
#define IMPACT_THRESHOLD      2.5f    // Aceleração > 2.5g = impacto
#define FALL_TIMEOUT_MS       3000    // Tempo máximo para detectar queda completa

// --- Limites e parâmetros do Oxímetro ---
#define FINGER_DC_MIN 8000.0f
#define FINGER_DC_MAX 240000.0f
#define FINGER_SNR_MIN 0.0030f
#define REFRACTORY_S 0.55f
#define RR_MIN_S 0.45f
#define RR_MAX_S 1.60f
#define RR_LOWER_FRAC 0.80f
#define RR_UPPER_FRAC 1.60f

// --- Estruturas de Dados ---
typedef struct {
    float x, y, z;
} VECT_3D;



// --- MQTT ---
static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip;
static bool mqtt_connected = false;
static absolute_time_t last_publish_time = 0;
static const uint32_t PUBLISH_INTERVAL_MS = 2000; // Publica a cada 2 segundos

void publish_message(const char *topic, const char *payload) {
    if (!mqtt_connected) {
        printf("[MQTT] Cliente não conectado, ignorando publicação para o tópico %s\n", topic);
        return;
    }
    printf("[MQTT] Publicando em '%s': %s\n", topic, payload);
    mqtt_publish(mqtt_client, topic, payload, strlen(payload), 1, 0, NULL, NULL);
}

// --- Funções MQTT ---
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    printf("[MQTT] %s\n", mqtt_connected ? "Conectado ao broker!, Teste Sensor Agora" : "Falha na conexão MQTT");
}

void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    if (!ipaddr)
    {
        printf("[DNS] Falha ao resolver %s\n", name);
        return;
    }
    broker_ip = *ipaddr;
    struct mqtt_connect_client_info_t ci = {.client_id = "pico-heartbeat-sender", .keep_alive = 60};
    mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_callback, NULL, &ci);
}

// --- Estrutura PulseState ---
typedef struct
{
    float dc_red, dc_ir, var_red, var_ir;
    float ma_sum, ma_buf[5];
    int ma_idx;
    int last_peak;
    float rr_hist[5];
    int rr_len;
    float bpm;
    bool finger_on;
    float last_rms_ir, last_snr, last_dc_ir;
    float acs_prev1, acs_prev2;
} PulseState;

static void ps_init(PulseState *s)
{
    memset(s, 0, sizeof(PulseState));
    s->dc_red = s->dc_ir = 10000.0f; // Valores iniciais mais realistas
    s->var_red = s->var_ir = 1000.0f;
}

// --- Estrutura e Funções MPU6050 ---
typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
} MPU6050_t;

void mpu6050_init(MPU6050_t *mpu, i2c_inst_t *i2c, uint8_t addr) {
    mpu->i2c = i2c;
    mpu->addr = addr;
    uint8_t buffer[2] = {0x6B, 0x00}; // PWR_MGMT_1, desliga modo sleep
    i2c_write_blocking(mpu->i2c, mpu->addr, buffer, 2, false);
    sleep_ms(100);
}

void mpu6050_get_accel(MPU6050_t *mpu, VECT_3D *vect) {
    uint8_t buffer[6];
    uint8_t reg = 0x3B; // ACCEL_XOUT_H
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, buffer, 6, false);
    
    int16_t ax = (buffer[0] << 8) | buffer[1];
    int16_t ay = (buffer[2] << 8) | buffer[3];
    int16_t az = (buffer[4] << 8) | buffer[5]; // Corrigido: buffer[5]
    
    vect->x = ax / 16384.0f;
    vect->y = ay / 16384.0f;
    vect->z = az / 16384.0f;
}

// --- Estrutura e Funções do Detector de Queda ---
typedef struct {
    float prev_accel_total;
    uint32_t fall_start_time;
    bool free_fall_detected;
    bool impact_detected;
} FallDetector_t;

void fall_detector_init(FallDetector_t *fd) {
    memset(fd, 0, sizeof(FallDetector_t));
    fd->prev_accel_total = 1.0f; // Assume 1g em repouso
}

// --- Funções I2C para MAX30102 ---
static bool max30102_wr8(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, b, 2, false) == 2;
}

static bool max30102_rd8(uint8_t reg, uint8_t *val)
{
    if (i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_MAX_PORT, MAX30102_ADDR, val, 1, false) == 1;
}

static bool max30102_rdbuf(uint8_t reg, uint8_t *buf, size_t n)
{
    if (i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_MAX_PORT, MAX30102_ADDR, buf, n, false) == (int)n;
}

#define REG_MODE_CFG 0x09
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_CNT 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CFG 0x08
#define REG_SPO2_CFG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PART_ID 0xFF
#define SPO2_CFG_RANGE_SR_PW 0x67

static void max_reset(void)
{
    max30102_wr8(REG_MODE_CFG, 0x40);
    sleep_ms(10);
    max30102_wr8(REG_MODE_CFG, 0x00);
    sleep_ms(10);
}

static bool max_init(uint8_t led_pa)
{
    max_reset();

    // Configuração mais robusta do MAX30102
    if (!max30102_wr8(0x02, 0x00)) return false; // FIFO reset
    if (!max30102_wr8(0x03, 0x00)) return false; // Mode reset
    if (!max30102_wr8(REG_FIFO_WR_PTR, 0)) return false;
    if (!max30102_wr8(REG_OVF_CNT, 0)) return false;
    if (!max30102_wr8(REG_FIFO_RD_PTR, 0)) return false;
    if (!max30102_wr8(REG_FIFO_CFG, (2 << 5) | (1 << 4))) return false; // SMP_AVE=2, FIFO_ROLLOVER_EN
    if (!max30102_wr8(REG_SPO2_CFG, SPO2_CFG_RANGE_SR_PW)) return false;
    if (!max30102_wr8(REG_LED1_PA, led_pa)) return false;
    if (!max30102_wr8(REG_LED2_PA, led_pa)) return false;
    if (!max30102_wr8(REG_MODE_CFG, 0x03)) return false; // SpO2 mode

    sleep_ms(100);
    return true;
}

static inline uint8_t fifo_available(void)
{
    uint8_t wr, rd;
    if (!max30102_rd8(REG_FIFO_WR_PTR, &wr)) return 0;
    if (!max30102_rd8(REG_FIFO_RD_PTR, &rd)) return 0;
    return (uint8_t)((wr - rd) & 0x1F);
}

static inline bool read_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t b[6];
    if (!max30102_rdbuf(REG_FIFO_DATA, b, 6))
        return false;
    *red = (((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) & 0x3FFFF;
    *ir = (((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5]) & 0x3FFFF;
    return true;
}

// --- Processamento ---
static float process_pulse(PulseState *s, uint32_t red_raw, uint32_t ir_raw, int idx)
{
    const float a_dc = 0.03f, a_var = 0.03f;
    float red = (float)red_raw, ir = (float)ir_raw;

    // Atualiza DC components
    s->dc_red += a_dc * (red - s->dc_red);
    s->dc_ir += a_dc * (ir - s->dc_ir);

    // Calcula AC components
    float ac_red = red - s->dc_red, ac_ir = ir - s->dc_ir;

    // Atualiza variância
    s->var_red += a_var * ((ac_red * ac_red) - s->var_red);
    s->var_ir += a_var * ((ac_ir * ac_ir) - s->var_ir);

    float dc_ir = fmaxf(s->dc_ir, 1.0f), dc_redv = fmaxf(s->dc_red, 1.0f);
    float rms_ir = sqrtf(fmaxf(s->var_ir, 1.0f)), rms_red = sqrtf(fmaxf(s->var_red, 1.0f));
    float snr = rms_ir / dc_ir;

    s->last_dc_ir = dc_ir;
    s->last_rms_ir = rms_ir;
    s->last_snr = snr;

    // Detecção de dedo mais tolerante
    s->finger_on = (dc_ir > FINGER_DC_MIN && dc_ir < FINGER_DC_MAX && 
                   dc_redv > FINGER_DC_MIN && dc_redv < FINGER_DC_MAX && 
                   snr >= FINGER_SNR_MIN);

    // Moving average para suavização
    s->ma_sum -= s->ma_buf[s->ma_idx];
    s->ma_buf[s->ma_idx] = ac_ir;
    s->ma_sum += s->ma_buf[s->ma_idx];
    s->ma_idx = (s->ma_idx + 1) % 5;
    float ac_s = s->ma_sum / 5.0f;

    // Detecção de batimentos
    if (s->finger_on)
    {
        float thr_vale = 0.45f * rms_ir;
        int refractory = (int)(REFRACTORY_S * FS_HZ);
        bool is_min_local = (s->acs_prev1 < s->acs_prev2) && (s->acs_prev1 <= ac_s);
        bool amp_ok = (-s->acs_prev1) > thr_vale;

        if (is_min_local && amp_ok && (idx - s->last_peak) > refractory)
        {
            float rr = (float)(idx - s->last_peak) / FS_HZ;
            s->last_peak = idx;

            if (rr > RR_MIN_S && rr < RR_MAX_S)
            {
                // Atualiza histórico de RR
                if (s->rr_len < 5)
                {
                    s->rr_hist[s->rr_len++] = rr;
                }
                else
                {
                    // Shift do array
                    for (int i = 1; i < 5; i++)
                        s->rr_hist[i - 1] = s->rr_hist[i];
                    s->rr_hist[4] = rr;
                }

                // Calcula BPM
                float m = 0.0f;
                for (int i = 0; i < s->rr_len; i++)
                    m += s->rr_hist[i];
                m /= s->rr_len;
                s->bpm = 60.0f / m;
            }
        }
    }
    else
    {
        s->bpm = 0.0f;
        s->rr_len = 0;
    }

    // Cálculo SpO2
    float spo2 = NAN;
    if (s->finger_on && rms_red > 0 && rms_ir > 0)
    {
        float R = (rms_red / dc_redv) / (rms_ir / dc_ir);
        spo2 = 110.0f - 25.0f * R;
        if (spo2 < 70.0f) spo2 = 70.0f;
        if (spo2 > 100.0f) spo2 = 100.0f;
    }

    s->acs_prev2 = s->acs_prev1;
    s->acs_prev1 = ac_s;

    return spo2;
}

static float calculate_total_accel(VECT_3D *accel) {
    return sqrt(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
}

void check_fall(FallDetector_t *fd, VECT_3D *accel) {
    float accel_total = calculate_total_accel(accel);
    char payload[128];

    // FASE 1: Detecção de Queda Livre
    if (accel_total < FREE_FALL_THRESHOLD && !fd->free_fall_detected) {
        printf("🚨 FASE 1: Queda livre detectada! Aceleração: %.2fg\n", accel_total);
        snprintf(payload, sizeof(payload), "{\"status\": \"queda_livre\", \"aceleracao\": %.2f}", accel_total);
        publish_message(MQTT_FALL_TOPIC, payload);
        fd->free_fall_detected = true;
        fd->fall_start_time = to_ms_since_boot(get_absolute_time());
    }

    // FASE 2: Detecção de Impacto
    if (fd->free_fall_detected && accel_total > IMPACT_THRESHOLD) {
        printf("💥 FASE 2: IMPACTO detectado! Aceleração: %.2fg\n", accel_total);
        snprintf(payload, sizeof(payload), "{\"status\": \"impacto\", \"aceleracao\": %.2f}", accel_total);
        publish_message(MQTT_FALL_TOPIC, payload);
        fd->impact_detected = true;
    }

    // FASE 3: Confirmação da Queda
    if (fd->impact_detected) {
        uint32_t time_since_fall_start = to_ms_since_boot(get_absolute_time()) - fd->fall_start_time;

        if (time_since_fall_start > 500 && time_since_fall_start < FALL_TIMEOUT_MS) {
            // Se a pessoa permanece imóvel (aceleração próxima a 1g)
            if (accel_total > 0.8f && accel_total < 1.2f) {
                printf("🆘 🆘 🆘 QUEDA CONFIRMADA! 🆘 🆘 🆘\n");
                publish_message(MQTT_FALL_TOPIC, "{\"status\": \"queda_confirmada\"}");
                fall_detector_init(fd); // Reset detector
            }
        } else if (time_since_fall_start >= FALL_TIMEOUT_MS) {
            printf("⚠️ Timeout - Resetando detecção de queda\n");
            snprintf(payload, sizeof(payload), "{\"status\": \"timeout_reset\"}");
            publish_message(MQTT_FALL_TOPIC, payload);
            fall_detector_init(fd); // Reset por timeout
        }
    }
}

// --- Publica no MQTT ---
void publish_heartbeat(float bpm, float spo2)
{
    if (!mqtt_connected)
    {
        printf("[MQTT] Cliente não conectado, ignorando publicação\n");
        return;
    }

    char payload[128];
    snprintf(payload, sizeof(payload), "[{\"batimento\": %.0f, \"oximetro\": %.0f}]", bpm, spo2);
    publish_message(MQTT_TOPIC, payload);
}

// --- Função principal ---
int main(void)
{
    stdio_init_all();
    printf("\n=== Inicializando Sistema de Monitoramento Cardíaco ===\n");
    sleep_ms(2000);

    // 1. Inicializar Wi-Fi e MQTT
    printf("[1/3] Inicializando Wi-Fi...\n");
    if (cyw43_arch_init())
    {
        printf("[ERROR] Falha ao inicializar Wi-Fi\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    printf("Conectando à rede %s...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000))
    {
        printf("[ERROR] Falha na conexão Wi-Fi\n");
        return -1;
    }
    printf("[SUCCESS] Wi-Fi conectado! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

    printf("[2/3] Inicializando cliente MQTT...\n");
    mqtt_client = mqtt_client_new();
    if (!mqtt_client)
    {
        printf("[ERROR] Falha ao criar cliente MQTT\n");
        return -1;
    }

    // Resolver DNS do broker
    err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
    if (err == ERR_OK)
    {
        dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
    }
    else if (err != ERR_INPROGRESS)
    {
        printf("[ERROR] Falha ao iniciar resolução DNS: %d\n", err);
        return -1;
    }

    // 3. Inicializar Sensores
    printf("[3/3] Inicializando sensores I2C...\n");
    // I2C para MPU6050
    i2c_init(I2C_MPU_PORT, MPU_BAUD_RATE);
    gpio_set_function(SCL_MPU_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_MPU_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_MPU_PIN);
    gpio_pull_up(SDA_MPU_PIN);

    // I2C para MAX30102
    i2c_init(I2C_MAX_PORT, 400 * 1000);
    gpio_set_function(SDA_MAX_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_MAX_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_MAX_PIN);
    gpio_pull_up(SCL_MAX_PIN);
    sleep_ms(500);

    // Inicializar MPU6050
    MPU6050_t mpu;
    mpu6050_init(&mpu, I2C_MPU_PORT, MPU6050_ADDR);
    printf("[INFO] MPU6050 inicializado.\n");

    // Inicializar MAX30102
    uint8_t max_part_id;
    if (!max30102_rd8(0xFF, &max_part_id)) {
        printf("[ERROR] Sensor MAX30102 não encontrado na porta i2c1!\n");
        return -1;
    }
    printf("[INFO] MAX30102 encontrado, PART_ID: 0x%02X\n", max_part_id);

    if (!max_init(LED_PA)) {
        printf("[ERROR] Falha na inicialização do MAX30102\n");
        return -1;
    }

    PulseState st;
    ps_init(&st);
    FallDetector_t fd;
    fall_detector_init(&fd);

    printf("[SUCCESS] Sistema inicializado! Aguardando dados do sensor...\n");

    // 4. Loop Principal
    int idx = 0;
    uint32_t last_mpu_read_time = 0;

    while (true)
    {
        cyw43_arch_poll();

        // Leitura do MAX30102 (alta frequência)
        uint8_t avail = fifo_available();
        if (avail > 0)
        {
            for (uint8_t i = 0; i < avail; i++)
            {
                uint32_t red, ir;
                if (!read_sample(&red, &ir)) {
                    printf("[ERROR] Falha ao ler amostra do sensor\n");
                    break;
                }

                float spo2 = process_pulse(&st, red, ir, idx++);

                // Publicar dados se válidos
                bool pulse_recent = (idx - st.last_peak) < (int)(FS_HZ * 3.0f);
                bool valid = st.finger_on && pulse_recent && 
                           (st.bpm >= 35.0f && st.bpm <= 200.0f) && 
                           !isnan(spo2);
                
                // Verificar intervalo de publicação
                absolute_time_t now = get_absolute_time();
                uint32_t time_since_last_publish = absolute_time_diff_us(last_publish_time, now) / 1000;

                if (valid && time_since_last_publish >= PUBLISH_INTERVAL_MS) {
                    publish_heartbeat(st.bpm, spo2);
                    last_publish_time = now;
                    printf("[DATA] BPM: %.0f, SpO2: %.0f%%\n", st.bpm, spo2);
                }
            }
        }

        // Leitura do MPU6050 (baixa frequência)
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_mpu_read_time > 50) { // Leitura a cada 50ms (20Hz)
            VECT_3D accel;
            mpu6050_get_accel(&mpu, &accel);
            check_fall(&fd, &accel);
            last_mpu_read_time = now_ms;
        }

        // Pequena pausa para não sobrecarregar a CPU
        sleep_ms(5);
    }

    cyw43_arch_deinit();
    return 0;
}
