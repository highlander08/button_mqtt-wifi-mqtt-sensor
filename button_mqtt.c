// // #include <stdio.h>
// // #include <string.h>
// // #include <stdlib.h>
// // #include "pico/stdlib.h"
// // #include "pico/cyw43_arch.h"
// // #include "lwip/apps/mqtt.h"
// // #include "lwip/ip_addr.h"
// // #include "lwip/dns.h"
// // #include "max30102.h"

// // // Configurações Wi-Fi
// // #define WIFI_SSID "Navega+ Highlander 2G"
// // #define WIFI_PASSWORD "Blessedhr10@"

// // // Configurações MQTT
// // #define MQTT_BROKER "broker.emqx.io"
// // #define MQTT_BROKER_PORT 1883
// // #define MQTT_TOPIC "embarca/batimentos"

// // // Variáveis Globais
// // static mqtt_client_t *mqtt_client;
// // static ip_addr_t broker_ip;
// // static bool mqtt_connected = false;

// // // Protótipos de Funções
// // static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
// // // void publish_mock_heartbeat();
// // void publish_real_heartbeat(void);
// // void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);

// // // Geração de valor aleatório de batimento cardíaco

// // // int mock_oximeter()
// // // {
// // //     return 95 + (rand() % 5); // 95 a 99 %
// // // }

// // // Função principal
// // int main()
// // {
// //     stdio_init_all();
// //     sleep_ms(2000);
// //     printf("\n=== Iniciando Publicação de Batimentos MQTT ===\n");

// //     // Inicializa Wi-Fi
// //     if (cyw43_arch_init())
// //     {
// //         printf("Erro na inicialização do Wi-Fi\n");
// //         return -1;
// //     }
// //     cyw43_arch_enable_sta_mode();

// //     printf("[Wi-Fi] Conectando...\n");
// //     if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000))
// //     {
// //         printf("[Wi-Fi] Falha na conexão Wi-Fi\n");
// //         return -1;
// //     }
// //     else
// //     {
// //         printf("[Wi-Fi] Conectado com sucesso!\n");
// //     }

// //     // Inicializa o sensor MAX30102
// //     max30102_init();

// //     // Inicializa cliente MQTT
// //     mqtt_client = mqtt_client_new();

// //     // Resolve DNS do broker MQTT
// //     err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
// //     if (err == ERR_OK)
// //     {
// //         dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
// //     }
// //     else if (err == ERR_INPROGRESS)
// //     {
// //         printf("[DNS] Resolvendo...\n");
// //     }
// //     else
// //     {
// //         printf("[DNS] Erro ao resolver DNS: %d\n", err);
// //         return -1;
// //     }

// //     // Loop principal
// //     while (true)
// //     {
// //         cyw43_arch_poll();

// //         if (mqtt_connected)
// //         {
// //             publish_real_heartbeat();
// //             sleep_ms(5000); // Aguarda 5 segundos entre envios
// //         }

// //         sleep_ms(100);
// //     }

// //     cyw43_arch_deinit();
// //     return 0;
// // }

// // // Publicar array de objeto JSON com batimento aleatório
// // // void publish_mock_heartbeat()
// // //     {
// // //     if (!mqtt_connected)
// // //     {
// // //         printf("[MQTT] Não conectado, não publicando\n");
// // //         return;
// // //     }

// // //     int bpm = mock_heartbeat();
// // //     int spo2 = mock_oximeter();

// // //     char payload[128];
// // //     snprintf(payload, sizeof(payload),
// // //              "[{\"batimento\": %d, \"oximetro\": %d}]",
// // //              bpm, spo2);

// // //     printf("[MQTT] Publicando batimento: %s\n", payload);

// // //     err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC, payload, strlen(payload), 0, 0, NULL, NULL);
// // //     if (err == ERR_OK)
// // //     {
// // //         printf("[MQTT] Publicação enviada com sucesso\n");
// // //     }
// // //     else
// // //     {
// // //         printf("[MQTT] Erro ao publicar: %d\n", err);
// // //     }
// // // }
// // // Função para publicar dados reais do sensor
// // void publish_real_heartbeat(void)
// // {
// //     if (!mqtt_connected)
// //     {
// //         printf("[MQTT] Não conectado, não publicando\n");
// //         return;
// //     }

// //     // Ler e calcular dados do sensor
// //     max30102_data_t data = {0};
// //     max30102_read_fifo(&data);
// //     max30102_calculate_bpm_spo2(&data);

// //     // Converter BPM e SpO2 para inteiros e validar
// //     int bpm = (int)data.bpm;
// //     int spo2 = (int)data.spo2;
// //     if (bpm > 40 && bpm < 200 && spo2 > 80 && spo2 <= 100)
// //     {
// //         char payload[128];
// //         snprintf(payload, sizeof(payload),
// //                  "[{\"batimento\": %d, \"oximetro\": %d}]",
// //                  bpm, spo2);

// //         printf("[MQTT] Publicando: %s\n", payload);

// //         err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC, payload, strlen(payload), 0, 0, NULL, NULL);
// //         if (err == ERR_OK)
// //         {
// //             printf("[MQTT] Publicação enviada com sucesso\n");
// //         }
// //         else
// //         {
// //             printf("[MQTT] Erro ao publicar: %d\n", err);
// //         }
// //     }
// //     else
// //     {
// //         printf("[Sensor] Dados inválidos: BPM=%.0f, SpO2=%.0f\n", data.bpm, data.spo2);
// //     }
// // }

// // // Callback de conexão MQTT
// // static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
// // {
// //     if (status == MQTT_CONNECT_ACCEPTED)
// //     {
// //         printf("[MQTT] Conectado ao broker!\n");
// //         mqtt_connected = true;
// //     }
// //     else
// //     {
// //         printf("[MQTT] Falha na conexão MQTT. Código: %d\n", status);
// //         mqtt_connected = false;
// //     }
// // }

// // // Callback de DNS
// // void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
// // {
// //     if (ipaddr != NULL)
// //     {
// //         broker_ip = *ipaddr;
// //         printf("[DNS] Resolvido: %s -> %s\n", name, ipaddr_ntoa(ipaddr));

// //         struct mqtt_connect_client_info_t ci = {
// //             .client_id = "pico-heartbeat-sender",
// //             .keep_alive = 60};

// //         printf("[MQTT] Conectando ao broker...\n");
// //         mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_callback, NULL, &ci);
// //     }
// //     else
// //     {
// //         printf("[DNS] Falha ao resolver DNS para %s\n", name);
// //     }
// // }

// // versao final 1
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "lwip/apps/mqtt.h"
// #include "lwip/ip_addr.h"
// #include "lwip/dns.h"

// // Configurações Wi-Fi
// #define WIFI_SSID "Navega+ Highlander 2G"
// #define WIFI_PASSWORD "Blessedhr10@"

// // Configurações MQTT
// #define MQTT_BROKER "broker.emqx.io"
// #define MQTT_BROKER_PORT 1883
// #define MQTT_TOPIC "embarca/batimentos"

// // Variáveis Globais
// static mqtt_client_t *mqtt_client;
// static ip_addr_t broker_ip;
// static bool mqtt_connected = false;

// // Protótipos de Funções
// static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
// void publish_mock_heartbeat(void);
// void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);

// // Função principal
// int main()
// {
//     stdio_init_all();
//     sleep_ms(2000);
//     printf("\n=== Iniciando Publicação de Batimentos MQTT (Fictícios) ===\n");

//     // Inicializa gerador de números aleatórios
//     srand((unsigned)time(NULL));

//     // Inicializa Wi-Fi
//     if (cyw43_arch_init())
//     {
//         printf("Erro na inicialização do Wi-Fi\n");
//         return -1;
//     }
//     cyw43_arch_enable_sta_mode();

//     printf("[Wi-Fi] Conectando...\n");
//     if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000))
//     {
//         printf("[Wi-Fi] Falha na conexão Wi-Fi\n");
//         return -1;
//     }
//     else
//     {
//         printf("[Wi-Fi] Conectado com sucesso!\n");
//     }

//     // Inicializa cliente MQTT
//     mqtt_client = mqtt_client_new();

//     // Resolve DNS do broker MQTT
//     err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
//     if (err == ERR_OK)
//     {
//         dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
//     }
//     else if (err == ERR_INPROGRESS)
//     {
//         printf("[DNS] Resolvendo...\n");
//     }
//     else
//     {
//         printf("[DNS] Erro ao resolver DNS: %d\n", err);
//         return -1;
//     }

//     // Loop principal
//     while (true)
//     {
//         cyw43_arch_poll();

//         if (mqtt_connected)
//         {
//             publish_mock_heartbeat(); // Publica batimentos e SpO2 fictícios
//             sleep_ms(5000);           // Aguarda 5 segundos entre envios
//         }

//         sleep_ms(100);
//     }

//     cyw43_arch_deinit();
//     return 0;
// }

// // Função para publicar dados fictícios de batimento e SpO2
// void publish_mock_heartbeat(void)
// {
//     if (!mqtt_connected)
//     {
//         printf("[MQTT] Não conectado, não publicando\n");
//         return;
//     }

//     int bpm = 60 + (rand() % 41);   // 60 a 100 BPM
//     int spo2 = 95 + (rand() % 6);   // 95 a 100 %

//     char payload[128];
//     snprintf(payload, sizeof(payload),
//              "[{\"batimento\": %d, \"oximetro\": %d}]",
//              bpm, spo2);

//     printf("[MQTT] Publicando (fictício): %s\n", payload);

//     err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC, payload, strlen(payload), 0, 0, NULL, NULL);
//     if (err == ERR_OK)
//     {
//         printf("[MQTT] Publicação enviada com sucesso\n");
//     }
//     else
//     {
//         printf("[MQTT] Erro ao publicar: %d\n", err);
//     }
// }

// // Callback de conexão MQTT
// static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
// {
//     if (status == MQTT_CONNECT_ACCEPTED)
//     {
//         printf("[MQTT] Conectado ao broker!\n");
//         mqtt_connected = true;
//     }
//     else
//     {
//         printf("[MQTT] Falha na conexão MQTT. Código: %d\n", status);
//         mqtt_connected = false;
//     }
// }

// // Callback de DNS
// void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
// {
//     if (ipaddr != NULL)
//     {
//         broker_ip = *ipaddr;
//         printf("[DNS] Resolvido: %s -> %s\n", name, ipaddr_ntoa(ipaddr));

//         struct mqtt_connect_client_info_t ci = {
//             .client_id = "pico-heartbeat-sender",
//             .keep_alive = 60};

//         printf("[MQTT] Conectando ao broker...\n");
//         mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_callback, NULL, &ci);
//     }
//     else
//     {
//         printf("[DNS] Falha ao resolver DNS para %s\n", name);
//     }
// }

// // versao final 2 - com max30102
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

// --- Configuração MAX30102 ---
#define I2C_PORT i2c1
#define SDA_PIN 2
#define SCL_PIN 3
#define MAX_ADDR 0x57
#define FS_HZ 100.0f
#define LED_PA 0x1F

// --- Limites e parâmetros ---
#define FINGER_DC_MIN 8000.0f
#define FINGER_DC_MAX 240000.0f
#define FINGER_SNR_MIN 0.0030f
#define REFRACTORY_S 0.55f
#define RR_MIN_S 0.45f
#define RR_MAX_S 1.60f
#define RR_LOWER_FRAC 0.80f
#define RR_UPPER_FRAC 1.60f

// --- MQTT ---
static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip;
static bool mqtt_connected = false;

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
    s->dc_red = s->dc_ir = 0.0f;
    s->var_red = s->var_ir = 1.0f;
    s->ma_sum = 0.0f;
    for (int i = 0; i < 5; i++)
        s->ma_buf[i] = 0.0f;
    s->ma_idx = 0;
    s->last_peak = -100000;
    s->rr_len = 0;
    s->bpm = 0.0f;
    s->finger_on = false;
    s->last_rms_ir = 0.0f;
    s->last_snr = 0.0f;
    s->last_dc_ir = 0.0f;
    s->acs_prev1 = s->acs_prev2 = 0.0f;
}

// --- Funções I2C para MAX30102 ---
static bool wr8(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_write_blocking(I2C_PORT, MAX_ADDR, b, 2, false) == 2;
}
static bool rd8(uint8_t reg, uint8_t *val)
{
    if (i2c_write_blocking(I2C_PORT, MAX_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_PORT, MAX_ADDR, val, 1, false) == 1;
}
static bool rdbuf(uint8_t reg, uint8_t *buf, size_t n)
{
    if (i2c_write_blocking(I2C_PORT, MAX_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_PORT, MAX_ADDR, buf, n, false) == (int)n;
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
    wr8(REG_MODE_CFG, 0x40);
    sleep_ms(10);
    wr8(REG_MODE_CFG, 0x00);
    sleep_ms(10);
}
static void max_init(uint8_t led_pa)
{
    max_reset();
    wr8(0x02, 0x00);
    wr8(0x03, 0x00);
    wr8(REG_FIFO_WR_PTR, 0);
    wr8(REG_OVF_CNT, 0);
    wr8(REG_FIFO_RD_PTR, 0);
    wr8(REG_FIFO_CFG, (2 << 5) | (1 << 4));
    wr8(REG_SPO2_CFG, SPO2_CFG_RANGE_SR_PW);
    wr8(REG_LED1_PA, led_pa);
    wr8(REG_LED2_PA, led_pa);
    wr8(REG_MODE_CFG, 0x03);
    sleep_ms(100);
}
static inline uint8_t fifo_available(void)
{
    uint8_t wr, rd;
    rd8(REG_FIFO_WR_PTR, &wr);
    rd8(REG_FIFO_RD_PTR, &rd);
    return (uint8_t)((wr - rd) & 0x1F);
}
static inline bool read_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t b[6];
    if (!rdbuf(REG_FIFO_DATA, b, 6))
        return false;
    *red = (((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) & 0x3FFFF;
    *ir = (((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5]) & 0x3FFFF;
    return true;
}

// --- Processamento ---
static float process(PulseState *s, uint32_t red_raw, uint32_t ir_raw, int idx)
{
    const float a_dc = 0.03f, a_var = 0.03f;
    float red = (float)red_raw, ir = (float)ir_raw;
    s->dc_red += a_dc * (red - s->dc_red);
    s->dc_ir += a_dc * (ir - s->dc_ir);
    float ac_red = red - s->dc_red, ac_ir = ir - s->dc_ir;
    s->var_red += a_var * ((ac_red * ac_red) - s->var_red);
    s->var_ir += a_var * ((ac_ir * ac_ir) - s->var_ir);
    float dc_ir = fmaxf(s->dc_ir, 1.0f), dc_redv = fmaxf(s->dc_red, 1.0f);
    float rms_ir = sqrtf(fmaxf(s->var_ir, 1.0f)), rms_red = sqrtf(fmaxf(s->var_red, 1.0f));
    float snr = rms_ir / dc_ir;
    s->last_dc_ir = dc_ir;
    s->last_rms_ir = rms_ir;
    s->last_snr = snr;
    s->finger_on = (dc_ir > FINGER_DC_MIN && dc_ir < FINGER_DC_MAX && dc_redv > FINGER_DC_MIN && dc_redv < FINGER_DC_MAX && snr >= FINGER_SNR_MIN);

    s->ma_sum -= s->ma_buf[s->ma_idx];
    s->ma_buf[s->ma_idx] = ac_ir;
    s->ma_sum += s->ma_buf[s->ma_idx];
    s->ma_idx = (s->ma_idx + 1) % 5;
    float ac_s = s->ma_sum / 5.0f;

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
                float mean = rr;
                if (s->rr_len)
                {
                    float soma = 0.0f;
                    for (int i = 0; i < s->rr_len; i++)
                        soma += s->rr_hist[i];
                    mean = soma / s->rr_len;
                }
                if (rr > RR_LOWER_FRAC * mean && rr < RR_UPPER_FRAC * mean)
                {
                    if (s->rr_len < 5)
                        s->rr_hist[s->rr_len++] = rr;
                    else
                    {
                        for (int i = 1; i < 5; i++)
                            s->rr_hist[i - 1] = s->rr_hist[i];
                        s->rr_hist[4] = rr;
                    }
                    float m = 0.0f;
                    for (int i = 0; i < s->rr_len; i++)
                        m += s->rr_hist[i];
                    m /= s->rr_len;
                    s->bpm = 60.0f / m;
                }
            }
        }
    }
    else
    {
        s->bpm = 0.0f;
        s->rr_len = 0;
    }

    float spo2 = NAN;
    if (s->finger_on)
    {
        float R = (rms_red / dc_redv) / (rms_ir / dc_ir);
        spo2 = 110.0f - 25.0f * R;
        if (spo2 < 70.0f)
            spo2 = 70.0f;
        if (spo2 > 100.0f)
            spo2 = 100.0f;
    }
    s->acs_prev2 = s->acs_prev1;
    s->acs_prev1 = ac_s;
    return spo2;
}

// --- Publica no MQTT ---
void publish_heartbeat(float bpm, float spo2)
{
    if (!mqtt_connected)
        return;
    char payload[128];
    snprintf(payload, sizeof(payload), "[{\"batimento\": %.0f, \"oximetro\": %.0f}]", bpm, spo2);
    printf("[MQTT] Publicando: %s\n", payload);
    mqtt_publish(mqtt_client, MQTT_TOPIC, payload, strlen(payload), 0, 0, NULL, NULL);
}

// --- Função principal ---
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("[INFO] Inicializando Wi-Fi...\n");
    if (cyw43_arch_init())
    {
        printf("[ERROR] Falha ao inicializar Wi-Fi\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    printf("[INFO] Conectando à rede %s...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        printf("[ERROR] Falha na conexão Wi-Fi\n");
        return -1;
    }
    printf("[INFO] Wi-Fi conectado!\n");

    printf("[INFO] Inicializando cliente MQTT...\n");
    mqtt_client = mqtt_client_new();
    err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
    if (err == ERR_OK)
        dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
    else if (err != ERR_INPROGRESS)
    {
        printf("[ERROR] Falha ao resolver DNS do broker MQTT\n");
        return -1;
    }

    printf("[INFO] Inicializando I2C para MAX30102...\n");
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    sleep_ms(300);

    // --- Verificação do sensor MAX30102 ---
    uint8_t reg = 0xFF; // REG_PART_ID
    uint8_t part = 0;
    int res = i2c_write_blocking(I2C_PORT, MAX_ADDR, &reg, 1, true);
    if (res == 1)
    {
        res = i2c_read_blocking(I2C_PORT, MAX_ADDR, &part, 1, false);
    }

    sleep_ms(2000); // Dá tempo para USB e sensor estabilizarem
    printf("[INFO] Inicializando sensor MAX30102...\n");
    max_init(LED_PA);
    PulseState st;
    ps_init(&st);
    printf("[INFO] Entrando no loop principal...\n");

    int idx = 0, ticker = (int)FS_HZ;
    while (true)
    {
        cyw43_arch_poll();
        uint8_t avail = fifo_available();
        if (avail == 0)
        {
            sleep_ms(5);
            continue;
        }
        for (uint8_t i = 0; i < avail; i++)
        {
            uint32_t red, ir;
            if (!read_sample(&red, &ir))
                break;
            float spo2 = process(&st, red, ir, idx++);
            if (--ticker <= 0)
            {
                bool pulse_recent = (idx - st.last_peak) < (int)(FS_HZ * 2.0f);
                bool valid = st.finger_on && pulse_recent && (st.bpm >= 35.0f && st.bpm <= 200.0f);
                if (valid && !isnan(spo2))
                    publish_heartbeat(st.bpm, spo2);
                ticker = (int)FS_HZ;
            }
        }
    }
    cyw43_arch_deinit();
    return 0;
}

