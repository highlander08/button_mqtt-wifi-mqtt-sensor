````markdown
# Raspberry Pi Pico W - Monitor de Batimentos com MAX30102 e MQTT

Este projeto utiliza o **Raspberry Pi Pico W** com o sensor **MAX30102** para medir batimentos cardíacos (BPM) e oxigenação do sangue (SpO2) e publica os dados em um broker MQTT.

---

## 📦 Materiais necessários

- Raspberry Pi Pico W
- Sensor MAX30102
- Cabos jumpers
- Computador com **Thonny** ou **VSCode** para programação

---

## 🔌 Conexões

| Pico W           | MAX30102 |
|-----------------|----------|
| GPIO2 (SDA)     | SDA      |
| GPIO3 (SCL)     | SCL      |
| GND             | GND      |
| 3.3V            | VCC      |

> ⚠️ **Não conecte no 5V**, use sempre 3.3V para o MAX30102.

---

## 🌐 Configuração Wi-Fi

No código, configure sua rede Wi-Fi:

```c
#define WIFI_SSID "SeuSSID"
#define WIFI_PASSWORD "SuaSenha"
````

---

## 🛰️ Configuração MQTT

Defina o broker e tópico que receberá os dados:

```c
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC "embarca/batimentos"
```

> Você pode usar brokers públicos como `emqx.io` ou criar o seu próprio broker.

---

## ⚙️ Código Fonte

O código principal está em **C/C++** para Raspberry Pi Pico W com SDK Pico e suporte I2C e MQTT.

Principais funções:

* Inicialização do Wi-Fi e MQTT
* Leitura do MAX30102 via I2C
* Processamento do sinal (BPM e SpO2)
* Publicação dos dados via MQTT

```c
// Exemplo de publicação
void publish_heartbeat(float bpm, float spo2)
{
    if (!mqtt_connected) return;
    char payload[128];
    snprintf(payload, sizeof(payload), "[{\"batimento\": %.0f, \"oximetro\": %.0f}]", bpm, spo2);
    printf("[MQTT] Publicando: %s\n", payload);
    mqtt_publish(mqtt_client, MQTT_TOPIC, payload, strlen(payload), 0, 0, NULL, NULL);
}
```

> O código completo inclui tratamento de sinais, filtro de ruído, média móvel, e cálculo de SpO2.

---

## 📝 Como usar

1. Conecte o MAX30102 ao Pico W conforme o esquema.
2. Configure Wi-Fi e broker MQTT no código.
3. Compile e envie o código para o Pico W.
4. Abra o monitor serial para ver os batimentos e SpO2.
5. Verifique seu broker MQTT para receber os dados.

---

## 💡 Observações

* O sensor só funciona com o dedo posicionado corretamente.
* Valores de BPM = 0 indicam dedo não detectado ou leitura inválida.
* Mantenha o Pico W próximo do seu computador durante testes para estabilizar a conexão Wi-Fi.
* Para produção, utilize broker MQTT seguro com autenticação.

---

## 📈 Exemplo de Saída Serial

```
RED=12304 IR=12881 BPM= 64 SpO2≈ 82% [DC_IR=9792 RMS_IR=6376 SNR=65.12% Finger=1]
[MQTT] Publicando: [{"batimento": 64, "oximetro": 82}]
```

---

## 🔧 Dependências

* Raspberry Pi Pico SDK
* Biblioteca `pico/cyw43_arch.h` para Wi-Fi
* Biblioteca `lwIP` para MQTT
* Biblioteca I2C do SDK Pico

---

## 🛠️ Referências

* [MAX30102 Datasheet](https://datasheet.lcsc.com/lcsc/1804081100_Maxim-MAX30102_C26213.pdf)
* [Pico W C SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
* [MQTT Broker EMQX](https://www.emqx.io/)

---

Feito com ❤️ por Highlander

```

---
```
