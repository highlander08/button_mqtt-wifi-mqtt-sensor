Perfeito! Aqui está uma versão **mais visual e completa do README.md**, pronta para GitHub, incluindo **diagramas de conexão e fluxo de dados**:

```markdown
# Monitor de Batimentos e SpO2 com Raspberry Pi Pico W

Este projeto implementa um **oxímetro de pulso digital** usando o sensor **MAX30102** e publica os valores de **batimentos cardíacos (BPM)** e **oxigenação sanguínea (SpO2)** em um broker **MQTT**, utilizando **Wi-Fi** do Raspberry Pi Pico W.

---

## 🔹 Funcionalidades

- Conexão automática à rede Wi-Fi.
- Leitura do sensor MAX30102 via I2C.
- Cálculo de BPM e SpO2 com detecção de dedo.
- Publicação dos dados em tempo real via MQTT.
- Exibição de informações detalhadas no console.

---

## 🔹 Requisitos de Hardware

- **Raspberry Pi Pico W**
- **Sensor MAX30102**
- Conexão I2C:
  - SDA → GPIO2
  - SCL → GPIO3
- Resistores pull-up (se necessário)

### 🔌 Diagrama de Conexão

```

Raspberry Pi Pico W          MAX30102

---

GPIO2 (SDA)      --------> SDA
GPIO3 (SCL)      --------> SCL
GND              --------> GND
3.3V             --------> VCC

````

---

## 🔹 Requisitos de Software

- SDK do **Raspberry Pi Pico (C/C++)**
- Bibliotecas:
  - `pico/stdlib.h`
  - `pico/cyw43_arch.h` (Wi-Fi)
  - `lwip/apps/mqtt.h` (MQTT)
  - `lwip/ip_addr.h` e `lwip/dns.h` (DNS)
- Compilador compatível com o Raspberry Pi Pico

---

## 🔹 Configuração

### Wi-Fi

Edite as credenciais no código:

```c
#define WIFI_SSID "Navega+ Highlander 2G"
#define WIFI_PASSWORD "Blessedhr10@"
````

### MQTT

Defina o broker e o tópico:

```c
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC "embarca/batimentos"
```

### MAX30102

* Endereço I2C padrão: `0x57`
* Configuração de ganho do LED: `LED_PA 0x1F`

---

## 🔹 Como Usar

1. Conecte o MAX30102 ao Raspberry Pi Pico W conforme o diagrama.
2. Configure as credenciais Wi-Fi e MQTT no código.
3. Compile o código com o **Pico SDK**.
4. Faça upload para o Pico W.
5. Abra o terminal para visualizar logs:

```
RED=12304 IR=12881 BPM= 72.0 SpO2≈ 98.0% [DC_IR=9792 RMS_IR=6376 SNR=65% Finger=1]
[MQTT] Publicando: [{"batimento": 72, "oximetro": 98}]
```

---

## 🔹 Fluxo de Dados

```mermaid
flowchart LR
    Dedo --> MAX30102[Sensor MAX30102]
    MAX30102 --> PicoW[Raspberry Pi Pico W]
    PicoW --> Processamento[Processamento de Sinais]
    Processamento --> MQTT[Publicação MQTT]
    MQTT --> Broker[Broker MQTT (ex: emqx.io)]
    Broker --> Dashboard[Dashboard ou Aplicativo]
```

* **Dedo:** O sinal óptico é detectado pelo sensor.
* **MAX30102:** Coleta os dados de luz vermelha e infravermelha.
* **Pico W:** Processa os sinais, calcula BPM e SpO2.
* **MQTT:** Publica os valores em formato JSON.
* **Broker:** Recebe os dados e disponibiliza para dashboards ou apps.

---

## 🔹 Observações

* Certifique-se de que o sensor esteja bem posicionado no dedo.
* Valores de BPM e SpO2 só são publicados se forem válidos.
* Frequência de leitura: 100 Hz.
* Inclui filtragem e validação de pulso para maior confiabilidade.

---

## 🔹 Licença

Este projeto está disponível sob a licença **MIT**.

---
```
