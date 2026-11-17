// comentários no repositório github

// CHECAR A SEQUENCIA DO CÓDIGO ANTES DE COMITAR

#include <Arduino.h>  // biblioteca principal do arduino
#include <lmic.h>     // biblioteca para comunicação LoRaWAN
#include <hal/hal.h>  // biblioteca para acesso ao hardware específico da placa
#include <Wire.h>     // biblioteca para comunicação I2C, usada pelo OLED
#include <Adafruit_GFX.h>    // biblioteca gráfica, usada para desenhar no display OLED
#include <Adafruit_SSD1306.h> // biblioteca para controlar o display OLED SSD1306
#include <Adafruit_ADS1X15.h> // biblioteca para ADS1115

#define dummy 0 // dummy testar display sem precisar do sensor, 1==dummy, 0 == sensor real
#define OLED_SDA 4 // pino de dados do OLED, conectado ao GPIO4 do TTGO
#define OLED_SCL 15 // pino de clock do OLED, conectado ao GPIO15 do TTGO
#define OLED_RST 16 // pino de reset do OLED, conectado ao GPIO16 do TTGO
#define SCREEN_WIDTH 128 // largura do display OLED, em pixels
#define SCREEN_HEIGHT 64 // altura do display OLED, em pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//#define LMIC_DEBUG_LEVEL 1
#define LORA_GAIN 10

#include <DHT.h>
#define DHT_PIN 13
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

Adafruit_ADS1115 ads; // leitura do CO pelo ADS1115
#define MICS_NH3_PIN  35
#define MICS_NO2_PIN  36

/*
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif
*/

// j verificará se há algum trabalho de transmissão em andamento antes de enviar o novo pacote
// envio do pacote através da função LMIC_setTxData2, após o envio outro é agendado após um intervalo TX_INTERVAL


// LMIC ------------------------------------------------------------------------------------------
//nwmskey e appskey são as chaves de rede e de aplicação para autenticar nosso dispositivo na rede lora
// o devaddr é device adress, um idetenficiador unico do dispositivo na rede.
static const u1_t PROGMEM NWKSKEY[16] = {0x6D, 0xDD, 0xD0, 0x2D, 0xE9, 0x20, 0x62, 0x27, 0x90, 0x51, 0x29, 0xC7, 0x30, 0xD2, 0xA4, 0xB7};
static const u1_t PROGMEM APPSKEY[16] = {0x96, 0xCB, 0x3E, 0x51, 0xD0, 0x71, 0xB2, 0x5B, 0xD8, 0x52, 0x5A, 0x67, 0x20, 0x9E, 0x6D, 0xF9};
static const u4_t DEVADDR = 0x260DF8C4;
// funções OTA, necessárias por serem chamadas pela biblioteca LMIC
// não compila sem elas, vazias pelos endereços serem definidos fixamente
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


// buffer onde os dados do pacote serão armazenados antes de serem enviados via LoRaWAN.
// está sobrando espaço, pois usaremos apenas 6 bytes para enviar os dados
uint8_t txBuffer[9]; // 24, 6
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60; // freq envio de dados LoRaWAN
// LMIC ------------------------------------------------------------------------------------------


void buildPacket(uint8_t txBuffer[9]);
void do_send(osjob_t* j); // envio pacote de dados
void setupLoRaWAN();


// definição global valores sensores
float temperatura = 0.0;
float umidade = 0.0;
uint16_t co = 0;
uint16_t co_min = 65535; // == -1, valor total na faixa de 32768 posit e negativo == 2^15
uint16_t co_max = 0, co_quant_leituras = 0;
uint32_t co_soma = 0;

const unsigned short DELAY_DISPLAY = 200;

void setup() {
    Serial.begin(115200);
    Wire.begin(OLED_SDA, OLED_SCL); // comunicação display

    // inicialização ADS1115
    Serial.println("--> init ADS1115");
    if (!ads.begin()) {
        Serial.println("--> erro leitura ADS1115");
        while (1);
    }
    // faixa de tensao
    ads.setGain(GAIN_TWOTHIRDS); // 6.144 V, 1 bit = 0.1875 mV
    //ads.setMode == modo continuo?
    ads.setDataRate(RATE_ADS1115_64SPS); // 128 padrao

    delay(100); // delay para garantir que a comunicação aconteça antes de prosseguir

    // inicialização oled
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)){ // 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(20, 30); // perto do centro do display
    display.print("MICS-6814 + DHT22");
    display.display();

    // inicialização DHT22
    Serial.println("--> init DHT22");
    dht.begin();

    setupLoRaWAN();
}


// não colocar delay no loop
void loop() {
    static unsigned long ultimoDHT = 0;
    static unsigned long ultimoCO = 0;
    static unsigned long ultimoDisplay = 0;
    unsigned long agora = millis();

    // pino 13 para GPIO do DHT umidade + temp
    if (agora - ultimoDHT >= 1000) {
        ultimoDHT = agora;

        float new_umidade = dht.readHumidity();
        float new_temp = dht.readTemperature();

        if (!isnan(new_umidade) && !isnan(new_temp)) {
            umidade = new_umidade;
            temperatura = new_temp;

            Serial.print("Temperatura: ");
            Serial.print(temperatura);
            Serial.print(" C | Umidade: ");
            Serial.print(umidade);
            Serial.println(" %");
        }
        else {
            Serial.println("--> erro leitura DHT22");
        }
    }

    // co
    if (agora - ultimoCO >= 100) {
        ultimoCO = agora;
        int16_t leitura = ads.readADC_SingleEnded(0); // canal A0
        //if (leitura < 0) leitura = 0; // evitar ruido
        co = (uint16_t) leitura;

        // atualiza minimo, maximo e quantidade de leituras para a media
        if (co < co_min)
            co_min = co;
        if (co > co_max)
            co_max = co;
        co_soma += co;
        co_quant_leituras++;

        Serial.print("CO: ");
        Serial.print(leitura);
        Serial.print(" | Tensão: ");
        Serial.print(ads.computeVolts(leitura), 3);
        Serial.println(" V");
    }

    // display ttgo
    if (agora - ultimoDisplay >= DELAY_DISPLAY) {
        ultimoDisplay = agora;
        float co_media = (co_quant_leituras > 0) ? (float)co_soma / co_quant_leituras : (float)co;

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("   MICS-6814 + DHT22");
        display.setCursor(0, 11);
        display.print("Temp: ");
        display.print(temperatura, 1);
        display.println(" C");
        display.setCursor(0, 22);
        display.print("Umd.: ");
        display.print(umidade, 1);
        display.println(" %");
        display.setCursor(0, 33);
        display.print("CO: ");
        display.print(co);
        display.print(" / ");
        display.print(ads.computeVolts((int16_t)co_media), 3);
        display.print(" V");
        display.setCursor(0, 44);
        display.print(co_min);
        display.print(" / ");
        display.print(co_max);
        display.print(" / ");
        display.print(co_media, 0);
        display.display();

        co_soma = 0;
        co_quant_leituras = 0;
    }

    // Executa rotina LoRaWAN, impede que o loop fique preso
    // processa eventos de rede (envio e recebimento de pacotes) + comunicação LoRaWAN
    os_runloop_once();
}


void buildPacket(uint8_t txBuffer[8]) { // 9
    // zera o buffer
    memset(txBuffer, 0, 8);

    // conversão em int para remover as virgulas
    int16_t tempInt = (int16_t)(temperatura * 100);
    uint16_t umidInt = (uint16_t)(umidade);

    // a parte alta é obtida deslocando os bits para a direita em 8 posições, bits mais significativos do valor.
    // A parte baixa é obtida usando uma operação AND com 0xFF (tempInt & 0xFF)
    txBuffer[0] = tempInt >> 8;
    txBuffer[1] = tempInt & 0xFF;
    txBuffer[2] = umidInt >> 8;
    txBuffer[3] = umidInt & 0xFF;
    txBuffer[4] = co >> 8;
    txBuffer[7] = co & 0xFF;
}



// LMIC ------------------------------------------------------------------------------------------

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,   // For TTGO 14, T-Beam 23
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND){ // checa se o rádio já está ocupado transmitindo algo. Se estiver, ele aborta o novo envio para não causar colisão
        Serial.println(F("OP_TXRXPEND, not sending"));
    //LoraStatus = "OP_TXRXPEND, not sending";
    }
    else{
        buildPacket(txBuffer);
        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);  // coloca o pacote na fila de envio na Porta 1. O parâmetro 0 indica que o envio não é confirmado, ou seja, o dispositivo não espera por um reconhecimento do servidor para considerar o envio bem-sucedido. Se fosse 1, o dispositivo aguardaria um reconhecimento do servidor para confirmar que o pacote foi recebido corretamente. O uso de envios não confirmados pode ser útil para economizar energia e reduzir a latência, mas pode resultar em perda de pacotes se houver interferência ou problemas de comunicação na rede.
        Serial.println("Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setupLoRaWAN(){
  Serial.println();
  Serial.println(F("--> init LMIC"));

  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  Serial.println(F("--> init os_init() + LMIC_reset()"));

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
 // LMIC_setupChannel(8, 994000000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  2);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);

#elif defined(CFG_as923)
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here

#else
# error Region not supported
#endif
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF9, LORA_GAIN);
  Serial.println(F("--> config LoRa done"));
  // Start job
  do_send(&sendjob);
}

// LMIC ------------------------------------------------------------------------------------------
