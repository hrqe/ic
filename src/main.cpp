#include <Arduino.h>  // biblioteca principal do arduino, para usar funções como pinMode, digitalWrite, delay, etc.
#include <lmic.h>     // biblioteca para comunicação LoRaWAN
#include <hal/hal.h>  // biblioteca para acesso ao hardware específico da placa
#include <Wire.h>     // biblioteca para comunicação I2C, usada tanto pelo OLED quanto pelo BMP280
#include <Adafruit_BMP280.h> // biblioteca para usar o sensor de temperatura e pressão BMP280
#include <Adafruit_GFX.h>    // biblioteca gráfica, usada para desenhar no display OLED
#include <Adafruit_SSD1306.h> // biblioteca para controlar o display OLED SSD1306

//#define  LMIC_DEBUG_LEVEL = 1
#define LMIC_DEBUG_LEVEL 1
#define CFG_au915
#define LORA_GAIN 20

#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// aqui temos defnições para o DISPLAY, PINOS, LEDS, DUMMY
#define dummy 0 // uso de dummy, para testar o display sem precisar do sensor, 1 p/ dummy, 0 p/ sensor real
#define OLED_SDA 4 // pino de dados do OLED, conectado ao GPIO4 do TTGO
#define OLED_SCL 15 // pino de clock do OLED, conectado ao GPIO15 do TTGO
#define OLED_RST 16 // pino de reset do OLED, conectado ao GPIO16 do TTGO
#define SCREEN_WIDTH 128 // largura do display OLED, em pixels
#define SCREEN_HEIGHT 64 // altura do display OLED, em pixels


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST); // inicio do display, usando os pinos definidos acima
Adafruit_BMP280 _sensor; // definindo o sensor


void buildPacket(uint8_t txBuffer[9]);
void do_send(osjob_t* j); // Declaração da função do_send, que é responsável por enviar o pacote de dados via LoRaWAN. O parâmetro j é um ponteiro para uma estrutura osjob_t, que representa um trabalho agendado para execução. Essa função será chamada quando for necessário enviar um pacote de dados, e ela verificará se há algum trabalho de transmissão em andamento antes de enviar o novo pacote. Se não houver nenhum trabalho em andamento, ela construirá o pacote usando a função buildPacket e o enviará usando a função LMIC_setTxData2 da biblioteca LMIC. Após o envio, ela agendará o próximo envio para ocorrer após um intervalo de tempo definido por TX_INTERVAL.

//nwmskey e appskey são as chaves de rede e de aplicação para autenticar nosso dispositivo na rede lora
// o devaddr é device adress, um idetenficiador unico do dispositivo na rede.
// config device antigo:
//static const PROGMEM u1_t NWKSKEY[16] = {0x1D, 0x4C, 0x02, 0xD0, 0x54, 0xD9, 0x37, 0xC4, 0xFB, 0x36, 0xEA, 0x48, 0xFD, 0xDB, 0x94, 0x65};
//static const u1_t PROGMEM APPSKEY[16] = {0x7A, 0x5D, 0xB6, 0xA9, 0xA5, 0x12, 0x6F, 0xC9, 0x85, 0x86, 0x5A, 0x63, 0x38, 0x17, 0xC2, 0xB5};
//static const u4_t DEVADDR = 0x260DC7A4;

static const u1_t PROGMEM NWKSKEY[16] = {0x6D, 0xDD, 0xD0, 0x2D, 0xE9, 0x20, 0x62, 0x27, 0x90, 0x51, 0x29, 0xC7, 0x30, 0xD2, 0xA4, 0xB7};
static const u1_t PROGMEM APPSKEY[16] = {0x96, 0xCB, 0x3E, 0x51, 0xD0, 0x71, 0xB2, 0x5B, 0xD8, 0x52, 0x5A, 0x67, 0x20, 0x9E, 0x6D, 0xF9};
static const u4_t DEVADDR = 0x260DF8C4;

// definição dos nossos pacotes de dados, e do intervalo de envio
int n_packet=0;
// aqui são funções OTA, como não estou usando OTA, elas estão vazias.
// mas são necessarias pois são chamadas pela biblioteca LMIC para obter as chaves de autenticação e o endereço do dispositivo. Se não fossem definidas, o código não compilava, pois a biblioteca espera que essas funções existam, mesmo que não sejam usadas. Elas estão vazias porque estamos usando chaves e endereço fixos definidos nas constantes NWKSKEY, APPSKEY e DEVADDR, em vez de obter esses valores dinamicamente por meio de OTA (Over-The-Air Activation).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// buffer onde os dados do pacote serão armazenados antes de serem enviados via LoRaWAN.
// está sobrando espaço, pois usaremos apenas 6 bytes para enviar os dados
uint8_t txBuffer[24];
static osjob_t sendjob;

const unsigned TX_INTERVAL = 15; // 300=5 minutos esse intervalo define a frequência com que os pacotes de dados serão enviados via LoRaWAN. No exemplo, o intervalo é definido como 15 segundos, o que significa que o dispositivo tentará enviar um pacote de dados a cada 15 segundos. No entanto, devido às limitações de ciclo de trabalho (duty cycle) impostas pelas regulamentações de rádio, o intervalo real entre os envios pode ser maior se o dispositivo atingir o limite de transmissão permitido. É importante ajustar esse intervalo de acordo com as necessidades do aplicativo e as restrições da rede para garantir uma comunicação eficiente e em conformidade com as regulamentações.

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
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
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
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
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

int count = 5;

float temperatura = 0.0;
float pressao = 0.0;

float pegarpressao(){
  if(dummy){
    return 1013.25; // valor fixo para pressão
  }
  else{
    return _sensor.readPressure() / 100.0F; // converte de Pa para hPa
  }
}

float pegartemperatura(){
  if(dummy){
    return 25.0; // valor fixo para temperatura
  }
  else{
    return _sensor.readTemperature();
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running

  //displayValues();

      if (LMIC.opmode & OP_TXRXPEND)  // checa se o rádio já está ocupado transmitindo algo. Se estiver, ele aborta o novo envio para não causar colisão
      {
        Serial.println(F("OP_TXRXPEND, not sending"));
        //LoraStatus = "OP_TXRXPEND, not sending";
      }
      else
      {

        temperatura = pegartemperatura();
        pressao = pegarpressao();

        buildPacket(txBuffer);
        n_packet++;
        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);  // coloca o pacote na fila de envio na Porta 1. O parâmetro 0 indica que o envio não é confirmado, ou seja, o dispositivo não espera por um reconhecimento do servidor para considerar o envio bem-sucedido. Se fosse 1, o dispositivo aguardaria um reconhecimento do servidor para confirmar que o pacote foi recebido corretamente. O uso de envios não confirmados pode ser útil para economizar energia e reduzir a latência, mas pode resultar em perda de pacotes se houver interferência ou problemas de comunicação na rede.
        Serial.println(F("Packet queued"));

      }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setupLoRaWAN()
{
  Serial.println();
  Serial.println(F("--> init LMIC"));
  // LMIC init
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
#elif defined(CFG_kr920)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif
  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF9, LORA_GAIN);

  Serial.println(F("config LoRa done"));

  // Start job
  do_send(&sendjob);

}


void setup() {
  Serial.begin(115200); // começamso a conexão com o monitor serial, para debug e leitura de dados
  Wire.begin(OLED_SDA, OLED_SCL); // começamos a comunicar com o display
  delay(100); // delay para garantir que a comunicação aconteça antes de prosseguir

  if(dummy == 0) {
    if(!_sensor.begin(0x76)) {  // conectamos ao sensor BMP280, 0x76 é o endereço padrão dele, 0x77 é o outro endereço possível, depende de como o sensor está configurado. Se a inicialização falhar, ele entra em um loop infinito piscando o LED para indicar erro e imprime mensagens de debug no monitor serial para ajudar a identificar o problema. Ele também fornece instruções sobre como verificar as conexões do sensor para garantir que estejam corretas.
      if(!_sensor.begin(0x77)) {
        Serial.println("ERRO: BMP280 não encontrado!");
        Serial.println("Verifique as conexões:");
        Serial.println("  VCC → 3.3V");
        Serial.println("  GND → GND");
        Serial.println("  SDA → GPIO4");
        Serial.println("  SCL → GPIO15");
      }
    }
    else {
      Serial.println("Sensor BMP280 inicializado com sucesso!");
    }
  }


  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);


  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(20, 30); // perto do centro do display
  display.print("Coleta temperatura e pressao");
  display.display();

  setupLoRaWAN();

}


void loop() {
  // coleto os valores de temp e pressão do sensor // uso dummy, depende.
  temperatura = pegartemperatura();
  pressao = pegarpressao();

  // Mostra no display
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== BMP280 SENSOR ===");
  display.setCursor(0, 16);
  display.print("Temp: ");
  display.print(temperatura, 1);
  display.println(" C");
  display.setCursor(0, 32);
  display.print("Pressao: ");
  display.print(pressao, 0);
  display.println(" hPa");
  //display.setCursor(0, 48);
  //display.print("=-=-=-=-=");
  display.display();

  // Executa rotina LoRaWAN
  os_runloop_once(); // esse comando é essencial para que a biblioteca LMIC funcione corretamente. Ele processa os eventos de rede, como o envio e recebimento de pacotes, e garante que a comunicação LoRaWAN ocorra de forma eficiente. Sem essa chamada, o dispositivo não seria capaz de enviar ou receber dados via LoRaWAN, e a funcionalidade de comunicação sem fio não funcionaria como esperado.
  // se ele não estivesse aqui o loop ficaria preso, e não conseguiria processar os eventos de rede, o que impediria o envio e recebimento de pacotes via LoRaWAN. Isso é crucial para garantir que a comunicação sem fio funcione corretamente, permitindo que o dispositivo envie os dados coletados para a rede e receba quaisquer mensagens ou comandos do servidor.
}


void buildPacket(uint8_t txBuffer[9]) {
  // Zera todo o buffer primeiro
  //comentario
  memset(txBuffer, 0, 9);

  // Converte temperatura removendo as vigrulas ( 25.34 -> 2534)
  int16_t tempInt = (int16_t)(temperatura * 100);

  // Converte pressão removendo as virgulas ( 1013.25 -> 1013)
  uint16_t pressInt = (uint16_t)(pressao);

  // como esses dados foram guardados em 2 bytes, vamos dividir esse valor em dois bytes para envia-lo
  //vamos separar em parte alta e parte baixa, usando operações bitwise
  // a parte alta é obtida deslocando os bits para a direita em 8 posições (tempInt >> 8), o que nos dá os bits mais significativos do valor.
  // A parte baixa é obtida usando uma operação AND com 0xFF (tempInt & 0xFF),
  // que nos dá os bits menos significativos do valor.
  // Temperatura ocupa 2 bytes
  txBuffer[0] = tempInt >> 8;
  txBuffer[1] = tempInt & 0xFF;

  // Pressão ocupa 2 bytes
  txBuffer[2] = pressInt >> 8;
  txBuffer[3] = pressInt & 0xFF;

  // Contador de pacotes (2 bytes)

  txBuffer[4] = 4;
  //txBuffer[4] = n_packet >> 8;
  //txBuffer[5] = n_packet & 0xFF;
}
