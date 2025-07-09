i) Uma breve descrição do funcionamento e uso para quem quiser reproduzir.

Descrição do Projeto
Componentes Utilizados:
DHT11: Sensor de temperatura e umidade.
ESP32: Microcontrolador com conectividade Wi-Fi.
ProtoBoard: Para montagem do circuito.
LED: Indicador visual.
LCD: Display para mostrar dados de temperatura e umidade.
Protocolo MQTT: Para comunicação com a aplicação.

Funcionamento:
Leitura dos Dados: O DHT11 mede a temperatura e a umidade do ambiente. O ESP32 lê esses dados através de uma das suas entradas digitais.
Processamento: O ESP32 processa os dados recebidos do DHT11.
Exibição Local: Os dados são exibidos em tempo real no LCD, mostrando a temperatura e a umidade.
Indicação Visual: O LED pode ser usado para indicar diferentes estados, como sucesso na leitura dos dados ou problemas de conexão.
Envio via MQTT: Utilizando a conectividade Wi-Fi do ESP32, os dados são enviados para um broker MQTT, onde podem ser acessados por uma aplicação externa.
Passo a Passo para Reproduzir:

Montagem do Hardware:

Conecte o DHT11 ao ESP32 conforme o diagrama de pinos: geralmente VCC ao 3.3V, GND ao GND e o pino de dados a um dos pinos digitais do ESP32.
Conecte o LCD ao ESP32 usando um adaptador I2C para simplificar a conexão.
Conecte o LED ao ESP32, com um resistor em série para limitar a corrente.
Monte todos os componentes na protoBoard.
Configuração do Software:

Instale o Arduino IDE e adicione a placa ESP32 na gerenciador de placas.
Instale as bibliotecas necessárias: DHT sensor library, Adafruit Unified Sensor, Wire, LiquidCrystal_I2C, e PubSubClient para MQTT.
Configure a rede Wi-Fi e o broker MQTT no código.

Operação:
Compile e faça o upload do código para o ESP32.
Abra o monitor serial para verificar as mensagens de depuração.
O LCD deve exibir a temperatura e a umidade.
O LED indicará erros de leitura do sensor.
Os dados serão enviados via MQTT para o broker configurado.

Considerações Finais:
Certifique-se de que todos os componentes estão corretamente conectados e funcionando.
Verifique a configuração do broker MQTT e a rede Wi-Fi.
Use uma aplicação cliente MQTT para monitorar os dados enviados pelo ESP32.


ii) O software desenvolvido e a documentação de código.
Código para o ESP32:

#include <SimpleDHT.h>//lib
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define TEMP_MAX          45
#define TIMOUT_DHT11      2000
#define TIMOUT_MQTT       5000
#define TIMOUT_WIFI_INIT  500
#define TEMP_MAX_WIFI     20

#define I2C_SDA 12
#define I2C_SCL 13

#define LED_BUILTIN 2
#define SINAL_RELE 33
//Descomente as linhas abaixo para o tipo do sensor DHT que você estiver usando! - TEST
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define MSG_BUFFER_SIZE	(50)

// DHT Sensor
const int DHTPin = 25;
// the setup function runs once when you press reset or power the board
//Objeto que realiza a leitura da umidade e temperatura
SimpleDHT11 dht(DHTPin);
// Definir o endereço do LCD para 0x27 para um display de 16 caracteres e 2 linhas
LiquidCrystal_I2C lcd(0x27, 16, 2);

const char* ssid     = "Narnia"; // Change this to your WiFi SSID
const char* password = "aafcj7197"; // Change this to your WiFi password
const char* mqtt_server = "broker.emqx.io";
const char *mqtt_username = "admin"; //"emqx";
const char *mqtt_password = "public";

//Tópicos
const char *humidity_ch = "sensor/humidity";
const char *temperature_ch = "sensor/temperature";
const char *ventoinha = "ventoinha";
const char *habVentoinha = "habVentoinha";
const char *erroDHT11 = "erroDHT11";

WiFiClient espClient;
PubSubClient client(espClient);
char msg[MSG_BUFFER_SIZE];

int readDhtSensor();
bool timerTimout(const unsigned long initTimer, const unsigned long timout);
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);

unsigned long tempInicio, tempInicioReconnect;
uint8_t cont = 0;
bool flagMQTT = false;
unsigned char toggle = 0, setOutput = 0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SINAL_RELE, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  digitalWrite(SINAL_RELE, LOW);   // turn the LED off by making the voltage LOW
  
  Wire.begin(I2C_SDA,I2C_SCL);
  Serial.begin(115200);
  Serial.println("Inicializando...");
  lcd.begin();
  // set cursor to first column, first row
  lcd.clear();
  lcd.setCursor(0,0);
  // print message
  lcd.println("Inicializando...");  

  // We start by connecting to a WiFi network
  Serial.println("******************");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  tempInicio = millis();     //inicializa o timer

  while (WiFi.status() != WL_CONNECTED && cont < TEMP_MAX_WIFI) {
    if (timerTimout(tempInicio, TIMOUT_WIFI_INIT))  //se passou o tempo de escrita de um pontinho    
    {
      Serial.print(".");
      tempInicio = millis();     //inicializa o timer
      cont++;
    }         
  }
  if(cont >= TEMP_MAX_WIFI)
    Serial.println("WiFi disconnected!");
  else
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP()); 

    //Inicializa a conexao com o broker MQTT
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback); 
    client.subscribe(ventoinha); 
    flagMQTT = true;  
  }  

  tempInicio = tempInicioReconnect = millis();     //inicializa o timer
  //tempInicioReconnect = millis();     //inicializa o timer
}

// the loop function runs over and over again forever
int errDHT;
float h, c;
uint8_t estado = 1;   //estado inicial da maquina de estados

void loop() 
{
  if (!client.connected()) //se nao esta conectado ao broker MQTT
  {
    if(timerTimout(tempInicioReconnect, TIMOUT_MQTT))  //se estourou o timer
    {
      reconnect();
      tempInicioReconnect = millis();     //inicializa o timer
    }
  }
  else if(timerTimout(tempInicioReconnect, TIMOUT_MQTT)) //se estiver conectado e estourou o timer
  {
    tempInicioReconnect = millis();     //inicializa o timer
    Serial.println("MQTT connected to server: ");
    Serial.print(mqtt_server);
  }

  client.loop();

  if(timerTimout(tempInicio, TIMOUT_DHT11))  //se passou o tempo para a leitura do DHT11
  {
    errDHT = readDhtSensor();
    if(errDHT != SimpleDHTErrSuccess) //se falhou a leitura do DHT11
    {
      //informa o erro pela serial
      Serial.print("Erro de leitura do DHT11!, err = "); 
      Serial.println(errDHT);

      //informa o erro no LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Erro no DHT11!");  

      //envio da mensagem de erro de leitura do DHT11 
      snprintf(msg, MSG_BUFFER_SIZE, "1");          
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish(erroDHT11, msg);          
    } 
    else  //se leu o DHT11 com sucesso
    {
      //envio da mensagem sem erro de leitura do DHT11 
      snprintf(msg, MSG_BUFFER_SIZE, "0");          
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish(erroDHT11, msg);   

      //exibe valores pela serial
      Serial.print((float)c); Serial.println(" *C ");
      Serial.print((float)h); Serial.println(" H");

      //Exibe os valores no LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Temp: ");   
      lcd.print(c); 
      lcd.print(" °C"); 
      lcd.setCursor(0,1); //vai para a primeira coluna da segunda linha
      lcd.print("Umid: ");                  
      lcd.print(h);
      lcd.print('%');

      if(flagMQTT)  //se a flag estiver setada
      {
        //envio da temperatura/uidadade da leitura do DHT11 via MQTT 
        snprintf (msg, MSG_BUFFER_SIZE, "%.2f", c);
        Serial.print("Publish message: ");
        Serial.println(msg);          
        client.publish(temperature_ch, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "%.2f", h);
        Serial.print("Publish message: ");
        Serial.println(msg);                    
        client.publish(humidity_ch, msg);      
      }
      if(c > TEMP_MAX)
      {
        toggle = 1;
        Serial.print("Alta temperatura!"); 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("High Temp!");   

        if(!setOutput)  //se a ventoinha nao estiver habilitada no aplicativo
        {
          digitalWrite(SINAL_RELE, HIGH);   //Liga a ventoinha
          digitalWrite(LED_BUILTIN, HIGH);   //Liga o LED do kit do ESP32
          if(flagMQTT)  //se a flag estiver setada
          {
            //envio da mensagem que ligou a ventoinha, alta temperatura  
            snprintf(msg, MSG_BUFFER_SIZE, "1");          
            Serial.print("Publish message: ");
            Serial.println(msg);
            client.publish(ventoinha, msg);
          }
        }
      }
      else  //se a temperatura estiver normal
      {
        toggle = 0;
        if(!setOutput)  //se a ventoinha nao estiver habilitada
        {
          digitalWrite(SINAL_RELE, LOW);   //Desliga a ventoinha
          digitalWrite(LED_BUILTIN, LOW);   //Desliga o LED do kit do ESP32          
          if(flagMQTT)  //se a flag estiver setada
          {
            //envio da mensagem que desligou a ventoinha, alta temperatura  
            snprintf(msg, MSG_BUFFER_SIZE, "0");  
            Serial.print("Publish message: ");
            Serial.println(msg);
            client.publish(ventoinha, msg); 
          }
        }    
      }               
    }  
    tempInicio = millis();     //inicializa o timer
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    //if (client.connect(clientId.c_str())) {
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(ventoinha);
      flagMQTT = true; 
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      flagMQTT = false; 
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String str1 =  String(topic) , str2 = String(habVentoinha), str3 = String(ventoinha);

  Serial.print("Messagem recebida [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  //Liga o LED e a ventoinha se recebeu 1 no primeiro caracter
  if(str1 == str3)
  {
    if ((char)payload[0] == '1') {
      Serial.println("Recebeu o comando: Ligar ventoinha");
      digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on 
      digitalWrite(SINAL_RELE, HIGH);   //Liga a ventoinha
      toggle = 1;
    } else {
      Serial.println("Recebeu o comando: Desligar a ventoinha");
      digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
      digitalWrite(SINAL_RELE, LOW);   //Desliga a ventoinha
      toggle = 0;
    }
  }
  else if(str1 == str2)
  {
    if ((char)payload[0] == '1') {
      Serial.println("Recebeu o comando: Habilitar ventoinha");
      setOutput = 1;
    } else {
      Serial.println("Recebeu o comando: Habilitar a ventoinha");
      setOutput = 0;
    }    
  }
}

int readDhtSensor()
{
  //declara variaveis que receberão a nova temperatura e humidade
  float novoC, novoH;
  //h = humidade; c = temperatura em graus celsius
  //float h, c;
  //aguarda 250ms
  //delay(250);
  //lê valores do sensor dht22 para as variaveis &c e &h
  int err = dht.read2(DHTPin, &novoC, &novoH, NULL);
  //verifica se ocorreu algum erro
  if (err != SimpleDHTErrSuccess) 
  {
    //informa o erro pela serial
    // Serial.print("Erro de leitura do DHT11!, err = "); 
    // Serial.println(err);

    // //informa o erro no LCD
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("Erro no DHT11!"); 
    //TODO: envio da mensagem de erro de leitura do DHT11 via MQTT
    return err;
  }
  //se foi possível ler a temperatura, então atribui para as variáveis
  c = novoC;
  h = novoH;
  // //exibe valores pela serial
  // Serial.print((float)c); Serial.println(" *C ");
  // Serial.print((float)h); Serial.println(" H");

  // //Exibe os valores no LCD
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Temp: ");   
  // lcd.print(c); 
  // lcd.setCursor(0,1); //vai para a primeira coluna da segunda linha
  // lcd.print(h);
  //TODO: envio da mensagem de leitura do DHT11 via MQTT

  return err;

  //aguarda 250ms
  //delay(250);
}

/*
** ================================================================================================================
**     Funcao       :  timerTimout
**     Descricao 	  :  Funcao que verifica se estourou o timer
**     @parametro
**         initTimer       - Tempo inicial em milisegundos que começou a contar o timer.
**     @parametro
**         timout          - Tempo final de estouro do timer.
**     Retorno      :  Do tipo booleano (bool) para saber se estourou ou nao o timer
** ================================================================================================================
*/
bool timerTimout(const unsigned long initTimer, const unsigned long timout)
{  
  return(millis() - initTimer > timout ? true: false);
}

iii) A descrição do hardware utilizado

ESP32: Microcontrolador com conectividade WiFi e Bluetooth.
DHT11: Sensor de temperatura e umidade.
Display LCD 16x2: Exibe as leituras de temperatura e umidade.
Cooler: Atuador que resfria o ambiente quando a temperatura ultrapassa 45°C.
Fonte de Alimentação: Para o ESP32 e o display.
Protoboard e Jumpers: Para conectar os componentes eletrônicos.
Caixas e Suportes 3D: Para acomodar os componentes, garantindo uma montagem organizada.


iv) A documentação das interfaces, protocolos e módulos de comunicação

MQTT: Protocolo de comunicação para envio e recebimento de dados pela internet.
Broker MQTT (EMQX): Facilita a comunicação entre o ESP32 e o aplicativo móvel.
Tópicos MQTT:
sensor/temperatura: Publica a temperatura medida.
sensor/umidade: Publica a umidade medida.
ventoinha: Publica o estado do cooler (ligado/desligado).
habVentoinha: Controla remotamente o estado do cooler.
WiFi: Utilizado pelo ESP32 para conexão à internet.

v) O projeto deve possuir comunicação/controle via internet (TCP/IP) e uso do Protocolo MQTT
O projeto utiliza o protocolo MQTT para comunicação via internet (TCP/IP). O ESP32 publica dados de temperatura e umidade em tópicos específicos e recebe comandos para controle do cooler através de tópicos de controle. A comunicação é mediada pelo broker MQTT (EMQX), permitindo monitoramento e controle em tempo real pelo aplicativo IoT MQTT Panel.
