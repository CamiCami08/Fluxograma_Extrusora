#include <Adafruit_GFX.h>  // Biblioteca para gráficos
#include <UTFTGLUE.h>      // Biblioteca gráfica simplificada
#include <EEPROM.h>        // Inclui a biblioteca EEPROM para manipulação de memória não volátil
#include <max6675.h>       // Inclui a biblioteca MAX6675 para leitura de temperatura de sensores termopares K

#define RELAY1 22  // Cooler para a zona de alimentação
#define RELAY2 23  // Cooler para a zona 1
#define RELAY3 24  // Cooler para a zona 2
#define RELAY4 25  // Cooler para a zona 3
#define RELAY5 26  // Cooler para o filamento
#define RELAY6 27  // Reserva
#define RELAY7 28  // Reserva
#define RELAY8 29  // Motor Fuso

int soPin1 = 30;              // Pino SO (MISO) do Sensor 1
int sckPin1 = 31;             // Pino SCK do Sensor 1
int csPin1 = 32;              // CS do Sensor 1
int soPin2 = 33;              // Pino SO (MISO) do Sensor 2
int sckPin2 = 34;             // Pino SCK do Sensor 2
int csPin2 = 35;              // CS do Sensor 2
int soPin3 = 36;              // Pino SO (MISO) do Sensor 3
int sckPin3 = 37;             // Pino SCK do Sensor 3
int csPin3 = 38;              // CS do Sensor 3
int pinSSR1 = 39;             // Pino digital onde o relé SSR1 está conectado
int pinSSR2 = 40;             // Pino digital onde o relé SSR2 está conectado
int pinPotenciometro = A15;   // Define o pino do potenciômetro
const int pinPWMfuso = 44;    // Pino PWM para controle do motor do fuso
const int pinPWMbobina = 45;  // Pino PWM para controle do motor de bobinamento
int before = 47;              // Botão para voltar (pino 47)
int next = 48;                // Botão para avançar (pino 48)
int incremento = 49;          // Botão para incremento (pino 49)
int select = 50;              // Botão para selecionar/confirmar (pino 50)
int decremento = 51;          // Botão para decremento (pino 51)


UTFTGLUE myGLCD(0x0154, A2, A1, A3, A4, A0);  // Configuração dos pinos de controle do display
MAX6675 thermocouple1(sckPin1, csPin1, soPin1);
MAX6675 thermocouple2(sckPin2, csPin2, soPin2);
MAX6675 thermocouple3(sckPin3, csPin3, soPin3);

bool modoAutomatico = false;                 // Variável que indica se o modo automático está ativado
int page = 0;                                // Página atual do sistema
int opcaoModo = 0;                           // Opção escolhida para o modo automático (0: Não, 1: Sim)
int velocidadeFuso = 0;                      // Velocidade do motor do fuso (0-99)
int velocidadeBobinamento = 0;               // Velocidade do motor de bobinamento (0-99)
int tempMinZ1 = 0;                           // Temperatura Mínima Zona 1 (0-999)
int tempMaxZ1 = 0;                           // Temperatura Maxima Zona 1 (0-999)
int tempMinZ2 = 0;                           // Temperatura Mínima Zona 2 (0-999)
int tempMaxZ2 = 0;                           // Temperatura Maxima Zona 2 (0-999)
int tempMinZ3 = 0;                           // Temperatura Mínima Zona 3 (0-999)
int tempMaxZ3 = 0;                           // Temperatura Maxima Zona 3 (0-999)
int tempZ1 = 0;                              // Temperatura Zona 1 (0-999)
int tempZ2 = 0;                              // Temperatura Zona 2 (0-999)
int tempZ3 = 0;                              // Temperatura Zona 3 (0-999)
int digitoAtual = 1;                         // Indica qual dígito está sendo ajustado (1 = centenas, 2 = dezenas, 3 = unidades)
bool telaResumoDesenhada = false;            // Variável para controlar se a tela já foi desenhada
bool telaResumoAutomaticoDesenhada = false;  // Flag para saber se a tela já foi desenhada
int posicaoMemoriaSelecionada = 0;           // Posição de memória atualmente selecionada (0-9)
const int totalMemorias = 10;                // Total de posições de memória
int velocidadeBobinaAtualizado = 0;          // Atualiza velocidade da variável com a correção
unsigned long endTimer = 600000;             // Tempo do contador em milissegundos (10 minutos)
unsigned long startTimer = 0;                // Armazena o tempo inicial do contador
bool timer = false;                          // Variável que será true quando o tempo for atingido
int FSZ1 = 0;                                // Fatores de segurança da zona 1
int FSZ2 = 0;                                // Fatores de segurança da zona 2
int FSZ3 = 0;                                // Fatores de segurança da zona 3

void setup() {
  Serial.begin(9600);

  pinMode(select, INPUT_PULLUP);      // Define o pino do botão "selecionar" como entrada com pull-up
  pinMode(decremento, INPUT_PULLUP);  // Define o pino do botão "decremento" como entrada com pull-up
  pinMode(incremento, INPUT_PULLUP);  // Define o pino do botão "incremento" como entrada com pull-up
  pinMode(before, INPUT_PULLUP);      // Define o pino do botão "voltar" como entrada com pull-up
  pinMode(next, INPUT_PULLUP);        // Define o pino do botão "avançar" como entrada com pull-up
  pinMode(pinPWMfuso, OUTPUT);        //
  pinMode(pinPWMbobina, OUTPUT);      //
  pinMode(pinSSR1, OUTPUT);           // Define o pino como saída para a resistência 1
  pinMode(pinSSR2, OUTPUT);           // Define o pino como saída para a resistência 2
  pinMode(RELAY1, OUTPUT);            // Define o pino como saída para o cooler para a zona de alimentação
  pinMode(RELAY2, OUTPUT);            // Define o pino como saída para o cooler para a zona 1
  pinMode(RELAY3, OUTPUT);            // Define o pino como saída para o cooler para a zona 2
  pinMode(RELAY4, OUTPUT);            // Define o pino como saída para o cooler para a zona 3
  pinMode(RELAY5, OUTPUT);            // Define o pino como saída para o cooler para o filamento
  pinMode(RELAY6, OUTPUT);            // Defina o pina como saída para reserva
  pinMode(RELAY7, OUTPUT);            // Defina o pina como saída para reserva
  pinMode(RELAY8, OUTPUT);            // Define o pino como saída para o Motor Fuso
  digitalWrite(pinSSR1, HIGH);        // Inicia com o relé da resistência 1 desligado
  digitalWrite(pinSSR2, HIGH);        // Inicia com o relé da resistência 2 desligado
  digitalWrite(RELAY1, HIGH);         // Inicia com cooler para a zona de alimentação desligado
  digitalWrite(RELAY2, HIGH);         // Inicia com cooler para a zona 1 desligado
  digitalWrite(RELAY3, HIGH);         // Inicia com cooler para a zona 2 desligado
  digitalWrite(RELAY4, HIGH);         // Inicia com cooler para a zona 3 desligado
  digitalWrite(RELAY5, HIGH);         // Inicia com cooler para o filamento desligado
  digitalWrite(RELAY6, HIGH);         // Inicia com o reserva desligado
  digitalWrite(RELAY7, HIGH);         // Inicia com o reserva desligado
  digitalWrite(RELAY8, HIGH);         // Inicia com o Motor Fuso desligado

  myGLCD.InitLCD();         // Inicializa o display LCD
  myGLCD.setFont(BigFont);  // Define uma fonte grande
  telaApresentacao();       // Exibe a tela de apresentação
}

void coletaDados() {
  // Exibe os valores de temperatura no monitor serial
  Serial.print(tempZ1);
  Serial.print(",");

  Serial.print(tempZ2);
  Serial.print(",");

  Serial.print(tempZ3);
  Serial.print(",");

  // Verifica e exibe o estado dos relés no monitor serial
  if (digitalRead(pinSSR1) == LOW) {
    Serial.print("ON    ");
  } else {
    Serial.print("OFF    ");
  }
  Serial.print(",");

  if (digitalRead(pinSSR2) == LOW) {
    Serial.print("ON    ");
  } else {
    Serial.print("OFF    ");
  }
  Serial.print(",");

  // Verifica e exibe o estado dos relés no monitor serial
  Serial.print(digitalRead(RELAY1) == LOW ? "ON    " : "OFF    ");
  Serial.print(",");

  Serial.print(digitalRead(RELAY2) == LOW ? "ON    " : "OFF    ");
  Serial.print(",");

  Serial.print(digitalRead(RELAY3) == LOW ? "ON    " : "OFF    ");
  Serial.print(",");

  Serial.println(digitalRead(RELAY4) == LOW ? "ON    " : "OFF    ");
  delay(5000);  // Atraso de 5 segundo entre as leituras
}

void loop() {
  switch (page) {
    case 0:
      delay(4000);                   // Atraso de 4 segundos para exibição da tela de apresentação
      page = 1;                      // Passa para a tela de pergunta sobre o modo automático
      telaPerguntaModoAutomatico();  // Exibe a tela de pergunta sobre o modo automático
      break;
    case 1:
      verificarBotoesModoAutomatico();  // Verifica os botões na tela de modo automático
      break;
    case 2:
      verificarBotoesVelocidadeFuso();  // Verifica os botões na tela de velocidade do motor fuso
      break;
    case 3:
      verificarBotoesVelocidadeBobinamento();  // Verifica os botões na tela de velocidade do motor de bobinamento
      break;
    case 4:
      verificarBotoesTempMinZ1();  // Verifica os botões na tela de Temperatura Minima zona 1
      break;
    case 5:
      verificarBotoesTempMaxZ1();  // Verifica os botões na tela de Temperatura Maxima zona 1
      break;
    case 6:
      verificarBotoesTempMinZ2();  // Verifica os botões na tela de Temperatura Minima zona 2
      break;
    case 7:
      verificarBotoesTempMaxZ2();  // Verifica os botões na tela de Temperatura Maxima zona 2
      break;
    case 8:
      verificarBotoesTempMinZ3();  // Verifica os botões na tela de Temperatura Minima zona 3
      break;
    case 9:
      verificarBotoesTempMaxZ3();  // Verifica os botões na tela de Temperatura Maxima zona 3
      break;
    case 10:
      telaResumo();  // Mostra todas as condições de temperaturas escolhidas
      break;
    case 11:
      verificarBotoesSalvarPrograma();
      break;
    case 12:
      verificarBotoesTelaSelecaoMemoria();
      break;
    case 13:
      verificarBotoesTelaPerguntaConfirmarGravacao();
      break;
    case 14:
      telaConfirmacao();
      break;
    case 15:
      verificarBotoesTelaIniciarProcesso();
      break;
    case 16:
      processoExtrusao();
      break;
    case 17:
      verificarBotoesTelaSelecaoMemoriaAutomatico();
      break;
    case 18:
      verificarBotoesTelaPerguntaConfirmarMemoriaAutomatico();
      break;
    case 19:
      verificarBotoesTelaResumoAutomatico();
      break;
    case 20:
      telaEncerramento();
      encerrarProcesso();
      break;
  }
}

void telaApresentacao() {
  myGLCD.clrScr();                                            // Limpa a tela
  myGLCD.setColor(250, 250, 250);                             // Define a cor do texto
  myGLCD.print("SISTEMA DE CONTROLE EXTRUSORA", CENTER, 80);  // Título do sistema
  myGLCD.print("Camila e Cleiton", CENTER, 120);              // Nomes dos autores
  myGLCD.print("2024", CENTER, 227);                          // Ano de criação
}

void telaPerguntaModoAutomatico() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor da pergunta
  myGLCD.print("Utilizar modo automatico?", CENTER, 80);  // Pergunta sobre o modo automático
  myGLCD.setColor(255, 40, 0);                            // Define a cor do texto "Não"
  myGLCD.print("Nao", 100, 250);                          // Opção "Não" e posição na tela
  myGLCD.setColor(0, 128, 0);                             // Define a cor do texto "Sim"
  myGLCD.print("Sim", 330, 250);                          // Opção "Sim" e posição na tela
}

void verificarBotoesModoAutomatico() {
  page = 1;
  telaResumoAutomaticoDesenhada = false;
  if (digitalRead(before) == LOW) {  // Se o botão de voltar for pressionado
    page = 17;
    telaSelecaoMemoriaAutomatico();  // Exibe a tela de Seleção Memória modo Automático
  }
  if (digitalRead(next) == LOW) {  // Se o botão de avançar for pressionado
    opcaoModo = 1;                 // Define a opção como "Não"
    modoAutomatico = false;        // Modo automático desativado
    page = 2;                      // Passa para a tela de velocidade do motor fuso
    telaVelocidadeFuso();          // Exibe a tela de velocidade do motor fuso
  }
}

void telaVelocidadeFuso() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor do texto
  myGLCD.print("Velocidade do motor fuso:", CENTER, 80);  // Título da tela de velocidade do motor fuso
  char buffer[3];                                         // Buffer para armazenar a velocidade formatada
  sprintf(buffer, "%02d", velocidadeFuso);                // Formata a velocidade para dois dígitos
  myGLCD.setColor(255, 255, 0);                           // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                      // Exibe a velocidade
}

void verificarBotoesVelocidadeFuso() {
  if (digitalRead(select) == LOW) {            // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : 1;  // Alterna entre ajustar dezenas e unidades
    telaVelocidadeFuso();                      // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      velocidadeFuso = constrain(velocidadeFuso + 10, 0, 99);  // Incrementa a dezena
    } else {
      velocidadeFuso = constrain(velocidadeFuso + 1, 0, 99);  // Incrementa a unidade
    }
    telaVelocidadeFuso();  // Atualiza a tela
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      velocidadeFuso = constrain(velocidadeFuso - 10, 0, 99);  // Decrementa a dezena
    } else {
      velocidadeFuso = constrain(velocidadeFuso - 1, 0, 99);  // Decrementa a unidade
    }
    telaVelocidadeFuso();  // Atualiza a tela
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 1;                        // Retorna para a tela de pergunta sobre o modo automático
    telaPerguntaModoAutomatico();    // Exibe a tela de pergunta sobre o modo automático
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 3;                      // Passa para a tela de velocidade do motor de bobinamento
    telaVelocidadeBobinamento();   // Exibe a tela de velocidade do motor de bobinamento
  }
}

void telaVelocidadeBobinamento() {
  myGLCD.clrScr();                                                  // Limpa a tela
  myGLCD.setColor(250, 250, 250);                                   // Define a cor do texto
  myGLCD.print("Velocidade do motor de bobinamento:", CENTER, 80);  // Título da tela de velocidade do motor de bobinamento
  char buffer[3];                                                   // Buffer para armazenar a velocidade formatada
  sprintf(buffer, "%02d", velocidadeBobinamento);                   // Formata a velocidade para dois dígitos
  myGLCD.setColor(255, 255, 0);                                     // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                                // Exibe a velocidade
}

void verificarBotoesVelocidadeBobinamento() {

  if (digitalRead(select) == LOW) {            // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : 1;  // Alterna entre ajustar dezenas ou unidades
    telaVelocidadeBobinamento();               // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      velocidadeBobinamento = constrain(velocidadeBobinamento + 10, 0, 99);  // Incrementa a dezena
    } else {
      velocidadeBobinamento = constrain(velocidadeBobinamento + 1, 0, 99);  // Incrementa a unidade
    }
    telaVelocidadeBobinamento();  // Atualiza a tela
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      velocidadeBobinamento = constrain(velocidadeBobinamento - 10, 0, 99);  // Decrementa a dezena
    } else {
      velocidadeBobinamento = constrain(velocidadeBobinamento - 1, 0, 99);  // Decrementa a unidade
    }
    telaVelocidadeBobinamento();  // Atualiza a tela
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 2;                        // Retorna para a tela de velocidade do motor fuso
    telaVelocidadeFuso();            // Exibe a tela de velocidade do motor fuso
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 4;                      // Passa para a tela de configuração de temperaturas
    telaTempMinZ1();               // Exibe a tela de temperaturas
  }
}

void telaTempMinZ1() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor do texto
  myGLCD.print("Temperatura Minima Zona 1", CENTER, 80);  // Título da tela de temperaturas
  char buffer[4];                                         // Buffer para armazenar a temperatura formatada
  sprintf(buffer, "%03d", tempMinZ1);                     // Formata a velocidade para três dígitos
  myGLCD.setColor(255, 255, 0);                           // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                      // Exibe a temperatura
}

void verificarBotoesTempMinZ1() {

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : (digitoAtual == 2) ? 3
                                                              : 1;  // Alterna entre centenas, dezenas e unidades
    telaTempMinZ1();                                                // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      tempMinZ1 = constrain(tempMinZ1 + 100, 0, 999);  // Incrementa a centena
    } else if (digitoAtual == 2) {
      tempMinZ1 = constrain(tempMinZ1 + 10, 0, 999);  // Incrementa a dezena
    } else {
      tempMinZ1 = constrain(tempMinZ1 + 1, 0, 999);  // Incrementa a unidade
    }
    telaTempMinZ1();  // Atualiza a tela
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      tempMinZ1 = constrain(tempMinZ1 - 100, 0, 999);  // Decrementa a centena
    } else if (digitoAtual == 2) {
      tempMinZ1 = constrain(tempMinZ1 - 10, 0, 999);  // Decrementa a dezena
    } else {
      tempMinZ1 = constrain(tempMinZ1 - 1, 0, 999);  // Decrementa a unidade
    }
    telaTempMinZ1();  // Atualiza a tela
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 3;                        // Retorna para a tela de velocidade do motor fuso
    telaVelocidadeBobinamento();     // Exibe a tela de velocidade do motor fuso
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 5;                      // Passa para a tela de configuração de temperaturas
    telaTempMaxZ1();               // Exibe a tela de temperaturas
  }
}

void telaTempMaxZ1() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor do texto
  myGLCD.print("Temperatura Maxima Zona 1", CENTER, 80);  // Título da tela de temperaturas
  char buffer[4];                                         // Buffer para armazenar a temperatura formatada
  sprintf(buffer, "%03d", tempMaxZ1);                     // Formata a velocidade para três dígitos
  myGLCD.setColor(255, 255, 0);                           // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                      // Exibe a temperatura
}

void verificarBotoesTempMaxZ1() {

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : (digitoAtual == 2) ? 3
                                                              : 1;  // Alterna entre centenas, dezenas e unidades
    telaTempMaxZ1();                                                // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      tempMaxZ1 = constrain(tempMaxZ1 + 100, 0, 999);  // Incrementa a centena
    } else if (digitoAtual == 2) {
      tempMaxZ1 = constrain(tempMaxZ1 + 10, 0, 999);  // Incrementa a dezena
    } else {
      tempMaxZ1 = constrain(tempMaxZ1 + 1, 0, 999);  // Incrementa a unidade
    }
    telaTempMaxZ1();  // Atualiza a tela
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      tempMaxZ1 = constrain(tempMaxZ1 - 100, 0, 999);  // Decrementa a centena
    } else if (digitoAtual == 2) {
      tempMaxZ1 = constrain(tempMaxZ1 - 10, 0, 999);  // Decrementa a dezena
    } else {
      tempMaxZ1 = constrain(tempMaxZ1 - 1, 0, 999);  // Decrementa a unidade
    }
    telaTempMaxZ1();  // Atualiza a tela
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 4;                        // Retorna para a tela de velocidade do motor fuso
    telaTempMinZ1();                 // Exibe a tela de velocidade do motor fuso
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 6;                      // Passa para a tela de configuração de temperaturas
    telaTempMinZ2();               // Exibe a tela de temperaturas
  }
}

void telaTempMinZ2() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor do texto
  myGLCD.print("Temperatura Minima Zona 2", CENTER, 80);  // Título da tela de temperaturas
  char buffer[4];                                         // Buffer para armazenar a temperatura formatada
  sprintf(buffer, "%03d", tempMinZ2);                     // Formata a velocidade para três dígitos
  myGLCD.setColor(255, 255, 0);                           // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                      // Exibe a temperatura
}

void verificarBotoesTempMinZ2() {

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : (digitoAtual == 2) ? 3
                                                              : 1;  // Alterna entre centenas, dezenas e unidades
    telaTempMinZ2();                                                // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      tempMinZ2 = constrain(tempMinZ2 + 100, 0, 999);  // Incrementa a centena
    } else if (digitoAtual == 2) {
      tempMinZ2 = constrain(tempMinZ2 + 10, 0, 999);  // Incrementa a dezena
    } else {
      tempMinZ2 = constrain(tempMinZ2 + 1, 0, 999);  // Incrementa a unidade
    }
    telaTempMinZ2();  // Atualiza a tela
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      tempMinZ2 = constrain(tempMinZ2 - 100, 0, 999);  // Decrementa a centena
    } else if (digitoAtual == 2) {
      tempMinZ2 = constrain(tempMinZ2 - 10, 0, 999);  // Decrementa a dezena
    } else {
      tempMinZ2 = constrain(tempMinZ2 - 1, 0, 999);  // Decrementa a unidade
    }
    telaTempMinZ2();  // Atualiza a tela
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 5;                        // Retorna para a tela de velocidade do motor fuso
    telaTempMaxZ1();                 // Exibe a tela de velocidade do motor fuso
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 7;                      // Passa para a tela de configuração de temperaturas
    telaTempMaxZ2();               // Exibe a tela de temperaturas
  }
}

void telaTempMaxZ2() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor do texto
  myGLCD.print("Temperatura Maxima Zona 2", CENTER, 80);  // Título da tela de temperaturas
  char buffer[4];                                         // Buffer para armazenar a temperatura formatada
  sprintf(buffer, "%03d", tempMaxZ2);                     // Formata a velocidade para três dígitos
  myGLCD.setColor(255, 255, 0);                           // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                      // Exibe a temperatura
}

void verificarBotoesTempMaxZ2() {

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : (digitoAtual == 2) ? 3
                                                              : 1;  // Alterna entre centenas, dezenas e unidades
    telaTempMaxZ2();                                                // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      tempMaxZ2 = constrain(tempMaxZ2 + 100, 0, 999);  // Incrementa a centena
    } else if (digitoAtual == 2) {
      tempMaxZ2 = constrain(tempMaxZ2 + 10, 0, 999);  // Incrementa a dezena
    } else {
      tempMaxZ2 = constrain(tempMaxZ2 + 1, 0, 999);  // Incrementa a unidade
    }
    telaTempMaxZ2();  // Atualiza a tela
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      tempMaxZ2 = constrain(tempMaxZ2 - 100, 0, 999);  // Decrementa a centena
    } else if (digitoAtual == 2) {
      tempMaxZ2 = constrain(tempMaxZ2 - 10, 0, 999);  // Decrementa a dezena
    } else {
      tempMaxZ2 = constrain(tempMaxZ2 - 1, 0, 999);  // Decrementa a unidade
    }
    telaTempMaxZ2();  // Atualiza a tela
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 6;                        // Retorna para a tela de velocidade do motor fuso
    telaTempMinZ2();                 // Exibe a tela de velocidade do motor fuso
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 8;                      // Passa para a tela de configuração de temperaturas
    telaTempMinZ3();               // Exibe a tela de temperaturas
  }
}

void telaTempMinZ3() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor do texto
  myGLCD.print("Temperatura Minima Zona 3", CENTER, 80);  // Título da tela de temperaturas
  char buffer[4];                                         // Buffer para armazenar a temperatura formatada
  sprintf(buffer, "%03d", tempMinZ3);                     // Formata a velocidade para três dígitos
  myGLCD.setColor(255, 255, 0);                           // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                      // Exibe a temperatura
}

void verificarBotoesTempMinZ3() {

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : (digitoAtual == 2) ? 3
                                                              : 1;  // Alterna entre centenas, dezenas e unidades
    telaTempMinZ3();                                                // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      tempMinZ3 = constrain(tempMinZ3 + 100, 0, 999);  // Incrementa a centena
    } else if (digitoAtual == 2) {
      tempMinZ3 = constrain(tempMinZ3 + 10, 0, 999);  // Incrementa a dezena
    } else {
      tempMinZ3 = constrain(tempMinZ3 + 1, 0, 999);  // Incrementa a unidade
    }
    telaTempMinZ3();  // Atualiza a tela
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      tempMinZ3 = constrain(tempMinZ3 - 100, 0, 999);  // Decrementa a centena
    } else if (digitoAtual == 2) {
      tempMinZ3 = constrain(tempMinZ3 - 10, 0, 999);  // Decrementa a dezena
    } else {
      tempMinZ3 = constrain(tempMinZ3 - 1, 0, 999);  // Decrementa a unidade
    }
    telaTempMinZ3();  // Atualiza a tela.
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 7;                        // Retorna para a tela de velocidade do motor fuso
    telaTempMaxZ2();                 // Exibe a tela de velocidade do motor fuso
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 9;                      // Passa para a tela de configuração de temperaturas
    telaTempMaxZ3();               // Exibe a tela de temperaturas
  }
}

void telaTempMaxZ3() {
  myGLCD.clrScr();                                        // Limpa a tela
  myGLCD.setColor(250, 250, 250);                         // Define a cor do texto
  myGLCD.print("Temperatura Maxima Zona 3", CENTER, 80);  // Título da tela de temperaturas
  char buffer[4];                                         // Buffer para armazenar a temperatura formatada
  sprintf(buffer, "%03d", tempMaxZ3);                     // Formata a velocidade para três dígitos
  myGLCD.setColor(255, 255, 0);                           // Define a cor do texto
  myGLCD.print(buffer, CENTER, 150);                      // Exibe a temperatura
}

void verificarBotoesTempMaxZ3() {

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado
    digitoAtual = (digitoAtual == 1) ? 2 : (digitoAtual == 2) ? 3
                                                              : 1;  // Alterna entre centenas, dezenas e unidades
    telaTempMaxZ3();                                                // Atualiza a tela
  }
  if (digitalRead(incremento) == LOW) {  // Se o botão de incremento for pressionado
    if (digitoAtual == 1) {
      tempMaxZ3 = constrain(tempMaxZ3 + 100, 0, 999);  // Incrementa a centena
    } else if (digitoAtual == 2) {
      tempMaxZ3 = constrain(tempMaxZ3 + 10, 0, 999);  // Incrementa a dezena
    } else {
      tempMaxZ3 = constrain(tempMaxZ3 + 1, 0, 999);  // Incrementa a unidade
    }
    telaTempMaxZ3();  // Atualiza a tela.
  }
  if (digitalRead(decremento) == LOW) {  // Se o botão de decremento for pressionado
    if (digitoAtual == 1) {
      tempMaxZ3 = constrain(tempMaxZ3 - 100, 0, 999);  // Decrementa a centena
    } else if (digitoAtual == 2) {
      tempMaxZ3 = constrain(tempMaxZ3 - 10, 0, 999);  // Decrementa a dezena
    } else {
      tempMaxZ3 = constrain(tempMaxZ3 - 1, 0, 999);  // Decrementa a unidade
    }
    telaTempMaxZ3();  // Atualiza a tela.
  }
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 8;                        // Retorna para a tela de velocidade do motor fuso
    telaTempMinZ3();                 // Exibe a tela temperatura minima zona 3
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 10;                     // Passa para a tela de configuração de temperaturas
    telaResumo();                  // Exibe a tela de resumo
  }
}

void telaResumo() {
  if (!telaResumoDesenhada) {  // Só desenha a tela se ainda não foi desenhada
    myGLCD.clrScr();           // Limpa a tela

    // Título da tela de resumo
    myGLCD.setColor(0, 102, 204);  // Azul claro para o título
    myGLCD.setFont(BigFont);       // Fonte maior para o título
    myGLCD.print("Resumo das Configuracoes", CENTER, 20);

    // Exibe a velocidade do motor do fuso.
    myGLCD.setColor(255, 153, 0);                            // Laranja para o texto de velocidade
    myGLCD.setFont(SmallFont);                               // Fonte menor para o texto
    myGLCD.print("Vel. Fuso:", 10, 60);                      // Mostra a velocidade do fuso
    myGLCD.setColor(255, 255, 0);                            // Amarelo para o valor da velocidade do fuso
    char buffer[4];                                          // Buffer para armazenar a velocidade formatada
    sprintf(buffer, "%02d", velocidadeFuso);                 // Formata a velocidade para dois dígitos
    myGLCD.print(buffer, 200, 60);                           // Exibe a velocidade
    myGLCD.fillRect(10, 75, 10 + (velocidadeFuso * 2), 85);  // Barra de velocidade

    // Exibe a velocidade do motor de bobinamento
    myGLCD.setColor(255, 153, 0);                                     // Laranja para o texto
    myGLCD.print("Vel. Bob:", 10, 100);                               // Mostra a velocidade do bobinamento
    myGLCD.setColor(255, 255, 0);                                     // Amarelo para o valor da velocidade de bobinamento
    sprintf(buffer, "%02d", velocidadeBobinamento);                   // Formata a velocidade para dois dígitos
    myGLCD.print(buffer, 200, 100);                                   // Exibe a velocidade
    myGLCD.fillRect(10, 115, 10 + (velocidadeBobinamento * 2), 125);  // Barra de velocidade

    // Exibe as temperaturas da Zona 1
    myGLCD.setColor(102, 204, 255);                   // Azul claro para a Zona 1
    myGLCD.print("Z1 Min:", 10, 140);                 // Mostra a temperatura mínima da zona 1
    sprintf(buffer, "%03d", tempMinZ1);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 140);                   // Exibe a temperatura
    myGLCD.fillRect(10, 155, 10 + (tempMinZ1), 165);  // Barra de temperatura mínima

    myGLCD.print("Z1 Max:", 10, 170);                 // Mostra a temperatura máxima da zona 1
    sprintf(buffer, "%03d", tempMaxZ1);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 170);                   // Exibe a temperatura
    myGLCD.fillRect(10, 185, 10 + (tempMaxZ1), 195);  // Barra de temperatura máxima

    // Exibe as temperaturas da Zona 2
    myGLCD.setColor(255, 102, 102);                   // Vermelho claro para a Zona 2
    myGLCD.print("Z2 Min:", 10, 200);                 // Mostra a temperatura mínima da zona 2
    sprintf(buffer, "%03d", tempMinZ2);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 200);                   // Exibe a temperatura
    myGLCD.fillRect(10, 215, 10 + (tempMinZ2), 225);  // Barra de temperatura mínima

    myGLCD.print("Z2 Max:", 10, 230);                 // Mostra a temperatura máxima da zona 2
    sprintf(buffer, "%03d", tempMaxZ2);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 230);                   // Exibe a temperatura
    myGLCD.fillRect(10, 245, 10 + (tempMaxZ2), 255);  // Barra de temperatura máxima

    // Exibe as temperaturas da Zona 3
    myGLCD.setColor(153, 255, 153);                   // Verde claro para a Zona 3
    myGLCD.print("Z3 Min:", 10, 260);                 // Mostra a temperatura mínima da zona 3
    sprintf(buffer, "%03d", tempMinZ3);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 260);                   // Exibe a temperatura
    myGLCD.fillRect(10, 275, 10 + (tempMinZ3), 285);  // Barra de temperatura mínima

    myGLCD.print("Z3 Max:", 10, 290);                 // Mostra a temperatura máxima da zona 3
    sprintf(buffer, "%03d", tempMaxZ3);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 290);                   // Exibe a temperatura
    myGLCD.fillRect(10, 305, 10 + (tempMaxZ3), 315);  // Barra de temperatura máxima

    telaResumoDesenhada = true;  // Marca a tela como desenhada
  }

  verificarBotoesTelaResumo();  // Continua verificando os botões
}
void verificarBotoesTelaResumo() {
  myGLCD.setFont(BigFont);           // Define uma fonte grande
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 9;
    telaResumoDesenhada = false;  // Retorna para a tela de velocidade do motor fuso
    telaTempMaxZ3();              // Exibe a tela temperatura minima zona 3
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 11;                     // Passa para a tela de configuração de temperaturas
    telaPerguntaSalvarPrograma();  // Exibe a tela de resumo
  }
}

void telaPerguntaSalvarPrograma() {
  myGLCD.clrScr();                               // Limpa a tela
  myGLCD.setColor(250, 250, 250);                // Define a cor do texto
  myGLCD.print("Salvar Programa?", CENTER, 80);  // Pergunta sobre o modo automático
  myGLCD.setColor(0, 128, 0);                    // Define a cor do texto "Não"
  myGLCD.print("Sim", 330, 250);                 // Opção "Sim" e posição na tela
  myGLCD.setColor(255, 40, 0);                   // Define a cor do texto "Sim"
  myGLCD.print("Nao", 100, 250);                 // Opção "Não" e posição na tela
}

void verificarBotoesSalvarPrograma() {
  if (digitalRead(before) == LOW) {  // Se o botão de voltar for pressionado
    page = 15;
    telaIniciarProcesso();  // Tela de iniciar o processo
  }
  if (digitalRead(next) == LOW) {  // Se o botão de avançar for pressionado
    page = 12;                     // Passa para a tela de velocidade do motor fuso
    telaSelecaoMemoria();          // Exibe a tela de velocidade do motor fuso
  }
}
void telaSelecaoMemoria() {
  myGLCD.clrScr();                 // Limpa a tela
  myGLCD.setColor(250, 250, 250);  // Define a cor do texto

  // Título da tela
  myGLCD.print("Selecione Memoria", CENTER, 20);

  // Exibe as posições de memória
  for (int i = 0; i < totalMemorias; i++) {
    if (i == posicaoMemoriaSelecionada) {
      myGLCD.setColor(0, 255, 0);  // Cor verde para a memória selecionada
    } else {
      myGLCD.setColor(255, 255, 255);  // Cor branca para as outras memórias
    }
    char buffer[15];
    sprintf(buffer, "Programa %02d", i + 1);    // Formata a string para exibir "Memoria 01", "Memoria 02", etc
    myGLCD.print(buffer, CENTER, 60 + i * 20);  // Exibe cada posição de memória
  }

  verificarBotoesTelaSelecaoMemoria();  // Verifica os botões na tela de seleção de memória
}

void verificarBotoesTelaSelecaoMemoria() {
  if (digitalRead(incremento) == LOW) {  // Se o botão "incremento" for pressionado
    posicaoMemoriaSelecionada++;         // Move para a próxima posição de memória
    if (posicaoMemoriaSelecionada >= totalMemorias) {
      posicaoMemoriaSelecionada = 0;  // Volta para a primeira memória se ultrapassar
    }
    telaSelecaoMemoria();  // Atualiza a tela
    delay(200);            // Debounce para evitar múltiplas leituras rápidas
  }

  if (digitalRead(decremento) == LOW) {  // Se o botão "decremento" for pressionado
    posicaoMemoriaSelecionada--;         // Move para a posição de memória anterior
    if (posicaoMemoriaSelecionada < 0) {
      posicaoMemoriaSelecionada = totalMemorias - 1;  // Vai para a última memória se for menor que zero
    }
    telaSelecaoMemoria();  // Atualiza a tela
    delay(200);            // Debounce para evitar múltiplas leituras rápidas
  }

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado

    telaPerguntaConfirmarGravacao();  // Chama a tela para confirmar a gravação
    page = 13;
  }
}

void telaPerguntaConfirmarGravacao() {
  myGLCD.clrScr();                                  // Limpa a tela
  myGLCD.setColor(250, 250, 250);                   // Define a cor do texto
  myGLCD.print("Confirmar Gravacao?", CENTER, 80);  // Pergunta sobre a confirmação da gravação

  myGLCD.setColor(0, 128, 0);     // Define a cor do texto "Sim"
  myGLCD.print("Sim", 330, 250);  // Opção "Sim" e posição na tela

  myGLCD.setColor(255, 40, 0);    // Define a cor do texto "Não"
  myGLCD.print("Nao", 100, 250);  // Opção "Não" e posição na tela
}

void verificarBotoesTelaPerguntaConfirmarGravacao() {
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 12;                       // Aqui você pode voltar para a tela de seleção de memória se desejar
    telaSelecaoMemoria();            // Volta para a tela de seleção de memória
  }

  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    gravarNaEEPROM();
  }
}

void gravarNaEEPROM() {
  EEPROM.write(0, velocidadeFuso);                  // Grava a velocidade do fuso (2 bytes: endereço 0 e 1)
  EEPROM.write(2, velocidadeBobinamento);           // Grava a velocidade do bobinamento (2 bytes: endereço 2 e 3)
  EEPROM.write(4, tempMinZ1);                       // Temperatura mínima Z1 (endereço 4 e 5)
  EEPROM.write(7, tempMaxZ1);                       // Temperatura máxima Z1 (endereço 6 e 7)
  EEPROM.write(10, tempMinZ2);                      // Temperatura mínima Z2 (endereço 8 e 9)
  EEPROM.write(13, tempMaxZ2);                      // Temperatura máxima Z2 (endereço 10 e 11)
  EEPROM.write(16, tempMinZ3);                      // Temperatura mínima Z3 (endereço 12 e 13)
  EEPROM.write(19, tempMaxZ3);                      // Temperatura máxima Z3 (endereço 14 e 15)
  myGLCD.print("Dados gravados!", CENTER, CENTER);  // Exibe a mensagem de confirmação no centro da tela
  telaConfirmacao();
}

void telaConfirmacao() {
  myGLCD.clrScr();                                  // Limpa a tela
  myGLCD.setBackColor(0, 0, 0);                     // Define o fundo da tela como preto
  myGLCD.setColor(0, 255, 0);                       // Define a cor do texto como verde
  myGLCD.print("Dados gravados!", CENTER, CENTER);  // Exibe a mensagem de confirmação no centro da tela
  delay(2000);                                      // Aguarda por 2 segundos para que a mensagem seja visível
  telaPerguntaModoAutomatico();
  page = 1;
  telaResumoDesenhada = false;
}

void telaIniciarProcesso() {
  myGLCD.clrScr();                                // Limpa a tela
  myGLCD.setColor(250, 250, 250);                 // Define a cor do texto
  myGLCD.print("Iniciar Processo?", CENTER, 80);  // Pergunta sobre a confirmação

  myGLCD.setColor(0, 128, 0);     // Define a cor do texto "Sim"
  myGLCD.print("Sim", 330, 250);  // Opção "Sim" e posição na tela

  myGLCD.setColor(255, 40, 0);    // Define a cor do texto "Não"
  myGLCD.print("Nao", 100, 250);  // Opção "Não" e posição na tela
}

void verificarBotoesTelaIniciarProcesso() {
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 1;                        // Aqui você pode voltar para a tela de seleção de memória se desejar
    telaPerguntaModoAutomatico();    // Volta para a tela de seleção de memória
  }

  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    processoExtrusao();
    page = 16;
  }
}

// Função para configurar o motor do fuso com base em uma velocidade específica
void configurarMotorFuso(int velocidadeFuso) {

  // Verifica se a velocidade do fuso está dentro do intervalo válido (0 a 255)
  if (velocidadeFuso >= 0 && velocidadeFuso <= 255) {
    // Aplica o valor de PWM no pino do motor do fuso
    // O valor PWM aplicado é a 'velocidadeFuso' somada a 150 (ajuste extra)
    analogWrite(pinPWMfuso, velocidadeFuso + 150);
  }
}

// Função para configurar o motor de bobinamento com base em uma velocidade específica
void configurarMotorBobinamento(int velocidadeBobinamento) {

  // Lê o valor do potenciômetro conectado ao pino 'pinPotenciometro'
  int valorPotenciometro = analogRead(pinPotenciometro);

  // Mapeia o valor lido do potenciômetro (de 0 a 1023) para a faixa de 0 a 99
  int ajuste = map(valorPotenciometro, 0, 1023, 0, 99);

  // Verifica se a velocidade de bobinamento está dentro do intervalo válido (0 a 255)
  if (velocidadeBobinamento >= 0 && velocidadeBobinamento <= 255) {
    // Aplica o valor de PWM no pino do motor de bobinamento
    // O valor PWM aplicado é a 'velocidadeBobinamento' somada ao ajuste e 150 (ajuste extra)
    analogWrite(pinPWMbobina, velocidadeBobinamento + ajuste + 150);

    // Atualiza uma variável global para armazenar o valor de velocidade com o ajuste aplicado
    velocidadeBobinaAtualizado = velocidadeBobinamento + ajuste;
  }
}

void telaSelecaoMemoriaAutomatico() {
  myGLCD.clrScr();                                 // Limpa a tela
  myGLCD.setColor(250, 250, 250);                  // Define a cor do texto
  myGLCD.print("Selecione Programa", CENTER, 20);  // Título da tela

  // Exibe as posições de memória
  for (int i = 0; i < totalMemorias; i++) {
    if (i == posicaoMemoriaSelecionada) {
      myGLCD.setColor(0, 255, 0);  // Cor verde para a memória selecionada
    } else {
      myGLCD.setColor(255, 255, 255);  // Cor branca para as outras memórias
    }
    char buffer[15];
    sprintf(buffer, "Programa %02d", i + 1);    // Formata a string para exibir "Memoria 01", "Memoria 02", etc
    myGLCD.print(buffer, CENTER, 60 + i * 20);  // Exibe cada posição de memória
  }
}

void verificarBotoesTelaSelecaoMemoriaAutomatico() {
  if (digitalRead(incremento) == LOW) {  // Se o botão "incremento" for pressionado
    posicaoMemoriaSelecionada--;         // Move para a próxima posição de memória
    if (posicaoMemoriaSelecionada >= totalMemorias) {
      posicaoMemoriaSelecionada = 0;  // Volta para a primeira memória se ultrapassar
    }
    telaSelecaoMemoria();  // Atualiza a tela
    delay(200);            // Debounce para evitar múltiplas leituras rápidas
  }

  if (digitalRead(decremento) == LOW) {  // Se o botão "decremento" for pressionado
    posicaoMemoriaSelecionada++;         // Move para a posição de memória anterior
    if (posicaoMemoriaSelecionada < 0) {
      posicaoMemoriaSelecionada = totalMemorias - 1;  // Vai para a última memória se for menor que zero
    }
    telaSelecaoMemoria();  // Atualiza a tela
    delay(200);            // Debounce para evitar múltiplas leituras rápidas
  }

  if (digitalRead(select) == LOW) {  // Se o botão "selecionar" for pressionado
    lerDaEEPROM();
    telaPerguntaConfirmarMemoriaAutomatico();  // Chama a tela para confirmar a gravação
    page = 18;
  }
}

void telaPerguntaConfirmarMemoriaAutomatico() {
  myGLCD.clrScr();                                  // Limpa a tela
  myGLCD.setColor(250, 250, 250);                   // Define a cor do texto
  myGLCD.print("Confirmar Programa?", CENTER, 80);  // Pergunta sobre a confirmação

  myGLCD.setColor(0, 128, 0);     // Define a cor do texto "Sim"
  myGLCD.print("Sim", 330, 250);  // Opção "Sim" e posição na tela

  myGLCD.setColor(255, 40, 0);    // Define a cor do texto "Não"
  myGLCD.print("Nao", 100, 250);  // Opção "Não" e posição na tela
}

void verificarBotoesTelaPerguntaConfirmarMemoriaAutomatico() {
  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 1;                        // Aqui você pode voltar para a tela de seleção de memória se desejar
    telaPerguntaModoAutomatico();    // Volta para a tela de seleção de memória
  }

  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 19;
    telaResumoAutomatico();
  }
}

void telaResumoAutomatico() {
  if (!telaResumoAutomaticoDesenhada) {  // Só desenha a tela se ainda não foi desenhada
    myGLCD.clrScr();                     // Limpa a tela

    // Título da tela de resumo
    myGLCD.setColor(0, 102, 204);                          // Azul claro para o título
    myGLCD.setFont(BigFont);                               // Fonte maior para o título
    myGLCD.print("Resumo das Configuracoes", CENTER, 20);  // Mostra o resumo

    // Exibe a velocidade do motor do fuso.
    myGLCD.setColor(255, 153, 0);                            // Laranja para o texto de velocidade
    myGLCD.setFont(SmallFont);                               // Fonte menor para o texto
    myGLCD.print("Vel. Fuso:", 10, 60);                      // Mostra a velocidade do fuso
    myGLCD.setColor(255, 255, 0);                            // Amarelo para o valor da velocidade do fuso
    char buffer[4];                                          // Buffer para armazenar a velocidade formatada
    sprintf(buffer, "%02d", velocidadeFuso);                 // Formata a velocidade para dois dígitos
    myGLCD.print(buffer, 200, 60);                           // Exibe a velocidade
    myGLCD.fillRect(10, 75, 10 + (velocidadeFuso * 2), 85);  // Barra de velocidade

    // Exibe a velocidade do motor de bobinamento
    myGLCD.setColor(255, 153, 0);                                     // Laranja para o texto
    myGLCD.print("Vel. Bob:", 10, 100);                               // Mostra a velocidade do bobinamento
    myGLCD.setColor(255, 255, 0);                                     // Amarelo para o valor da velocidade de bobinamento
    sprintf(buffer, "%02d", velocidadeBobinamento);                   // Formata a velocidade para dois dígitos
    myGLCD.print(buffer, 200, 100);                                   // Exibe a velocidade
    myGLCD.fillRect(10, 115, 10 + (velocidadeBobinamento * 2), 125);  // Barra de velocidade

    // Exibe as temperaturas da Zona 1
    myGLCD.setColor(102, 204, 255);                   // Azul claro para a Zona 1
    myGLCD.print("Z1 Min:", 10, 140);                 // Mostra a temperatura mínima da zona 1
    sprintf(buffer, "%03d", tempMinZ1);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 140);                   // Exibe a temperatura
    myGLCD.fillRect(10, 155, 10 + (tempMinZ1), 165);  // Barra de temperatura mínima

    myGLCD.print("Z1 Max:", 10, 170);                 // Mostra a temperatura máxima da zona 1
    sprintf(buffer, "%03d", tempMaxZ1);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 170);                   // Exibe a temperatura
    myGLCD.fillRect(10, 185, 10 + (tempMaxZ1), 195);  // Barra de temperatura máxima

    // Exibe as temperaturas da Zona 2
    myGLCD.setColor(255, 102, 102);                   // Vermelho claro para a Zona 2
    myGLCD.print("Z2 Min:", 10, 200);                 // Mostra a temperatura mínima da zona 2
    sprintf(buffer, "%03d", tempMinZ2);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 200);                   // Exibe a temperatura
    myGLCD.fillRect(10, 215, 10 + (tempMinZ2), 225);  // Barra de temperatura mínima

    myGLCD.print("Z2 Max:", 10, 230);                 // Mostra a temperatura máxima da zona 2
    sprintf(buffer, "%03d", tempMaxZ2);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 230);                   // Exibe a temperatura
    myGLCD.fillRect(10, 245, 10 + (tempMaxZ2), 255);  // Barra de temperatura máxima

    // Exibe as temperaturas da Zona 3
    myGLCD.setColor(153, 255, 153);                   // Verde claro para a Zona 3
    myGLCD.print("Z3 Min:", 10, 260);                 // Mostra a temperatura mínima da zona 3
    sprintf(buffer, "%03d", tempMinZ3);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 260);                   // Exibe a temperatura
    myGLCD.fillRect(10, 275, 10 + (tempMinZ3), 285);  // Barra de temperatura mínima

    myGLCD.print("Z3 Max:", 10, 290);                 // Mostra a temperatura máxima da zona 3
    sprintf(buffer, "%03d", tempMaxZ3);               // Formata a temperatura para três dígitos
    myGLCD.print(buffer, 200, 290);                   // Exibe a temperatura
    myGLCD.fillRect(10, 305, 10 + (tempMaxZ3), 315);  // Barra de temperatura máxima

    telaResumoAutomaticoDesenhada = true;  // Marca a tela como desenhada
  }

  verificarBotoesTelaResumoAutomatico();  // Continua verificando os botões
}

void verificarBotoesTelaResumoAutomatico() {
  myGLCD.setFont(BigFont);  // Define uma fonte grande

  if (digitalRead(before) == LOW) {  // Se o botão "voltar" for pressionado
    page = 1;                        // Retorna para a tela de velocidade do motor fuso
    telaPerguntaModoAutomatico();    // Exibe a tela temperatura minima zona 3
  }
  if (digitalRead(next) == LOW) {  // Se o botão "avançar" for pressionado
    page = 15;                     // Passa para a tela de configuração de temperaturas
    telaIniciarProcesso();         // Exibe a tela iniciar processo
  }
}

void lerDaEEPROM() {

  velocidadeFuso = EEPROM.read(0);         // Lê a velocidade do fuso
  velocidadeBobinamento = EEPROM.read(2);  // Lê a velocidade do bobinamento
  tempMinZ1 = EEPROM.read(4);              // Lê as temperatura minima da Zona 1
  tempMaxZ1 = EEPROM.read(7);              // Lê as temperatura maxima da Zona 1
  tempMinZ2 = EEPROM.read(10);             // Lê as temperatura minima da Zona 2
  tempMaxZ2 = EEPROM.read(13);             // Lê as temperatura maxima da Zona 2
  tempMinZ3 = EEPROM.read(16);             // Lê as temperatura minima da Zona 3
  tempMaxZ3 = EEPROM.read(19);             // Lê as temperatura maxima da Zona 3
}

// Função que exibe as informações na tela de extrusão
void telaExtrusaoEstatica() {
  // Limpa a tela e desenha elementos fixos uma vez
  myGLCD.clrScr();

  // Cabeçalho
  myGLCD.setColor(255, 255, 0);              // Define a cor do texto
  myGLCD.setFont(BigFont);                   // Define uma fonte grande
  myGLCD.print("AQUECENDO...", CENTER, 20);  // Mostra na tela que está aquecendo

  // Divisória para zonas de temperatura
  myGLCD.setColor(255, 165, 0);        // Cor laranja para as divisórias
  myGLCD.drawRect(10, 80, 475, 150);   // Retângulo ao redor das zonas de temperatura
  myGLCD.drawLine(160, 80, 160, 150);  // Linha entre Z1 e Z2
  myGLCD.drawLine(320, 80, 320, 150);  // Linha entre Z2 e Z3

  // Temperatura Zona 1
  myGLCD.setColor(0, 0, 255);        // Azul para o texto das zonas
  myGLCD.print("Temp. Z1", 20, 85);  // Mostra na tela temperatura atual da zona 1

  // Temperatura Zona 2
  myGLCD.setColor(0, 0, 255);         // Define a cor do texto
  myGLCD.print("Temp. Z2", 180, 85);  // Mostra na tela temperatura atual da zona 2

  // Temperatura Zona 3
  myGLCD.setColor(0, 0, 255);         // Define a cor do texto
  myGLCD.print("Temp. Z3", 340, 85);  // Mostra na tela temperatura atual da zona 3

  // Divisória para velocidades dos motores
  myGLCD.setColor(255, 165, 0);         // Define a cor do texto
  myGLCD.drawRect(10, 200, 475, 270);   // Retângulo ao redor das velocidades dos motores
  myGLCD.drawLine(240, 200, 240, 270);  // Linha entre Fuso e Bobinamento

  // Velocidade do Fuso
  myGLCD.setColor(0, 0, 255);       // Azul para o texto das velocidades
  myGLCD.print("V.Fuso", 20, 205);  // Mostra na tela velocidade do fuso

  // Velocidade do Bobinamento
  myGLCD.setColor(0, 0, 255);       // Define a cor do texto
  myGLCD.print("V.Bob", 260, 205);  // Mostra na tela velocidade do bobinamento
}

void atualizarValoresTelaExtrusao() {
  // Temperatura Zona 1
  myGLCD.setColor(255, 0, 0);                        // Cor vermelha para a barra de temperatura
  myGLCD.fillRect(20, 110, 20 + (tempZ1 * 2), 130);  // Barra de temperatura ajustada pelo valor de tempZ1
  myGLCD.setColor(255, 255, 255);                    // Branco para o valor da temperatura
  myGLCD.print(String(tempZ1) + " C", 20, 140);      // Mostra na tela temperatura atual da zona 1

  // Temperatura Zona 2
  myGLCD.setColor(255, 0, 0);                          // Define a cor do texto
  myGLCD.fillRect(180, 110, 180 + (tempZ2 * 2), 130);  // Barra de temperatura ajustada pelo valor de tempZ2
  myGLCD.setColor(255, 255, 255);                      // Define a cor do texto
  myGLCD.print(String(tempZ2) + " C", 180, 140);       // Mostra na tela temperatura atual da zona 2

  // Temperatura Zona 3
  myGLCD.setColor(255, 0, 0);                          // Define a cor do texto
  myGLCD.fillRect(340, 110, 340 + (tempZ3 * 2), 130);  // Barra de temperatura ajustada pelo valor de tempZ3
  myGLCD.setColor(255, 255, 255);                      // Define a cor do texto
  myGLCD.print(String(tempZ3) + " C", 340, 140);       // Mostra na tela temperatura atual da zona 3

  // Velocidade do Fuso
  myGLCD.setColor(0, 255, 0);                                // Verde para a barra de velocidade
  myGLCD.fillRect(20, 230, 20 + (velocidadeFuso * 2), 250);  // Barra de velocidade ajustada pelo valor de velocidadeFuso
  myGLCD.setColor(255, 255, 255);                            // Branco para o valor de velocidade
  myGLCD.print(String(velocidadeFuso), 20, 255);             // Mostra na tela velocidade do fuso

  // Velocidade do Bobinamento
  myGLCD.setColor(0, 255, 0);                                              // Define a cor do texto
  myGLCD.fillRect(260, 230, 260 + (velocidadeBobinaAtualizado * 2), 250);  // Barra de velocidade ajustada pelo valor de velocidadeBobinaAtualizado
  myGLCD.setColor(255, 255, 255);                                          // Define a cor do texto
  myGLCD.print(String(velocidadeBobinaAtualizado), 260, 255);              // Mostra na tela velocidade do bobinamento
}

// Função responsável por controlar o processo de extrusão dentro de um loop contínuo
void processoExtrusao() {

  telaExtrusaoEstatica();  // Exibe a tela estática apenas uma vez
  do {
    if (startTimer == 0) {
      startTimer = millis();  // Inicia o temporizador
    }
    if (millis() - startTimer >= endTimer) {
      timer = true;  // Verifica se o tempo estabelecido foi atingido
    }

    //Cálculo dos Fatores de segurança das Zonas de Temperatura
    FSZ1 = tempMaxZ1 + (tempMaxZ1 * 0.25);  // Temperatura max + % de segurança da zona 1
    FSZ2 = tempMaxZ2 + (tempMaxZ2 * 0.25);  // Temperatura max + % de segurança da zona 2
    FSZ3 = tempMaxZ3 + (tempMaxZ3 * 0.25);  // Temperatura max + % de segurança da zona 3

    tempZ1 = int(thermocouple1.readCelsius() - 2);  // Lê a temperatura do Sensor 1
    tempZ2 = int(thermocouple2.readCelsius() - 3);  // Lê a temperatura do Sensor 2
    tempZ3 = int(thermocouple3.readCelsius() + 5);  // Lê a temperatura do Sensor 3
    atualizarValoresTelaExtrusao();                 // Atualiza apenas os valores

    if ((tempZ1 >= tempMinZ1) && (tempZ2 >= tempMinZ2) && (tempZ3 >= tempMinZ3)) {
      timer = true;
      digitalWrite(RELAY8, LOW);                          // Inicia o motor do fuso
      configurarMotorFuso(velocidadeFuso);                // Inicia o motor do fuso com base na variável 'velocidadeFuso'
      configurarMotorBobinamento(velocidadeBobinamento);  // Inicia o motor de bobinamento com base na variável 'velocidadeBobinamento'
      //digitalWrite(RELAY5, LOW);                          // Liga Refrigeração Z4 Saída Filamento
    } else {
      // Se alguma zona não estiver dentro dos limites
      if (timer == false) {
        if (millis() - startTimer > endTimer) {
          telaFalhaAquecimento();  // Alerta de aquecimento
        }
      }
      if (tempZ1 < tempMinZ1 || tempZ1 > tempMaxZ1) {
        if (tempZ1 <= tempMinZ1) {
          digitalWrite(pinSSR1, LOW);  // Liga Resistência 1
        }
        if (tempZ1 >= tempMaxZ1) {
          digitalWrite(pinSSR1, HIGH);  // Desliga Resistência 1
          digitalWrite(RELAY2, LOW);    // Liga Refrigeração Z1

          if (tempZ1 >= FSZ1) {
            telaFSMaxZ1();
          }
        } else {
          digitalWrite(RELAY2, HIGH);  // Desliga Refrigeração Z1
        }
      }

      if (tempZ2 < tempMinZ2 || tempZ2 > tempMaxZ2) {
        if (tempZ2 <= tempMinZ2) {
          digitalWrite(pinSSR2, LOW);  // Liga Resistência 2
        }
        if (tempZ2 >= tempMaxZ2) {
          digitalWrite(pinSSR2, HIGH);  // Desliga Resistência 1
          digitalWrite(RELAY3, LOW);    // Liga Refrigeração Z3
          if (tempZ2 >= FSZ2) {
            telaFSMaxZ2();
          }
        } else {
          digitalWrite(RELAY3, HIGH);  // Desliga Refrigeração Z3
        }
      }

      if (tempZ3 < tempMinZ3 || tempZ3 > tempMaxZ3) {
        if (tempZ3 >= tempMaxZ3) {
          digitalWrite(RELAY4, LOW);  // Liga Refrigeração Z3
        } else {
          digitalWrite(RELAY4, HIGH);  // Desliga Refrigeração Z3
        }
      }

      if (tempZ3 <= tempMinZ3) {
        if (timer == false) {
          if (millis() - startTimer > endTimer) {
            telaFalhaAquecimento();
          }
        }
      } else {
        if (tempZ3 >= FSZ3) {
          telaFSMaxZ3();
        }
        if (timer == false) {
          if (millis() - startTimer > endTimer) {
            telaFalhaAquecimento();
          }
        }
      }
    }
    coletaDados();
  } while (true);  // Loop contínuo sem condição de saída

  delay(1000);  // Aguarda 1 segundo antes de atualizar novamente
}

void telaFSMaxZ1() {
  myGLCD.clrScr();                  // Limpa a tela
  myGLCD.setBackColor(255, 0, 0);   // Cor de fundo vermelha
  myGLCD.setColor(255, 0, 0);       // Cor do texto
  myGLCD.fillRect(0, 0, 480, 320);  // Preenche a tela inteira com vermelho

  // Desenha um triângulo amarelo (sinal de alerta) usando linhas
  myGLCD.setColor(255, 255, 0);         // Cor amarela para o triângulo
  myGLCD.drawLine(240, 50, 200, 150);   // Lado esquerdo do triângulo
  myGLCD.drawLine(200, 150, 280, 150);  // Base do triângulo
  myGLCD.drawLine(280, 150, 240, 50);   // Lado direito do triângulo

  // Círculo vermelho dentro do triângulo para simbolizar calor
  myGLCD.setColor(255, 0, 0);       // Cor vermelha para o círculo
  myGLCD.fillCircle(240, 120, 20);  // Círculo no centro do triângulo

  // Texto do aviso.
  myGLCD.setColor(255, 255, 255);                        // Define a cor do texto para branco
  myGLCD.setFont(BigFont);                               // Define uma fonte grande
  myGLCD.print("EXCESSO DE TEMPERATURA!", CENTER, 240);  // Mensagem principal
  myGLCD.print("ZONA 1", CENTER, 270);                   // Mensagem secundária
}

void telaFSMaxZ2() {
  myGLCD.clrScr();                  // Limpa a tela
  myGLCD.setBackColor(255, 0, 0);   // Cor de fundo vermelha
  myGLCD.setColor(255, 0, 0);       // Cor do texto
  myGLCD.fillRect(0, 0, 480, 320);  // Preenche a tela inteira com vermelho

  // Desenha um triângulo amarelo (sinal de alerta) usando linhas
  myGLCD.setColor(255, 255, 0);         // Cor amarela para o triângulo
  myGLCD.drawLine(240, 50, 200, 150);   // Lado esquerdo do triângulo
  myGLCD.drawLine(200, 150, 280, 150);  // Base do triângulo
  myGLCD.drawLine(280, 150, 240, 50);   // Lado direito do triângulo

  // Círculo vermelho dentro do triângulo para simbolizar calor
  myGLCD.setColor(255, 0, 0);       // Cor vermelha para o círculo
  myGLCD.fillCircle(240, 120, 20);  // Círculo no centro do triângulo

  // Texto do aviso.
  myGLCD.setColor(255, 255, 255);                        // Define a cor do texto para branco
  myGLCD.setFont(BigFont);                               // Define uma fonte grande
  myGLCD.print("EXCESSO DE TEMPERATURA!", CENTER, 240);  // Mensagem principal
  myGLCD.print("ZONA 2", CENTER, 270);                   // Mensagem secundária
}

void telaFSMaxZ3() {
  myGLCD.clrScr();                  // Limpa a tela
  myGLCD.setBackColor(255, 0, 0);   // Cor de fundo vermelha
  myGLCD.setColor(255, 0, 0);       // Cor do texto
  myGLCD.fillRect(0, 0, 480, 320);  // Preenche a tela inteira com vermelho

  // Desenha um triângulo amarelo (sinal de alerta) usando linhas
  myGLCD.setColor(255, 255, 0);         // Cor amarela para o triângulo
  myGLCD.drawLine(240, 50, 200, 150);   // Lado esquerdo do triângulo
  myGLCD.drawLine(200, 150, 280, 150);  // Base do triângulo
  myGLCD.drawLine(280, 150, 240, 50);   // Lado direito do triângulo

  // Círculo vermelho dentro do triângulo para simbolizar calor
  myGLCD.setColor(255, 0, 0);       // Cor vermelha para o círculo
  myGLCD.fillCircle(240, 120, 20);  // Círculo no centro do triângulo

  // Texto do aviso.
  myGLCD.setColor(255, 255, 255);                        // Define a cor do texto para branco
  myGLCD.setFont(BigFont);                               // Define uma fonte grande
  myGLCD.print("EXCESSO DE TEMPERATURA!", CENTER, 240);  // Mensagem principal
  myGLCD.print("ZONA 3", CENTER, 270);                   // Mensagem secundária
}

void telaFalhaAquecimento() {
  myGLCD.clrScr();                  // Limpa a tela
  myGLCD.setBackColor(255, 0, 0);   // Cor de fundo vermelha
  myGLCD.setColor(255, 0, 0);       // Cor do texto
  myGLCD.fillRect(0, 0, 480, 320);  // Preenche a tela inteira com vermelho

  // Desenha um triângulo amarelo (sinal de alerta) usando linhas
  myGLCD.setColor(255, 255, 0);         // Cor amarela para o triângulo
  myGLCD.drawLine(240, 50, 200, 150);   // Lado esquerdo do triângulo
  myGLCD.drawLine(200, 150, 280, 150);  // Base do triângulo
  myGLCD.drawLine(280, 150, 240, 50);   // Lado direito do triângulo

  // Círculo vermelho dentro do triângulo para simbolizar calor
  myGLCD.setColor(255, 0, 0);       // Cor vermelha para o círculo
  myGLCD.fillCircle(240, 120, 20);  // Círculo no centro do triângulo

  // Texto do aviso.
  myGLCD.setColor(255, 255, 255);                      // Define a cor do texto para branco
  myGLCD.setFont(BigFont);                             // Define uma fonte grande
  myGLCD.print("FALHA NO AQUECIMENTO!", CENTER, 240);  // Mensagem principal
  myGLCD.print("ZONA 1-2-3", CENTER, 270);             // Mensagem secundária
}

bool confirmarSaidaExtrusao() {
  telaConfirmacaoSaida();  // Exibe a tela de confirmação de saída
  while (true) {
    // Verifica se o botão "avançar" foi pressionado para confirmar a saída (Sim)
    if (digitalRead(next) == LOW) {
      delay(200);   // Debounce
      break;        // Sai do loop do...while
      return true;  // Retorna true, indicando que o usuário escolheu encerrar
      page = 20;
      telaEncerramento();
    }
    // Verifica se o botão "voltar" foi pressionado para cancelar a saída (Não)
    if (digitalRead(before) == LOW) {
      delay(200);       // Debounce
      myGLCD.clrScr();  // Limpa a tela
      telaExtrusaoEstatica();
      return false;  // Retorna false, indicando que o usuário escolheu continuar
    }
  }
}

// Função que exibe a tela de confirmação de saída
void telaConfirmacaoSaida() {
  myGLCD.clrScr();                              // Limpa a tela
  myGLCD.setColor(250, 250, 250);               // Define a cor do texto
  myGLCD.print("Parar Extrusao?", CENTER, 80);  // Pergunta sobre a confirmação

  myGLCD.setColor(0, 128, 0);     // Define a cor do texto "Sim"
  myGLCD.print("Sim", 330, 250);  // Opção "Sim" e posição na tela

  myGLCD.setColor(255, 40, 0);    // Define a cor do texto "Não"
  myGLCD.print("Nao", 100, 250);  // Opção "Não" e posição na tela
}

void telaEncerramento() {
  // Limpa a tela e desenha elementos fixos uma vez
  myGLCD.clrScr();

  // Cabeçalho
  myGLCD.setColor(255, 255, 0);  // Define a cor do texto
  myGLCD.setFont(BigFont);       // Fonte maior para o título
  myGLCD.print("Finalizando Processo...)", CENTER, 20);

  // Divisória para zonas de temperatura
  myGLCD.setColor(255, 165, 0);        // Cor laranja para as divisórias
  myGLCD.drawRect(10, 80, 475, 150);   // Retângulo ao redor das zonas de temperatura
  myGLCD.drawLine(160, 80, 160, 150);  // Linha entre Z1 e Z2
  myGLCD.drawLine(320, 80, 320, 150);  // Linha entre Z2 e Z3

  // Temperatura Zona 1
  myGLCD.setColor(0, 0, 255);        // Azul para o texto das zonas
  myGLCD.print("Temp. Z1", 20, 85);  // Mostra na tela temperatura atual da zona 1

  // Temperatura Zona 2
  myGLCD.setColor(0, 0, 255);         // Define a cor do texto
  myGLCD.print("Temp. Z2", 180, 85);  // Mostra na tela temperatura atual da zona 2

  // Temperatura Zona 3
  myGLCD.setColor(0, 0, 255);         // Define a cor do texto
  myGLCD.print("Temp. Z3", 340, 85);  // Mostra na tela temperatura atual da zona 3

  // Divisória para velocidades dos motores
  myGLCD.setColor(255, 165, 0);         // Define a cor do texto
  myGLCD.drawRect(10, 200, 475, 270);   // Retângulo ao redor das velocidades dos motores
  myGLCD.drawLine(240, 200, 240, 270);  // Linha entre Fuso e Bobinamento

  // Velocidade do Fuso
  myGLCD.setColor(0, 0, 255);       // Azul para o texto das velocidades
  myGLCD.print("V.Fuso", 20, 205);  // Mostra na tela velocidade do fuso

  // Velocidade do Bobinamento
  myGLCD.setColor(0, 0, 255);       // Define a cor do texto
  myGLCD.print("V.Bob", 260, 205);  // Mostra na tela velocidade do bobinamento
}

// Função para encerrar o processo de extrusão
void encerrarProcesso() {
  myGLCD.clrScr();  // Limpa a tela

  // Cabeçalho
  myGLCD.setColor(255, 255, 0);             // Define a cor do texto
  myGLCD.setFont(BigFont);                  // Fonte maior para o título
  myGLCD.print("ENCERRADO!)", CENTER, 20);  // Mostra a mensagem de encarredo
  delay(4000);                              // Atraso de 4 segundos
  page = 1;
  telaPerguntaModoAutomatico();
}
