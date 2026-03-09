#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>




// MOTOR A
#define IN1_1_A 5
#define IN2_1_A 6
#define PWM_1_A 4

#define IN1_2_A 3
#define IN2_2_A 46
#define PWM_2_A 7

// MOTOR B
#define IN1_1_B 11
#define IN2_1_B 12
#define PWM_1_B 10

#define IN1_2_B 13
#define IN2_2_B 14
#define PWM_2_B 47

// ----------- CANAIS PWM ------------
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

#define PWM_FREQ 20000
#define PWM_RES 8

// ----------- SERIAL CABECA-MUSCULO ---------
#define RX_CABECA 17
#define TX_CABECA 18

// ----------- I2C OLED DISPLAY ---------
#define SDA_PIN 8
#define SCL_PIN 9
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C  // endereço padrão do SSD1306

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Definições para menu
#define BYTE_INICIA 0xAA
#define BYTE_PARA 0x55

struct MenuData {
  int itemSelecionado;
  int estadoMenu;  // 0 = MENU, 1 = CALIBRACAO, 2 = INICIAR
  int16_t anguloBola;
  int16_t anguloLinha;
  bool linhaDetectada;
  bool IRdetectado;
  int subMenuCal;           // 0 = SUBMENU_PRINCIPAL, 1 = SUBMENU_GOL, 2 = SUBMENU_BUSSOLA
  int itemSubMenuCal;       // 0 = Gol, 1 = Bussola
  int16_t headingBussola;   // heading atual da bussola para calibração
};

struct StatusComunicacao {
  bool olhoOK;
  bool peOK;
  bool allOK;  // tudo funcionando
};

MenuData menuRecebido = {0, 0};
bool estaNoMenu = false;
StatusComunicacao statusCom = {false, false, false};
bool testeCompleto = false;

void setup() {

  // -------- I2C SETUP (para OLED) ----------
  Wire.begin(SDA_PIN, SCL_PIN);

  // -------- OLED DISPLAY SETUP ----------
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 OLED nao encontrado!"));
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("MUSCULO INICIALIZADO"));
  display.display();
  delay(2000);
  display.clearDisplay();
  
  // -------- SERIAL SETUP ----------
  Serial1.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);

  // -------- OUTPUTS ----------
  pinMode(IN1_1_A, OUTPUT);
  pinMode(IN2_1_A, OUTPUT);
  pinMode(IN1_2_A, OUTPUT);
  pinMode(IN2_2_A, OUTPUT);

  pinMode(IN1_1_B, OUTPUT);
  pinMode(IN2_1_B, OUTPUT);
  pinMode(IN1_2_B, OUTPUT);
  pinMode(IN2_2_B, OUTPUT);

  // -------- PWM SETUP ----------
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_A, PWM_CH1);

  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_A, PWM_CH2);

  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_B, PWM_CH3);

  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_B, PWM_CH4);
  
  // -------- TESTE DE COMUNICAÇÃO ----------
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 25);
  display.println("TESTANDO...");
  display.display();
  
  testeCompleto = false;
  unsigned long tempoInicio = millis();
  while (!testeCompleto && (millis() - tempoInicio) < 5000) {
    receberDadosMenu();
    delay(50);
  }
  
  // Exibir resultado
  exibirResultadoTeste();
  delay(3000);  // Mostrar resultado por 3 segundos
  
  // Marcar que estamos no menu
  estaNoMenu = true;
  display.clearDisplay();
  display.display();
}


void Motor_1(int vel1){
int pwm1 = constrain(abs(vel1), 0, 255);
ledcWrite(PWM_CH1, pwm1);
if(vel1 >= 0){
digitalWrite(IN1_1_A, HIGH);
digitalWrite(IN2_1_A, LOW);
} else {
digitalWrite(IN1_1_A, LOW);
digitalWrite(IN2_1_A, HIGH);
}
}


void Motor_2(int vel2){
int pwm2 = constrain(abs(vel2), 0, 255);
ledcWrite(PWM_CH2, pwm2);
if(vel2 >= 0){
digitalWrite(IN1_2_A, HIGH);
digitalWrite(IN2_2_A, LOW);
} else {
digitalWrite(IN1_2_A, LOW);
digitalWrite(IN2_2_A, HIGH);
}
}

void Motor_3(int vel3){
int pwm3 = constrain(abs(vel3), 0, 255);
ledcWrite(PWM_CH3, pwm3);
if(vel3 >= 0){
digitalWrite(IN1_1_B, HIGH);
digitalWrite(IN2_1_B, LOW);
} else {
digitalWrite(IN1_1_B, LOW);
digitalWrite(IN2_1_B, HIGH);
}
}

void Motor_4(int vel4){
int pwm4 = constrain(abs(vel4), 0, 255);
ledcWrite(PWM_CH4, pwm4);
if(vel4 >= 0){
digitalWrite(IN1_2_B, HIGH);
digitalWrite(IN2_2_B, LOW);
} else {
digitalWrite(IN1_2_B, LOW);
digitalWrite(IN2_2_B, HIGH);
}
}



void FRENTE(int vel1, int vel2, int vel3, int vel4){
Motor_1(vel1);
Motor_2(vel2);
Motor_3(-vel3);
Motor_4(-vel4);
}

// ----------- FUNCOES DISPLAY OLED ---------
void exibirDisplay(String linha1, String linha2 = "", String linha3 = "", String linha4 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  if (linha1 != "") {
    display.println(linha1);
  }
  if (linha2 != "") {
    display.println(linha2);
  }
  if (linha3 != "") {
    display.println(linha3);
  }
  if (linha4 != "") {
    display.println(linha4);
  }
  
  display.display();
}

// Desenhar menu no display com opção selecionada invertida
void desenharMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("==== MENU ====");
  display.println();
  
  // Opção 0: Calibração
  if (menuRecebido.itemSelecionado == 0) {
    // Invertido (preto com texto branco)
    display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(5, 25);
    display.println("CALIBRACAO");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(5, 25);
    display.println("CALIBRACAO");
  }
  
  // Opção 1: Iniciar
  if (menuRecebido.itemSelecionado == 1) {
    // Invertido (preto com texto branco)
    display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(5, 41);
    display.println("INICIAR");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(5, 41);
    display.println("INICIAR");
  }
  
  display.display();
}

// Desenhar submenu de calibração
void desenharSubmenuCalibracao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("==== CALIBRACAO ====");
  display.println();
  
  // Se está no menu principal de calibração
  if (menuRecebido.subMenuCal == 0) {
    // Mostra opções: Gol / Bussola
    if (menuRecebido.itemSubMenuCal == 0) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(5, 25);
      display.println("GOL");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(5, 25);
      display.println("GOL");
    }
    
    if (menuRecebido.itemSubMenuCal == 1) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(5, 41);
      display.println("BUSSOLA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(5, 41);
      display.println("BUSSOLA");
    }
  }
  // Se está calibrando a bussola
  else if (menuRecebido.subMenuCal == 2) {
    display.println("=== CAL BUSSOLA ===");
    display.println();
    display.print("Heading: ");
    display.print(menuRecebido.headingBussola);
    display.println(" graus");
    display.println();
    display.println("Aperte para gravar");
    display.println("o valor!");
  }
  // Se está calibrando o gol
  else if (menuRecebido.subMenuCal == 1) {
    display.println("=== CAL GOL ===");
    display.println();
    display.println("Em desenvolvimento...");
  }
  
  display.display();
}

// Desenhar dados ao vivo quando em estado INICIAR
void desenharDadosOperacao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("==== OPERACAO ====");
  display.println();
  
  // Exibir ângulo da bola (IR)
  display.print("IR: ");
  if (menuRecebido.IRdetectado) {
    display.print(menuRecebido.anguloBola / 10.0, 1);  // divide por 10 para desescalar
    display.println(" deg");
  } else {
    display.println("Não detectado");
  }
  
  // Exibir ângulo da linha
  display.print("Linha: ");
  if (menuRecebido.linhaDetectada) {
    display.print(menuRecebido.anguloLinha / 10.0, 1);  // divide por 10 para desescalar
    display.println(" deg");
  } else {
    display.println("Não detectada");
  }
  
  display.println();
  display.println("Pressione Botão 3");
  display.println("para voltar ao MENU");
  
  display.display();
}

// Receber dados do menu via Serial1
void receberDadosMenu() {
  while (Serial1.available() >= 4) {
    byte marcador = Serial1.peek();
    
    // Verificar se é mensagem de status de comunicação (0xFF)
    if (marcador == 0xFF) {
      Serial1.read();  // consome 0xFF
      if (Serial1.available() >= sizeof(StatusComunicacao) + 1) {
        StatusComunicacao temp;
        Serial1.readBytes((uint8_t*)&temp, sizeof(StatusComunicacao));
        byte stop = Serial1.read();
        if (stop == 0xFE) {  // 0xFE é fim do status
          statusCom = temp;
          testeCompleto = true;
        }
      }
    }
    // Caso contrário, é MenuData (começa com BYTE_INICIA)
    else if (marcador == BYTE_INICIA) {
      Serial1.read();  // consome BYTE_INICIA
      if (Serial1.available() >= sizeof(MenuData) + 1) {
        MenuData temp;
        Serial1.readBytes((uint8_t*)&temp, sizeof(MenuData));
        byte stop = Serial1.read();
        if (stop == BYTE_PARA) {
          menuRecebido = temp;
          estaNoMenu = (menuRecebido.estadoMenu == 0);  // se estado é MENU (0)
        }
      }
    } else {
      Serial1.read();  // descarta byte inválido
    }
  }
}

// Exibir resultado do teste de comunicação
void exibirResultadoTeste() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("===== TESTE ======");
  display.println();
  
  display.print("OLHO: ");
  display.println(statusCom.olhoOK ? "OK" : "FALHA");
  
  display.print("PE: ");
  display.println(statusCom.peOK ? "OK" : "FALHA");
  
  display.println();
  if (statusCom.allOK) {
    display.println("TUDO OK!");
  } else {
    display.println("Verifique conexoes!");
  }
  
  display.display();
}











void loop(){
  // Receber dados do menu e sensores da cabeça
  receberDadosMenu();
  
  // Se estamos no estado MENU, desenhar o menu
  if (menuRecebido.estadoMenu == 0) {
    desenharMenu();
  } 
  // Se estamos em CALIBRACAO
  else if (menuRecebido.estadoMenu == 1) {
    desenharSubmenuCalibracao();
  }
  // Se estamos em INICIAR, mostrar dados ao vivo
  else if (menuRecebido.estadoMenu == 2) {
    desenharDadosOperacao();
  }
  
  delay(50);
}