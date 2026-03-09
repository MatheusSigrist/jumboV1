#include <Arduino.h>
#include <math.h>
#define BYTE_INICIA 0xAA
#define BYTE_PARA 0x55
#include <HCSR04.h>    // <-- Biblioteca Ultrassonico

bool atacante = false;  // true = atacante, false = defensor
bool jaSaudouCabeca = false;  // Flag para enviar "OK" no startup

#define ID_PLACA_OLHO 0x01
#define ID_PLACA_PE 0x02



HCSR04 hcT(4, 5);    // <-- Ultrassonico 1 - TRIGG, ECHO
HCSR04 hcF(10, 9);    // <-- Ultrassonico 2 - TRIGG, ECHO
HCSR04 hcD(21, 47);    // <-- Ultrassonico 3 - TRIGG, ECHO
HCSR04 hcE(37, 36);    // <-- Ultrassonico 4 - TRIGG, ECHO

float  ultraT = 0;
float  ultraF = 0;
float  ultraE = 0;
float  ultraD = 0;

void L_Ultra() {     // <-- Função para leitura dos sensores ultrassonicos
  ultraD = hcD.dist();
  ultraE = hcE.dist();
  ultraF = hcF.dist();
  ultraT = hcT.dist();

}


#define RX_CABECA 20
#define TX_CABECA 30
#define RX_CAMERA 40 
#define TX_CAMERA 50
//#define ID_PLACA_CIMA 0x01  // antigo
#define ID_PLACA_OLHO 0x01  // usado ao enviar dados para cabeça













//------------------------------------ IR SEEKER -----------------------------//
const int NUM_SENSORES = 12;
const int sensoresTSOP[NUM_SENSORES] = {
  6, 7, 46, 11, 12, 13, 14, 48,
  45, 35, 38, 39  
};
float angulos[NUM_SENSORES] = {
  0, 30, 60, 90, 120, 150, 180, 210,
  240, 270, 300, 330 
};
const unsigned long JANELA_TEMPO = 10;    // <-- Janela de tempo para contagens de pulso de IR (10ms)
const int LIMIAR_PULSOS = 2;    // <-- Limiar de pulsos para identificar que é a bola
unsigned int pulsos[NUM_SENSORES];
unsigned int intensidade = 0;  // <-- Declarada globalmente para ser usada em enviarDados()



void contarPulsosSensores() {     // <-- Função de contagens de pulsos IR emitidos pela bola (IR SEEKER)
  for (int i = 0; i < NUM_SENSORES; i++) pulsos[i] = 0;
  unsigned long t0 = millis();
  int oldState[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++)
    oldState[i] = digitalRead(sensoresTSOP[i]);
  while (millis() - t0 < JANELA_TEMPO) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int s = digitalRead(sensoresTSOP[i]);
      if (oldState[i] == HIGH && s == LOW) {
        pulsos[i]++;
      }
      oldState[i] = s;
    } 
  }
  // Atualizar intensidade global
  intensidade = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    intensidade += pulsos[i];
  }
}

float calculaAnguloBola() {     // <-- Função para cálculo do angulo da bola pelos pulsos de IR lidos (IR SEEKER)
  float x = 0, y = 0, soma_pesos = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (pulsos[i] >= LIMIAR_PULSOS) {
      float rad = angulos[i] * PI / 180.0;
      x += pulsos[i] * cos(rad);
      y += pulsos[i] * sin(rad);
      soma_pesos += pulsos[i];
    }
  }
  if (soma_pesos == 0) return -1.0;
  float angulo_bola = atan2(y, x) * 180.0 / PI;
  if (angulo_bola < 0) angulo_bola += 360.0;
  return angulo_bola;
}












//---------------jeitim-2------------------//
#include <stdint.h>

struct Pacote {
  int16_t uD;
  int16_t uE;
  int16_t uF;
  int16_t uT;
  int16_t angulo;
  int16_t intensidade;
};

struct PacoteEstado {
  bool sozinho;    // mantido para compatibilidade
  bool atacante;   // true = atacante, false = defensor
};

void LeituraSerial() {
  while (Serial1.available() >= 2) {
    if (Serial1.read() == BYTE_INICIA) {
      byte id = Serial1.read();
      if (id == ID_PLACA_OLHO || id == ID_PLACA_PE) {
        if (Serial1.available() >= sizeof(PacoteEstado) + 1) {
          PacoteEstado temp;
          Serial1.readBytes((uint8_t*)&temp, sizeof(PacoteEstado));
          byte stop = Serial1.read();
          if (stop == BYTE_PARA) {
            atacante = temp.atacante;  // recebe o papel (atacante/defensor)
          }
        }
      }
    }
  }
}


void enviarDados() { 
  Pacote p;

  p.uD = (int16_t)round(ultraD * 10.0);
  p.uE = (int16_t)round(ultraE * 10.0);
  p.uF = (int16_t)round(ultraF * 10.0);
  p.uT = (int16_t)round(ultraT * 10.0);
  p.angulo = (int16_t)round(calculaAnguloBola() * 10.0);
  p.intensidade = (int16_t)round(intensidade * 10.0);

  Serial1.write(BYTE_INICIA);
  Serial1.write(ID_PLACA_OLHO);
  Serial1.write((uint8_t*)&p, sizeof(Pacote));
  Serial1.write(BYTE_PARA);
}

//--------------------------------------//





void setup() {
    for (int i = 0; i < NUM_SENSORES; i++)
      pinMode(sensoresTSOP[i], INPUT);
    Serial1.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);
}

void loop(){
  // Na primeira iteração, enviar "OK" para a cabeça (para comunicação teste)
  if (!jaSaudouCabeca) {
    Serial1.print("OK");
    jaSaudouCabeca = true;
    delay(100);
  }

  contarPulsosSensores(); // Leitura IR SEEKER
  L_Ultra(); // Leitura dos ultras

  // ler estado recebido da cabeça (atacante/defensor)
  LeituraSerial();

  enviarDados(); 
  delay(50);
}
}