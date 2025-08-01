#include <Arduino.h>
#include <Ultrasonic.h>

// --- 1. Definição dos Pinos ---
// É uma boa prática definir todos os pinos no início.
// Facilita a manutenção e a leitura do código.

// Motores (L298N)
// Motor Esquerda
const int ENA_PIN = 32; // Pino de Enable A (controle de velocidade)
const int IN1_PIN = 27;
const int IN2_PIN = 14;
// Motor Direita
const int ENB_PIN = 33; // Pino de Enable B (controle de velocidade)
const int IN3_PIN = 26;
const int IN4_PIN = 25;

// Sensor Ultrassônico Frontal (HC-SR04)
const int TRIG_FRONT_PIN = 19;
const int ECHO_FRONT_PIN = 18;
// Segundo Sensor Ultrassônico (opcional, para evitar colisões laterais)
const int TRIG_SIDE_PIN = 23;
const int ECHO_SIDE_PIN = 22;

// --- Configurações de PWM para Controle de Velocidade ---
const int PWM_FREQUENCY = 5000;    // Frequência do sinal PWM em Hz. 5000 é um bom valor para motores.
const int PWM_RESOLUTION = 8;      // Resolução em bits. 8 bits = valores de 0 a 255.
const int LEFT_MOTOR_CHANNEL = 0;  // Canal PWM para o motor da esquerda (ENA)
const int RIGHT_MOTOR_CHANNEL = 1; // Canal PWM para o motor da direita (ENB)

const int HALF_SPEED = 200; // Metade da velocidade máxima (255 / 2)

// --- 2. Criação do Objeto do Sensor ---
// Criamos uma "variável" especial do tipo Ultrasonic.
// A biblioteca cuidará da complexidade de ler os dados para nós.
Ultrasonic ultrasonicLeft(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
Ultrasonic ultrasonicRight(TRIG_SIDE_PIN, ECHO_SIDE_PIN);

// --- Declarações das Funções de Movimento ---
// É uma boa prática declarar as funções aqui, antes de usá-las.
void moveForward();
void turnLeft();
void turnRight();
void stopMotors();

void setup() {
  // --- 3. Configuração Inicial (Roda uma vez) ---

  // Inicia a comunicação com o computador para podermos ver mensagens de debug.
  Serial.begin(115200);
  Serial.println("Iniciando o Robô...");

  // Configura todos os pinos de controle dos motores como SAÍDA (OUTPUT).
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Configura os canais PWM com a frequência e resolução definidas.
  ledcSetup(LEFT_MOTOR_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RIGHT_MOTOR_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  // Associa os pinos ENA e ENB aos seus respectivos canais PWM.
  ledcAttachPin(ENA_PIN, LEFT_MOTOR_CHANNEL);
  ledcAttachPin(ENB_PIN, RIGHT_MOTOR_CHANNEL);
  // Garante que o robô comece parado.
  stopMotors();

  Serial.println("Setup concluído. Robô pronto.");
}

void loop() {
  // --- 4. Loop Principal (Roda para sempre) ---

  // 1. Lê a distância do sensor frontal.
  // A função .read() retorna a distância em centímetros por padrão.
  long distanciaEsquerda = ultrasonicLeft.read();
  long distanciaDireita = ultrasonicRight.read();

  // 2. Mostra a distância no Monitor Serial para podermos depurar.
  // Para ver isso, use o comando "Upload and Monitor" do PlatformIO.
  Serial.print("Distância do obstáculo esquerda: ");
  Serial.print(distanciaEsquerda);
  Serial.println(" cm");
  Serial.print("Distância do obstáculo direita: ");
  Serial.print(distanciaDireita);
  Serial.println(" cm");

  // --- Lógica de Decisão Inteligente ---
  // Verifica cada sensor individualmente para tomar a decisão correta.
  // A condição ">= 2" respeita a "zona cega" do sensor.
  if (distanciaEsquerda < 20 && distanciaEsquerda >= 2) {
    // Se o obstáculo está à ESQUERDA, vira para a DIREITA.
    Serial.println("Obstáculo à esquerda! Virando para a direita...");
    turnRight();
    delay(500); // Vira por meio segundo. Ajuste este valor conforme o comportamento do seu robô.
  } else if (distanciaDireita < 20 && distanciaDireita >= 2) {
    // Se o obstáculo está à DIREITA, vira para a ESQUERDA.
    Serial.println("Obstáculo à direita! Virando para a esquerda...");
    turnLeft();
    delay(500); // Vira por meio segundo. Ajuste este valor conforme o comportamento do seu robô.
  } else {
    // Se o caminho está livre...
    Serial.println("Caminho livre. Movendo para frente.");
    moveForward();
  }
  delay(100); // Pequena pausa para estabilidade.
}

// --- 5. Definições das Funções de Movimento ---
// ATENÇÃO: As funções devem ser definidas FORA do loop().

void moveForward() {

  // Motor Esquerda para frente
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  // Motor Direita para frente (geralmente em direção oposta à esquerda)
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  // Define a velocidade para ambos os motores
  ledcWrite(LEFT_MOTOR_CHANNEL, HALF_SPEED);
  ledcWrite(RIGHT_MOTOR_CHANNEL, HALF_SPEED);
}

void turnRight() {
  // Para virar para a direita, fazemos o oposto de virar para a esquerda.
  // Motor Esquerda para frente

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  // Motor Direita para trás
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  // Define a velocidade para ambos os motores
  ledcWrite(LEFT_MOTOR_CHANNEL, HALF_SPEED);
  ledcWrite(RIGHT_MOTOR_CHANNEL, HALF_SPEED);
}

void turnLeft() {
  // Para virar no próprio eixo (pivô), um motor vai para frente e o outro para trás.
  // Motor Esquerda para trás
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  // Motor Direita para frente
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  // Define a velocidade para ambos os motores
  ledcWrite(LEFT_MOTOR_CHANNEL, HALF_SPEED);
  ledcWrite(RIGHT_MOTOR_CHANNEL, HALF_SPEED);
}

void stopMotors() {
  // Para parar, colocamos LOW em ambos os pinos de controle de cada motor.
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  // Também é uma boa prática zerar a velocidade via PWM.
  ledcWrite(LEFT_MOTOR_CHANNEL, 0);
  ledcWrite(RIGHT_MOTOR_CHANNEL, 0);
}
