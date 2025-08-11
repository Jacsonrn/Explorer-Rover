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

// --- Pinos dos Sensores Ultrassônicos (HC-SR04) ---
// Sensores Frontais
const int TRIG_FRONT_LEFT_PIN = 19;
const int ECHO_FRONT_LEFT_PIN = 18;
const int TRIG_FRONT_RIGHT_PIN = 23;
const int ECHO_FRONT_RIGHT_PIN = 22;
const int TRIG_FRONT_CENTER_PIN = 17;
const int ECHO_FRONT_CENTER_PIN = 21;
// Sensores Laterais
const int TRIG_SIDE_LEFT_PIN = 4;
const int ECHO_SIDE_LEFT_PIN = 5;
const int TRIG_SIDE_RIGHT_PIN = 13;
const int ECHO_SIDE_RIGHT_PIN = 16;

// --- Configurações de PWM para Controle de Velocidade ---
const int PWM_FREQUENCY = 5000;    // Frequência do sinal PWM em Hz. 5000 é um bom valor para motores.
const int PWM_RESOLUTION = 8;      // Resolução em bits. 8 bits = valores de 0 a 255.
const int LEFT_MOTOR_CHANNEL = 0;  // Canal PWM para o motor da esquerda (ENA)
const int RIGHT_MOTOR_CHANNEL = 1; // Canal PWM para o motor da direita (ENB)

// --- Constantes de Comportamento ---
const int MOVE_SPEED = 255;          // Velocidade de movimento (0-255)
const int TURN_SPEED = 230;          // Velocidade de curva, mais lenta para precisão
const int OBSTACLE_DISTANCE_CM = 40; // Distância para considerar um obstáculo
const int REVERSE_DURATION_MS = 250; // Duração da manobra de ré
const int TURN_DURATION_MS = 400;    // Duração da curva

// --- Máquina de Estados (State Machine) ---
// Define os possíveis estados do robô.
enum RobotState {
  MOVING_FORWARD,
  MANEUVER_START,
  MANEUVER_REVERSING,
  MANEUVER_TURNING_LEFT,
  MANEUVER_TURNING_RIGHT
};

RobotState currentState = MOVING_FORWARD; // O robô começa andando para frente.
unsigned long stateChangeTimestamp = 0;   // Guarda o tempo em que uma manobra começou.


// --- 2. Criação do Objeto do Sensor ---
// Criamos uma "variável" especial do tipo Ultrasonic.
// A biblioteca cuidará da complexidade de ler os dados para nós.
Ultrasonic ultrasonicFrontLeft(TRIG_FRONT_LEFT_PIN, ECHO_FRONT_LEFT_PIN);
Ultrasonic ultrasonicFrontRight(TRIG_FRONT_RIGHT_PIN, ECHO_FRONT_RIGHT_PIN);
Ultrasonic ultrasonicFrontCenter(TRIG_FRONT_CENTER_PIN, ECHO_FRONT_CENTER_PIN);
Ultrasonic ultrasonicSideLeft(TRIG_SIDE_LEFT_PIN, ECHO_SIDE_LEFT_PIN);
Ultrasonic ultrasonicSideRight(TRIG_SIDE_RIGHT_PIN, ECHO_SIDE_RIGHT_PIN);

// --- Declarações das Funções de Movimento ---
// É uma boa prática declarar as funções aqui, antes de usá-las.
void moveForward();
void moveBackward();
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
  // A função loop agora apenas gerencia o estado atual do robô.
  // A lógica é executada milhares de vezes por segundo, sem bloqueios.

  switch (currentState) {
    case MOVING_FORWARD: {
      moveForward();
      // Lê apenas os sensores frontais enquanto se move para frente.
      long distFrontLeft = ultrasonicFrontLeft.read();
      long distFrontCenter = ultrasonicFrontCenter.read();
      long distFrontRight = ultrasonicFrontRight.read();

      Serial.printf("Avançando... Distâncias Frontais (E/C/D): %ld/%ld/%ld cm\n",
                    distFrontLeft, distFrontCenter, distFrontRight);
      Serial.printf("Distâncias Laterais (E/D): %ld/%ld cm\n",
                    ultrasonicSideLeft.read(), ultrasonicSideRight.read());

      // Condição para iniciar uma manobra
      if ((distFrontLeft < OBSTACLE_DISTANCE_CM && distFrontLeft >= 2) ||
          (distFrontCenter < OBSTACLE_DISTANCE_CM && distFrontCenter >= 2) ||
          (distFrontRight < OBSTACLE_DISTANCE_CM && distFrontRight >= 2)) {
        currentState = MANEUVER_START;
      }
      break;
    }

    case MANEUVER_START: {
      Serial.println("Obstáculo detectado! Iniciando manobra...");
      stopMotors();
      // Lê os sensores laterais para decidir para onde virar.
      long distSideLeft = ultrasonicSideLeft.read();
      long distSideRight = ultrasonicSideRight.read();
      
      stateChangeTimestamp = millis(); // Marca o início da manobra de ré.
      currentState = MANEUVER_REVERSING;

      // Pré-decide para qual lado virar e armazena para o próximo estado.
      if (distSideLeft > distSideRight) {
        // Se o lado esquerdo tem mais espaço, vira para a esquerda.
        Serial.println("Planejando virar para a esquerda.");
        // Usamos um truque: mudamos para o estado de virar à esquerda, mas a ré ainda será executada.
        // Quando a ré terminar, ele já saberá para onde ir.
        currentState = MANEUVER_TURNING_LEFT;
      } else {
        Serial.println("Planejando virar para a direita.");
        currentState = MANEUVER_TURNING_RIGHT;
      }
      // Inicia a ré imediatamente
      moveBackward();
      break;
    }

    case MANEUVER_TURNING_LEFT:
    case MANEUVER_TURNING_RIGHT: {
      // Este estado é um pouco complexo: ele começa com a ré e depois vira.
      if (millis() - stateChangeTimestamp <= REVERSE_DURATION_MS) {
        // Ainda está no tempo da ré.
        moveBackward();
      } else if (millis() - stateChangeTimestamp <= REVERSE_DURATION_MS + TURN_DURATION_MS) {
        // O tempo da ré acabou, agora vira.
        if (currentState == MANEUVER_TURNING_LEFT) {
          turnLeft();
        } else {
          turnRight();
        }
      } else {
        // A manobra inteira (ré + curva) terminou.
        Serial.println("Manobra concluída. Retomando movimento.");
        currentState = MOVING_FORWARD;
      }
      break;
    }
  }
  delay(10); // Pequena pausa para não sobrecarregar o processador.
}

// --- 5. Definições das Funções de Movimento ---
// ATENÇÃO: As funções devem ser definidas FORA do loop().

void moveForward() {
  // Para ir para frente, ambos os motores giram para frente.
  // Motor Esquerda para frente
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  // Motor Direita para frente
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  // Define a velocidade para ambos os motores
  ledcWrite(LEFT_MOTOR_CHANNEL, MOVE_SPEED);
  ledcWrite(RIGHT_MOTOR_CHANNEL, MOVE_SPEED);
}

void moveBackward() {
  // Para ir para trás, ambos os motores giram para trás.
  // Motor Esquerda para trás
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  // Motor Direita para trás
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  // Define a velocidade para ambos os motores
  ledcWrite(LEFT_MOTOR_CHANNEL, MOVE_SPEED);
  ledcWrite(RIGHT_MOTOR_CHANNEL, MOVE_SPEED);
}

void turnRight() {
  // Para virar à direita no próprio eixo (pivô).
  // Motor Esquerda para frente
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  // Motor Direita para trás
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  // Define a velocidade para ambos os motores
  ledcWrite(LEFT_MOTOR_CHANNEL, TURN_SPEED);
  ledcWrite(RIGHT_MOTOR_CHANNEL, TURN_SPEED);
}

void turnLeft() {
  // Para virar à esquerda no próprio eixo (pivô).
  // Motor Esquerda para trás
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  // Motor Direita para frente
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  // Define a velocidade para ambos os motores
  ledcWrite(LEFT_MOTOR_CHANNEL, TURN_SPEED);
  ledcWrite(RIGHT_MOTOR_CHANNEL, TURN_SPEED);
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
