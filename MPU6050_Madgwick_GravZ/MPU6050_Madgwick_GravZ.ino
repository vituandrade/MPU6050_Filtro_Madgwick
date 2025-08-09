#include <Wire.h>
#include <math.h>

#define MPU_addr 0x68

// ======================================================================
// PASSO 1: CONFIGURE O EIXO QUE ESTARÁ NA VERTICAL (APONTANDO PARA CIMA)
// Mude para 'x', 'y' ou 'z' minúsculo, dependendo da montagem do seu sensor no foguete.
// 'y' é um palpite comum para foguetes.
#define VERTICAL_AXIS 'y'
// ======================================================================

// Offsets de calibração que serão preenchidos automaticamente
float acc_x_offset = 0, acc_y_offset = 0, acc_z_offset = 0;
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// Quaternion de orientação [w, x, y, z]
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Parâmetro beta do filtro de Madgwick
float beta = 0.002f;

// Variáveis de tempo
unsigned long lastUpdate = 0;

// Constantes para as escalas de leitura de alta performance
const float ACCEL_SCALE_FACTOR = 2048.0; // Para ±16g
const float GYRO_SCALE_FACTOR = 16.4;    // Para ±2000 °/s

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Acordar o MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);

  // Configura o giroscópio para ±2000 graus/s
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x18); 
  Wire.endTransmission(true);
  delay(10);

  // Configura o acelerômetro para ±16g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x18);
  Wire.endTransmission(true);
  delay(10);

  // Chama a nova rotina de calibração automática
  calibrateMPUVertical(); 

  lastUpdate = micros();
}

void loop() {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  ax_raw = Wire.read() << 8 | Wire.read();
  ay_raw = Wire.read() << 8 | Wire.read();
  az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gx_raw = Wire.read() << 8 | Wire.read();
  gy_raw = Wire.read() << 8 | Wire.read();
  gz_raw = Wire.read() << 8 | Wire.read();

  // Usa os novos fatores de escala
  float ax = ax_raw / ACCEL_SCALE_FACTOR;
  float ay = ay_raw / ACCEL_SCALE_FACTOR;
  float az = az_raw / ACCEL_SCALE_FACTOR;
  float gx = (gx_raw / GYRO_SCALE_FACTOR) * (PI / 180.0f);
  float gy = (gy_raw / GYRO_SCALE_FACTOR) * (PI / 180.0f);
  float gz = (gz_raw / GYRO_SCALE_FACTOR) * (PI / 180.0f);

  // Aplica a compensação de offset calculada automaticamente
  ax -= acc_x_offset;
  ay -= acc_y_offset;
  az -= acc_z_offset;
  gx -= gyro_x_offset;
  gy -= gyro_y_offset;
  gz -= gyro_z_offset;

  unsigned long now = micros();
  float deltat = (now - lastUpdate) / 1.0e6f;
  lastUpdate = now;

  MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, deltat);

  // Envia os quaternions pela serial para telemetria ou visualização 3D
  Serial.print(q[0]);
  Serial.print(",");
  Serial.print(q[1]);
  Serial.print(",");
  Serial.print(q[2]);
  Serial.print(",");
  Serial.println(q[3]);
  
  delay(20);
}

void calibrateMPUVertical() {
  Serial.println("Iniciando calibração vertical. Mantenha o foguete parado na vertical...");
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int samples = 2000;

  for (int i = 0; i < samples; i++) {
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    ax_raw = Wire.read() << 8 | Wire.read();
    ay_raw = Wire.read() << 8 | Wire.read();
    az_raw = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    gx_raw = Wire.read() << 8 | Wire.read();
    gy_raw = Wire.read() << 8 | Wire.read();
    gz_raw = Wire.read() << 8 | Wire.read();
    ax_sum += ax_raw; ay_sum += ay_raw; az_sum += az_raw;
    gx_sum += gx_raw; gy_sum += gy_raw; gz_sum += gz_raw;
    delay(2);
  }

  // Calcula a média das leituras
  float avg_ax = (ax_sum / (float)samples) / ACCEL_SCALE_FACTOR;
  float avg_ay = (ay_sum / (float)samples) / ACCEL_SCALE_FACTOR;
  float avg_az = (az_sum / (float)samples) / ACCEL_SCALE_FACTOR;

  // A calibração do giroscópio é sempre a média, para zerar a rotação
  gyro_x_offset = (gx_sum / (float)samples / GYRO_SCALE_FACTOR) * (PI / 180.0f);
  gyro_y_offset = (gy_sum / (float)samples / GYRO_SCALE_FACTOR) * (PI / 180.0f);
  gyro_z_offset = (gz_sum / (float)samples / GYRO_SCALE_FACTOR) * (PI / 180.0f);

  // ======================================================================
  // CALIBRAÇÃO INTELIGENTE DO ACELERÔMETRO
  // ======================================================================
  // Zera os eixos que deveriam ser zero e remove 1g do eixo vertical
  if (VERTICAL_AXIS == 'x') {
    acc_x_offset = avg_ax - 1.0f; // Ou +1.0f se o eixo X estiver para baixo
    acc_y_offset = avg_ay;
    acc_z_offset = avg_az;
  } else if (VERTICAL_AXIS == 'y') {
    acc_x_offset = avg_ax;
    acc_y_offset = avg_ay - 1.0f; // Ou +1.0f se o eixo Y estiver para baixo
    acc_z_offset = avg_az;
  } else { // Assume 'z'
    acc_x_offset = avg_ax;
    acc_y_offset = avg_ay;
    acc_z_offset = avg_az - 1.0f; // Ou +1.0f se o eixo Z estiver para baixo
  }

  Serial.println("Calibração concluída!");
  Serial.println("Offsets calculados (copie para a versão de voo):");
  Serial.print("acc_x_offset = "); Serial.println(acc_x_offset, 6);
  Serial.print("acc_y_offset = "); Serial.println(acc_y_offset, 6);
  Serial.print("acc_z_offset = "); Serial.println(acc_z_offset, 6);
  Serial.print("gyro_x_offset = "); Serial.println(gyro_x_offset, 6);
  Serial.print("gyro_y_offset = "); Serial.println(gyro_y_offset, 6);
  Serial.print("gyro_z_offset = "); Serial.println(gyro_z_offset, 6);
  delay(3000);
}


// Filtro de Madgwick - Versão Correta e Estável
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  // ... (código do filtro idêntico ao anterior, está correto) ...
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float f1, f2, f3;
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;

  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;

  J_11or24 = _2q3;
  J_12or23 = 2.0f * q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  q1 += (qDot1 - (beta * hatDot1)) * deltat;
  q2 += (qDot2 - (beta * hatDot2)) * deltat;
  q3 += (qDot3 - (beta * hatDot3)) * deltat;
  q4 += (qDot4 - (beta * hatDot4)) * deltat;

  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}