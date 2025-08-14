// CÓDIGO PARA O PROCESSING

import processing.serial.*; // Importa a biblioteca para comunicação serial
import processing.opengl.*; // Habilita o renderizador 3D

Serial myPort;  // Objeto para a porta serial
float w, x, y, z; // Variáveis para armazenar os quaternions recebidos

void setup() {
  size(800, 600, P3D); // Cria uma janela 3D de 800x600 pixels
  
  // Lista todas as portas seriais disponíveis e imprime no console
  printArray(Serial.list());
  
  // Abre a porta serial correspondente ao seu Arduino.
  // Mude o número no índice [0] se necessário.
  String portName = Serial.list()[2]; 
  myPort = new Serial(this, portName, 9600);
  
  // Configura a serial para ler dados até encontrar uma nova linha ('\n')
  myPort.bufferUntil('\n');
}

void draw() {
  background(20, 20, 50); // Fundo azul escuro
  translate(width/2, height/2, 0); // Centraliza a origem na tela
  
  // Iluminação
  lights();
  
  // Aplica a rotação do quaternion ao objeto
  // Esta é a matriz de rotação padrão derivada de um quaternion
  applyMatrix(w*w + x*x - y*y - z*z, 2*x*y + 2*w*z,         2*x*z - 2*w*y,         0,
              2*x*y - 2*w*z,         w*w - x*x + y*y - z*z, 2*y*z + 2*w*x,         0,
              2*x*z + 2*w*y,         2*y*z - 2*w*x,         w*w - x*x - y*y + z*z, 0,
              0,                     0,                     0,                     1);

  // Desenha o objeto 3D (uma caixa que parece um celular/placa)
  stroke(150, 150, 255); // Cor da borda
  fill(50, 50, 150, 200); // Cor do preenchimento
  box(150, 250, 20); // Desenha a caixa com dimensões (largura, altura, profundidade)
}

// Esta função é chamada automaticamente sempre que dados chegam na serial
void serialEvent(Serial p) {
  String data = p.readStringUntil('\n');
  if (data != null) {
    data = trim(data); // Remove espaços em branco
    
    // Separa a string "w,x,y,z" em um array de floats
    float[] nums = float(split(data, ','));
    
    // Se a string foi lida corretamente (tem 4 valores)
    if (nums.length == 4) {
      w = nums[0];
      x = nums[1];
      y = nums[2];
      z = nums[3];
    }
  }
}
