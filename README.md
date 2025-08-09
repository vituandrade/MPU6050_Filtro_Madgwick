# MPU6050 - Obtenção de Quaternions com Filtro de Madgwick

<div align="center">
  <img src="https://github.com/user-attachments/assets/4027d6d9-f44a-4ca7-816d-44c832a574bf" alt="MPU6050 Quaternion Banner">
</div>

Projeto para leitura da orientação de um sensor MPU6050, obtendo quaternions em tempo real através do filtro de Madgwick.  
Inclui rotina de calibração automática para maior precisão durante o voo.

---

## O que foi feito neste projeto

O desenvolvimento deste código envolveu as seguintes etapas:

- Configuração do MPU6050
- Implementação de calibração automática para acelerômetro e giroscópio  
- Processamento dos dados inerciais pelo **Filtro de Madgwick** para cálculo de quaternions  
- Envio dos quaternions via porta serial para telemetria ou visualização 3D  

---

## Funcionamento

O sistema realiza a leitura bruta do acelerômetro e giroscópio, aplica os fatores de escala, corrige os offsets obtidos na calibração e processa os dados na função **MadgwickQuaternionUpdate**.  

O resultado é enviado no seguinte formato:

```
q0,q1,q2,q3
```

Onde cada `q` representa um componente do quaternion de orientação.

---

## Principais recursos

- **Calibração automática**
- **Filtro de Madgwick otimizado** para estabilidade e precisão  

---

## Conexões do MPU6050

- **VCC** → 3.3V ou 5V  
- **GND** → GND do microcontrolador  
- **SCL** → Pino de clock I²C (ex.: A5 no Arduino Uno)  
- **SDA** → Pino de dados I²C (ex.: A4 no Arduino Uno)  

---

## Trainees

**Victor Henrick Santos Andrade**  
Trainee no projeto de leitura inercial para foguetes  

**Guilherme Miller Gama Cardoso**  
Trainee no projeto de leitura inercial para foguetes  
