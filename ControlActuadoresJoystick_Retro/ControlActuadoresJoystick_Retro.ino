/*
act1: Freno.           0x0D
act2: Acelerador.      0x0A


Documentacion cableado
A0: Tap central joystick control. Incremento acelera, decremento frena.
A2: Pot actuador 1 (Freno)
A3: Pot actuador 2 (Accel)

Pin 8 digital: Tx hacia los puentes

*/







//****Variables para comunicacin serial.****//
#include <SoftwareSerial.h>

#define rxPin 7 
#define txPin 8  // pin8 is asssigned as TX pin of arduino

SoftwareSerial smcSerial = SoftwareSerial (rxPin, txPin);
//******************************************//


//****Metodos para control de puente H.****//
void exitSafeStart_Protocol(byte address)
{
  smcSerial.write(0xAA);
  smcSerial.write(address);
  // Function for exiting from error mode
  //smcSerial.write(0x83);
  smcSerial.write(0x03); //Se pone en 0 el MSB.
}

void setMotorSpeed_Protocol(int address, int speed)  // Function for sending data to motor controller
{
  if(speed < -3200) speed = -3200;
  if(speed >  3200) speed =  3200;
  
  smcSerial.write(0xAA);
  smcSerial.write(address);
  if (speed < 0) 
  {
    //smcSerial.write(0x86);
    smcSerial.write(0x06); //motor reverse command with MSB cleared.
    speed = -speed;//make the value of the speed positive
  }
  else 
  {
    //smcSerial.write(0x85);//motor forward command
    smcSerial.write(0x05);
  }
  smcSerial.write(speed & 0x1F); //Primer byte de velocidad.
  smcSerial.write(speed >> 5);   //Segundo byte de velocidad.
}
//******Fin Metodos para control de puente H.****//


/***Considerando que el actuador no se utilizara en todo su rango
se define una constante de mapeo.***/ 
////Encontrar valores con pruebas en ControlActuadores_Joystick_NoRetro

//Freno.
// Extendido 28 - 1009 Retraido.
// Primero considerar carrera completa. Despues considerar carrera de 8 cm en lugar de los 10 posibles (1009/10*8 = 807.2)
int act1_minVal = 28;   // Extendido.
int act1_maxVal = 964; // Retraido.

//Acelerador.
int act2_minVal = 28;
//int act2_minVal = 350;   // Extendido.
//int act2_maxVal = 965;
int act2_maxVal = 350;//El acelerador esta inicialmente ligeramente extendido, para que quede en contacto con el acelerador.


//Rango del joystick (Valores experimentales).
int joyForward_minVal = 513;
int joyForward_maxVal = 1023;

int joyBackward_minVal = 0;
int joyBackward_maxVal = 512;  


int acel_posDeseada = 0;    
int freno_posDeseada = 0;
int error_acel;
int error_freno;
int controlAcel, controlFreno;

int frenAddress = 0x0D; //13 = 1101
int acelAddress = 0x0A; //10 = 1010


//Variables para el filtrado de las señales analogicas.
#define filterSamples 11  
int joySmoothArray[filterSamples]; // Arreglo para almacenar los valores analogicos crudos.
int smoothJS_Y;

int act1SmoothArray[filterSamples]; //Valores crudos de la retroalimentacion del actuador 1.
int smoothAct1;

int act2SmoothArray[filterSamples];
int smoothAct2;

//Constantes de control PID.
//int Kp = 20; //Buscar adecuada
int Kp = 25;
int Ki = 1;
float Kd = 0;

int umbral = 2; //Valor para evitar mover los actuadores con valores muy pequeños en el potenciometro
int umbral_error = 4;

void setup()
{
  Serial.begin(115200);
  smcSerial.begin(19200);
  //exitSafeStart_Protocol(frenAddress); 
}

void loop()
{
  int joystick1_Y = analogRead(A0);
  smoothJS_Y = digitalSmooth(joystick1_Y, joySmoothArray);
  
  
  int act1_PosRaw = analogRead(A2); //Freno
  smoothAct1 = digitalSmooth(act1_PosRaw, act1SmoothArray);
  
  int act2_PosRaw = analogRead(A3); //Acelerador
  smoothAct2 = digitalSmooth(act2_PosRaw, act2SmoothArray);
  
 
  if(smoothJS_Y > (joyForward_minVal + umbral) ) //Se quiere acelerar
  {
    freno_posDeseada = act1_maxVal; //Valor del actuador retraido.
    
    acel_posDeseada = map(smoothJS_Y, joyForward_minVal, joyForward_maxVal, act2_maxVal, act2_minVal);//Mapeo inverso (si el joystick aumenta el valor del actuador disminuye)

  }
  else if(smoothJS_Y < (joyBackward_maxVal - umbral) ) //Se quiere frenar.
  {
    acel_posDeseada = act2_maxVal;//Valor del actuador retraido.
    //                             x                 0                  512             28          1009
    freno_posDeseada = map(smoothJS_Y, joyBackward_minVal, joyBackward_maxVal, act1_minVal, act1_maxVal); //Mapeo directo (si el joystick aumenta el valor del actuador aumenta)
    /*
    512-3 = 509 -> Freno casi totalmente retraido  (~1009)
              0 -> Freno totalmente extendido      (28) 
    */
  }
  else //El joystick esta centrado. Retraer ambos actuadores.
  {
    freno_posDeseada = act1_maxVal; //Retraer completamente el freno.
    acel_posDeseada = act2_maxVal; //Retraer completamente el acelerador.
  }
  
  //Calcular valores de control con retroalimentacion
  error_freno = smoothAct1 - freno_posDeseada; //
  controlFreno = error_freno*Kp;
  
  error_acel = acel_posDeseada - smoothAct2; //
  //error_acel = smoothAct2 - acel_posDeseada;
  controlAcel = -error_acel*Kp;//
  
    
    
  Serial.print("j:");  Serial.print(smoothJS_Y);
  //Serial.print(", Jf:"); Serial.print(smoothJoystick_Y);

  Serial.print(",\t\tF_po:"); Serial.print(smoothAct1); //Valor del potenciometro del act1 (Freno)
  Serial.print(", F_de:"); Serial.print(freno_posDeseada); //Valor deseado para el pot del freno.
  Serial.print(", F_er:"); Serial.print(error_freno); //valor - posDeseada
  Serial.print(", F_ct:"); Serial.print(controlFreno); //Valor de control aplicado al actuador de freno
  
    
  Serial.print(",\t\tA_po:"); Serial.print(smoothAct2);
  Serial.print(", A_de:"); Serial.print(acel_posDeseada);
  Serial.print(", A_er:"); Serial.print(error_acel);
  Serial.print(", A_ct:"); Serial.println(controlAcel);
  
  
  //Mover el freno.
  if(error_freno > umbral_error || error_freno < -umbral_error)
    setMotorSpeed_Protocol(frenAddress, controlFreno);
  
  //Mover el acelerador.
  if(error_acel > umbral_error || error_acel < -umbral_error)
    setMotorSpeed_Protocol(acelAddress, controlAcel);
}

