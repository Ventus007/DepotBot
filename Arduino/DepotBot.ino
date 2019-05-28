#include <QTRSensors.h>

//Bluetooth
int state = 0; // Variable lectura dato serial
int funcion = 0;
int bandera=0;

//Ultrasonido
const int Trigger = 9;   //Pin digital 2 para el Trigger del sensor
const int Echo = 8;   //Pin digital 3 para el Echo del sensor
int para = 0;

//Buzzer
const int pinBuzzer = 10;
int tono = 0;
int suena = 0;
const int tonos[] = {261, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};

//Fotoresistencia
int pinFotores = A5; 
int fotoresValor = 0;
int mitad = 1024/1.4;
int obj_encima = 0;

//LEDS
const int LEDAL = 12;
const int LEDA = 13;
int cont_LED1 = 0;
int cont_LED2 = 0;

//Modo Manual
int velo = 160;
int velo_giro = 100;

//Modo Fiesta
int cont_fiesta = 0;
int fase_fiesta = 0;
int tonePin = 10;

//Modo Autonomo
int inicio = 1;
int fin = 0;
int hacer_alg = 0;
int n_intercepcion = 0;
int cont = 0;
int puedes = 0;
int camino[10]= {0,0,0,0,0,0,0,0,0,0};


#include <stdbool.h>
#include <stdlib.h> 
#include <stdio.h> 
//Globals
int NUM_NODES = 9;

int INT_MAX = 100;

//end of globals


struct Point {
    bool inSPT;
    int pointID;
    int distance; //record the distance of all the points
    Point * prev;
    int northNeighbor;
    int rightNeightbor;
    int leftNeighbor;
    int southNeighbor;
};


int findShortestDistanceInGraph(Point * points){
    int min= INT_MAX;
    int shortestPathPt = 0;
    for (int i = 0; i < NUM_NODES; i++){
//        int distance = points[i].distance;
//        bool inSpt = points[i].inSPT;
        if (points[i].inSPT == false && points[i].distance < min){
            shortestPathPt = i;
            min = points[i].distance;
        }
    }
    return shortestPathPt;
}

void getNumNodesToBeTraced(Point * points,int size, int * nodes, int source, int dest){
    int j = dest;
    int i = size - 1;
    while (j != source && j >=0 && i >= 0){
        nodes[i] = points[j].pointID;
        j = points[j].prev->pointID;
        i --;
    }
    nodes[0] = source;
}


//points start at 0 -->
int shortestPath(int graph[9][9], int source, int dest, int initialDirection, int aux, int position){
    //create an array of points so that we can keep track of previous points for backtrace
    Point points [NUM_NODES];
    for (int i = 0; i < NUM_NODES; i++){
        points[i].pointID = i;
        points[i].distance = INT_MAX; //set all the distances initially to INFINITY
        points[i].prev = NULL;
        points[i].inSPT = false;
    }
    points[source].distance = 0; //set the distance equal to 0 for the source
    
    for (int i = 0; i < NUM_NODES && points[dest].inSPT == false; i++){
        //take the shortest distance in the graph and update its neighbors, using the matrix
        int minPt = findShortestDistanceInGraph(points);
        //set as in inSPT
        points[minPt].inSPT = true;
        //update the neighbors
        //u is the min pt
        for (int j = 0; j < NUM_NODES; j++){
            if (points[j].inSPT == false && graph[minPt][j] && points[minPt].distance != INT_MAX && points[minPt].distance+ graph[minPt][j]< points[j].distance){
                //set the previous pointer
                points[j].prev = &points[minPt];
                points[j].distance = points[minPt].distance + graph[j][minPt];
            }
        }
    }
    //now we have the the shortest path and we can backtrace
    int j = dest;
    
    //I dont want to use STL like queue or vector, since I am running on Arduino
    int count = 0;
    while (j != source){
        count++;
        j = points[j].prev->pointID;
    }
    count ++; //to include the source
    //create an array of size count
    int nodes[count];
    char myConcatenation[80];
    
    //this function modifies the nodes so now nodes has src -> p1 -> ... -> dest
    getNumNodesToBeTraced(points, count, nodes, source, dest);
    for (int i = 0; i < count; i++){
        nodes[i] = nodes[i]+1;
        //fprintf(stdout, "%d,", nodes[i]);
    }
    //fprintf(stdout, "\n");
    
    if (aux == 0)
    {
        return count;
    }
    else
    {
        return nodes[position];
    }
    //when we update the u point (v --> u), then we update the previous points
}


//######################################################
 
#define NUM_SENSORS             6   // Numero de sensores que usa
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN             13  // emitter is controlled by digital pin 2

QTRSensorsRC qtrrc((unsigned char[]) {11, 18, 17, 16,15,14} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

// Definir pines motores
#define pwmi 6
#define mi1 7
#define mi2 5
#define pwmd 3
#define md1 4
#define md2 2

//definir otros pines

//#define pulsador 3             // pulsador de inicio 

// Definir constantes PID
int Vmax=86; 
float Kp=0.5; //0.18  // 0.5-0.6   //Kp 1
float Ki=0.001;
int Kd=20;   //4     //4       //4

const int margen=500;                // valor extremo para activar freno
const int Vfreno=-30;                // valor freno rueda interior para evitar salidas

// Define variables PID
int position=0;
int derivativo=0;
int proporcional=0;
int proporcional_old=0;
int integral=0;
int velocidad=0;

// Define variables boton
//boolean estado=HIGH;
//boolean estado_old=HIGH;
//boolean inicio=0;

// Define variables lectura BT
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void setup()
{
  //LEDS
  pinMode(LEDAL, OUTPUT);
  pinMode(LEDA, OUTPUT);
    
  //Ultrasonidos
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0

  
  //configura pines digitales
  pinMode(pwmi, OUTPUT);
  pinMode(mi1, OUTPUT);
  pinMode(mi2, OUTPUT);
  pinMode(pwmd, OUTPUT);
  pinMode(md1, OUTPUT);
  pinMode(md2, OUTPUT);
  //pinMode(pulsador, INPUT);           
  //delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

   // calibración
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  Serial.print("calibracion: ");

  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtrrc.calibratedMinimumOn[i];
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
    Serial.println();
    Serial.print("Vmax,Kp,Ki,Kd iniciales:");
    Serial.print(Vmax);
    Serial.print(",");
    Serial.print(Kp);
    Serial.print(",");
    Serial.print(Ki);
    Serial.print(",");
    Serial.println(Kd);
}


void loop() 
{
  //Bluetooth
  if(Serial.available() > 0){
             state = Serial.read();
             Serial.print(state);
             Serial.print("AQUIII");
        } 
       
      if (state == 90 && funcion == 0) { //Seguidor Z
          funcion = 1;
          state = 0;
        }
       if (state == 90 && funcion == 1)
       {
          funcion = 0;
          state = 0;
       }
       if (state == 89 && funcion == 0) { // Manual Y
          funcion = 2;
          Serial.print("HOLAAAA");
          state = 0;
        }
       if (state == 89 && funcion == 2)
       {
          funcion = 0;
          state = 0;
       }
       if (state == 88 && funcion == 0) { // Autonomo con lineas X
          funcion = 3;
          state = 0;
        }
       if (state == 88 && funcion == 3)
       {
          funcion = 0;
          state = 0;
       }
       if (state == 87 && funcion == 0) // Autonoma sin lineas W
       {
          funcion = 4;
          state = 0;
       }
       if (state == 87 && funcion == 4)
       {
          funcion = 0;
          state = 0;
       }
       if (state == 86 && funcion == 0) // Fiesta V
       {
          funcion = 5;
          state = 0;
       }
       if (state == 86 && funcion == 5)
       {
          funcion = 0;
          state = 0;
       }

       Serial.print(state);
       Serial.print(" state");
       Serial.println();
       Serial.print(funcion);
       Serial.print(" funcion");
       Serial.println();
       switch (funcion){
         case 1:
            //LED
            LED(cont_LED1);
            
            position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0); //0 para linea negra, 1 para linea blanca
            qtrrc.read(sensorValues);
            for (unsigned char i = 0; i < 6; i++)  //bucle para sacar datos en raw de sensores
            {
            Serial.print(sensorValues[i]);
            Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
            }
              para = ultrasonidos_Stop();
              // quitar para no reiniciar funcion a 0.
              if (para == 1){
                funcion = 0;
              }
              pid(sensorValues);
              //frenos();

              break;
           case 2:
           //LED
              LED(cont_LED2);
              if(state==119) // w
             {
                motores((velo-40), (velo+20)); //Mover_Adelante //derecha/izq
                
             }
             else if(state==115) //  s
             { 
                motores(-(velo-40), -(velo+20)); //Mover_Retroceso

             }
             else if(state==100) // d
             { 
                motores(-40, velo_giro); //Mover_Derecha

             }
             else if(state==97) // a
             { 
                motores(velo_giro, -40); //Mover_Izquierda();

             }
             else if(state==113) // q
             { 
               digitalWrite(md2,LOW);
               digitalWrite(md1, LOW);
               digitalWrite(mi2,LOW);
               digitalWrite(mi1, LOW);
             }
              break;
           case 3:
            LED(cont_LED1);
            if(state==65) 
            {
               Serial.print("UNOOOOOO");
               state = 0;
               fin = 9;
            }
             if(state==66) 
            {
               Serial.print("DOOOOOS");
               state = 0;
               fin = 0;
            }
             if(state==67) 
            {
               Serial.print("TREEEEE");
               state = 0;
               fin = 0;
            }
            lleva_obj(obj_encima);
            cont = algoritmo_autonomo( inicio, fin, 0, 0);
            camino[cont];
            if (fin != 0 && obj_encima == 1){
                // ALGORITMO -----
                if(hacer_alg == 0){
                  
                  for (int i = 0; i<cont-1; i++)
                  {
                    camino[i] = algoritmo_autonomo( inicio, fin, 1, i);
                    Serial.println(camino[i]);
                    delay(500);
                  }
                 
                  hacer_alg ++;
                }
                         Serial.print("\n");
                          Serial.print(cont);
                          Serial.print("\n");
                          Serial.print("\n");
                          Serial.print(n_intercepcion);
                          Serial.print("\n");
                          Serial.print("\n");
                          Serial.print(camino[n_intercepcion]);
                          Serial.print("\n");
                
                // ------
                if ((sensorValues[1] >= 2000) && (sensorValues[2] >= 2000) && (sensorValues[3] >= 2000) && (sensorValues[4] >= 2000)&& puedes == 6){
                  Serial.print("ENTRA INTERC");
                  digitalWrite(md2,LOW);
                  digitalWrite(md1, LOW);
                  digitalWrite(mi2,LOW);
                  digitalWrite(mi1, LOW);
                  if ((cont-1) == n_intercepcion){
                        digitalWrite(md2,LOW);
                        digitalWrite(md1, LOW);
                        digitalWrite(mi2,LOW);
                        digitalWrite(mi1, LOW);
                        funcion = 0;
                  }else{
                    switch (camino[n_intercepcion]){
                     case 0:
                        motores((velo-40), (velo+20)); //Adelante
                         Serial.print("\n");
                          Serial.print("RECTO");
                          Serial.print("\n");
                        delay(50);
                        digitalWrite(md2,LOW);
                        digitalWrite(md1, LOW);
                        digitalWrite(mi2,LOW);
                        digitalWrite(mi1, LOW);
                        break;
                     case 1:
                        motores(0, (velo_giro+20)); //Mover_Derecha
                          Serial.print("\n");
                          Serial.print("DERECHA");
                          Serial.print("\n");
                        delay(1200);
                        digitalWrite(md2,LOW);
                        digitalWrite(md1, LOW);
                        digitalWrite(mi2,LOW);
                        digitalWrite(mi1, LOW);
                        break;
                     case 2:
                        motores(velo_giro, 0); //Mover_Izquierda
                          Serial.print("\n");
                          Serial.print("IZQUIERDA");
                          Serial.print("\n");
                        delay(1200);
                        digitalWrite(md2,LOW);
                        digitalWrite(md1, LOW);
                        digitalWrite(mi2,LOW);
                        digitalWrite(mi1, LOW);
                        break;
                     default:
                        digitalWrite(md2,LOW);
                        digitalWrite(md1, LOW);
                        digitalWrite(mi2,LOW);
                        digitalWrite(mi1, LOW);
                        break; 
                  }
                  n_intercepcion += 1;
                  }
                  
                }
                if(puedes == 6){
                  puedes = 0;
                }
                puedes ++;
                position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0);  //0 para linea negra, 1 para linea blanca
                qtrrc.read(sensorValues);
                for (unsigned char i = 0; i < 6; i++)  //bucle para sacar datos en raw de sensores
                {
                Serial.print(sensorValues[i]);
                Serial.print('\t');  // tab to format the raw data into columns in the Serial monitor
                }
                  para = ultrasonidos_Stop();
                  // quitar para no reiniciar funcion a 0.
                  if (para == 1){
                    funcion = 0;
                  }
                  pid(sensorValues);
                  //frenos();
                  Serial.print("\n");
                  Serial.print("Numero intercepcion : ");
                  Serial.print(n_intercepcion);

            }
              break;
           case 4:
              break;
           case 5:
              if (fase_fiesta == 0){
                motores(-velo_giro, velo_giro); //Mover_Derecha
                
                fase_fiesta = 1;
              }else{
                fase_fiesta = 0;
                motores((velo_giro+70), -velo_giro); //Mover_Izquierda();
                
              }
              buzzer_fiesta();
              break;
              
           default:
               //Sonido stand by
               tono = 2;
               buzzer(tono, suena);
               //Reset seguidor de lineas
               digitalWrite(md2,LOW);
               digitalWrite(md1, LOW);
               digitalWrite(mi2,LOW);
               digitalWrite(mi1, LOW);
               int position=0;
               int derivativo=0;
               int proporcional=0;
               int proporcional_old=0;
               int integral=0;
               int velocidad=0;
               n_intercepcion =0;
               hacer_alg = 0;
               // Seg Lineas Hasta Aqui
               break;
               Serial.print("HOLAA");

       }
}

void lleva_obj(int &obj_encima){
  fotoresValor = analogRead(pinFotores);
  Serial.print(fotoresValor, DEC);
  if(fotoresValor > mitad){
      Serial.print("oscuro");
      obj_encima = 1;
  }else{
      Serial.print("luz");
  }
  Serial.println();
  //delay(5000);
}

void LED (int &cont_LED){
   

   if (suena == 500){
      digitalWrite(LED_BUILTIN, HIGH);  
      delay(500);                       
      digitalWrite(LED_BUILTIN, LOW);
      cont_LED = 0;
   }
   else{
     cont_LED += 1;
   }
}

void buzzer(int tono, int &suena){
   if (suena == 1400){
     tone(pinBuzzer, tonos[tono]);
     delay(100);
     noTone(pinBuzzer);
     suena = 0;
   }
   else{
     suena += 1;
   }
}

int ultrasonidos_Stop()
{
  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
  
  //Serial.print("Distancia: ");
  //Serial.print(d);      //Enviamos serialmente el valor de la distancia
  //Serial.print("cm");
  //Serial.println();
  delay(100);          //Hacemos una pausa de 100ms

  if (d < 10){
    return 1;
  }
  else{
    return 0;
  }
}

void pid(int sensorValues[6])
{
  proporcional = position - 2500;                                         //referencia a mitad de los sensores
  derivativo = (proporcional - proporcional_old);
  integral = (integral + proporcional_old);
  
  if (integral > 1000) integral = 1000;                                   //límites integral
  if (integral < -1000) integral = -1000;                                 //límites integral
  velocidad = (proporcional * Kp) + (derivativo * Kd) + (integral * Ki);
  if (velocidad > Vmax) velocidad = Vmax;                                 //limita a velocidad máxima
  if (velocidad < -Vmax) velocidad = -Vmax;                               //limita a velocidad máxima
  Serial.print("posicion: ");
  Serial.print(position);
  if (velocidad > 0)
  {
    motores(Vmax-(velocidad), (Vmax+25));                                        //gira a la derecha
  }
  if (velocidad < 0)
  {
    motores((Vmax), Vmax+velocidad);                                        //gira a la izquierda

  }
  proporcional_old = proporcional;
  
}

void motores(int motor_izq, int motor_der)
{
   Serial.print(" mot izq: ");
   Serial.print(motor_izq);
   Serial.print(" der: ");
   Serial.print(motor_der);
   Serial.println();
   
  if (motor_izq >= 0)
  {
    digitalWrite(mi1,HIGH);
    digitalWrite(mi2, LOW);


    analogWrite(pwmi, motor_izq);
  }
   if (motor_izq < 0)
   {
    digitalWrite(mi1,LOW);
    digitalWrite(mi2, HIGH);


    analogWrite(pwmi, motor_izq);
    }
 
  if (motor_der >= 0)
  {
    digitalWrite(md2,HIGH);
    digitalWrite(md1,LOW);

    analogWrite(pwmd, motor_der);
  }

   if (motor_der < 0)
  {
    digitalWrite(md2,LOW);
    digitalWrite(md1,HIGH);

    analogWrite(pwmd, motor_der);
  }
}

int algoritmo_autonomo(int inicio, int fin, int aux, int cont){
    
    int destino = 9;
    //0 represents that there is no edge between the nodes
    int graph[9][9] = {
        {0,4,2,0,0,0,0,0,0},
        {4,0,0,9,0,0,0,7,0},
        {2,0,0,6,4,0,0,0,0},
        {0,9,6,0,3,0,0,0,0},
        {0,0,4,0,0,3,0,0,0},
        {0,0,0,3,3,0,2,0,0},
        {0,0,0,0,0,2,0,0,1},
        {0,7,0,0,0,0,0,0,5},
        {0,0,0,0,0,0,1,5,0},
    };
    
    int size = shortestPath(graph, 0, destino-1, 2, 0, 0);
    int x[size];
    for (int i = 0; i < size; i++){
        x[i] = shortestPath(graph, 0, destino-1, 2, 1, i);
        fprintf(stdout, "%d,", x[i]);
    }
    fprintf(stdout, "\n");
    int camino[size-1];
    int i=0;
    while(i<size-1)
    {
        switch(x[i]){
            case 1: if(x[i+1] == 3){camino[i]=1;}
                    else{camino[i]=0;}
                    break;
            case 2: if(x[i+1]==8){camino[i]=0;}
                    else{camino[i]=1;}
                    break;
            case 3: if(x[i+1]==4){camino[i]=2;}
                    else{camino[i]=0;}
                    break;
            case 4: if(x[i+1]==2){camino[i]=2;}
                    else{camino[i]=1;}
                    break;
            case 5: if(x[i+1]==6){camino[i]=2;}
                    break;
            case 6: if(x[i+1]==4){camino[i]=1;}
                    else{camino[i]=0;}
                    break;
            case 7:if(x[i+1]==9){camino[i]=2;}
                    else{camino[i]=1;}
                    break;
            case 8:if(x[i+1]==2){camino[i]=2;}
                    else{camino[i]=1;}
                    break;
            case 9:if(x[i+1]==8){camino[i]=2;}
                   else{camino[i]=1;}
                    break;
        }
        i++;
        
    }
    
    printf("El camino es: \n");
    i=0;
    while(i<size-1)
    {
     
        printf("%d",camino[i]);
        i++;
    }

    if (aux == 0)
    {
      return size;  
    }
    else
    {
      return camino[cont];
    }
    
}

void buzzer_fiesta(){
    tone(tonePin, 184, 416.66625);
    delay(462.9625);
    delay(92.5925);
    tone(tonePin, 277, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 416.66625);
    delay(462.9625);
    delay(92.5925);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 246, 494.791171875);
    delay(549.76796875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(283.56453125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 277, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 155, 494.791171875);
    delay(549.76796875);
    delay(5.78703125);
    tone(tonePin, 138, 666.666);
    delay(740.74);
    delay(648.1475);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 277, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 246, 494.791171875);
    delay(549.76796875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 277, 494.791171875);
    delay(549.76796875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 494.791171875);
    delay(549.76796875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 207, 494.791171875);
    delay(549.76796875);
    delay(5.78703125);
    tone(tonePin, 155, 666.666);
    delay(740.74);
    delay(648.1475);
    tone(tonePin, 184, 369.791296875);
    delay(410.87921875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(283.56453125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 244.791421875);
    delay(271.99046875);
    delay(5.78703125);
    tone(tonePin, 233, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);
    tone(tonePin, 184, 119.791546875);
    delay(133.10171875);
    delay(283.56453125);
    tone(tonePin, 233, 119.791546875);
    delay(133.10171875);
    delay(5.78703125);


}
