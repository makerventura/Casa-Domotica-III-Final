/* Autor : Santiago Ventura Gomez . 2017
Curso Experto Universitario en Robótica , Programación e Impresión 3D
Asignatura : Robótica
Profesor : Alberto Valero
Actividad 2 : Construcción de una Casa Domótica Fase III FINAL . **** PROGRAMA DEFINITIVO ****
*/

/********************************************************* FASE I : CONTROL DE LA ILUMINACION ***************************************************************
/*
Objetivo : Primera Parte de la Construcción de una Casa Domótica . En esta fase pretendemos implementar un programa que encienda / apague las luces de la casa
de forma automática según la luz recibida por el sensor ldr , y/o de forma manual mediante la pulsación de un botón
*/
/* La casa dispone en esta primera fase , de la siguiente iluminación :

*  Dormitorio planta superior : led_cama ( PIN No. 12 ) , led_escritorio ( PIN No. 11 )
*  Modulo_Entrada , planta inferior : led_cocina ( PIN No. 10 )
*  Porche de la entrada , zona jardín : led_porche ( PIN No. 9 )
*/
/*
Descripción de la Máquina :

Estados :
* S0 : Luces Apagadas + DIA
* S1 : Luces Apagadas + NOCHE
* S2 : Luces Encendidas + DIA
* S3 : Luces Encendidas + NOCHE

Eventos  :
* E0 : Boton pulsar y soltar
* E1 : Nivel de Luz Ambiente >= 200
* E2 : Nivel de Luz Ambiente <  200
*/

/*  NOTAS A LA PROGRAMACION DE LA FASE I :

        Por la construcción de nuestro sistema , no podremos nunca mantener más de 24 horas seguidas
        encendidas las luces porque el sistema automático las apaga al llegar el día si no se pulsa
        el botón .También hay que tener en cuenta que en caso de ausencia prolongada de la casa , se
        producirían gastos innecesarios de luz
*/

/********************************************************* FASE II : CONTROL AUTOMATICO DE LA PUERTA DE GARAJE ************************************************
/*
Objetivo : Segunda Parte de la Construcción de una Casa Domótica . En esta fase pretendemos implementar un programa que controle la apertura y cierre de una
puerta de garaje en funcion de si el coche está fuera , dentro o pasando bajo la puerta
*/
/* Para dicho control , el garaje dispone de :

*  Servomotor 0º - 180º , para abrir y cerrar la puerta : ( PIN No. 6 )
*  Sensor IR en exterior puerta garaje :  ( PIN No. 8 )
*  Sensor IR en interior puerta garaje :  ( PIN No. 7 )
*  Luz en garaje : ( PIN No. 13 ) . Luz regulada junto con el mecanismo de control de la puerta del garaje , no con la iluminación general
*/
/*
Descripción de la Máquina de control del garaje :

Estados :
* S0 : Coche fuera , IRext=0 , IRint=0
* S1 : Coche fuera , IRext=1 , IRint=0
* S2 : Coche bajo puerta , IRext=1 , IRint=1
* S3 : Coche bajo puerta , IRext=0 , IRint=1
* S4 : Coche dentro , IRext=0 , IRint=0
Eventos  :
* E1 : IRext 1 -> 0
* E2 : IRext 0 -> 1
* E3 : IRint 1 -> 0
* E4 : IRint 0 -> 1
*/

/*  NOTAS A LA PROGRAMCION DE LA FASE II :

    1)    La luz del garaje queda regulada exclusivamente con el automatismo de la puerta , permaneciendo encendida mientras la puerta se encuentre abierta
          y apagada mientras la puerta esté cerrada.
    2)    Este programa , tal y como está ideado tiene al menos las siguientes limitaciones :
          *Si hay un coche dentro del garaje , NO permite la entrada de un segundo vehículo
          *Si el garaje está vacio , no me permite SALIR del garaje usando el sensor de deteccion
          *Cualquier cosa ( vehículo , persona , etc ) que intercepte el IR exterior hace que se abra la puerta del garaje .Problema de seguridad.
          Una posible mejora de este sistema de apertura podría ser la inclusion del sensor de luz en combinacion con el IR , de manera que el
          evento inicial de apertura del garaje fuese IRext=1 && 3 destellos seguidos en menos de 5 segundos en el sensor de luz ( luz >500 )
          0->1->0->1->0->1 ( tiempo < 5000 )
*/

/*********************************************************      FASE III : SISTEMA DE ALARMA       ************************************************
/*
Objetivo : En esta tercera fase , se pretende implementar un sistema de alarma que detecte presencia en el exterior de la casa . Detectada presencia fuera ,
se encienden todas las luces de la casa y comienza una cuenta atras de 10 segundos para deconectarla mediante el pulsador que controla manualmente la iluminacion
de la casa . Si pasado ese tiempo no se ha pulsado el botón , todos los sistemas de la casa se bloquean y el zumbador empezará a sonar indefinidamente .
Nota : En mi programa , la única manera de sacar al sistema del salto de alarma es mediante un RESET general

Para dicho sistema de alarma , la casa cuenta con :

* Emisor de luz laser actuado manualmente y no controlado por el microcontrolador
* Sensor LDR de luz : ( PIN no. A1 )
* Zumbador : ( PIN no. 4 )

La detección de presencia exterior se realiza rodeando todo el perímetro de la cas mediante un haz de luz laser , que saliendo del emisor laser ,
rebota sucesivamente en tres espejos colocados en las esquinas del perímetro , e incide en el sensor LDR que cierra el rectángulo .
El haz laser estará continuamente incidiendo sobre el sensor LDR a través de los espejos, y cuando algo corte dicha emisión se entenderá
que es una presencia que entra en la zona de la casa .
*/
/* Descripcion de la máquina de estados que gobierna el sistema de alarma :

Estados :

*  S0  :  ALARMA ON , INTRUSO FALSE
*  S1  :  ALARMA ON , INTRUSO TRUE
*  S2  :  SALTO DE ALARMA , CASA BLOQUEADA
*  S3  :  ALARMA OFF , CASA FUNCIONAMIENTO NORMAL

Eventos :

*  E0  :  LDR_ALARMA  <  200
*  E1  :  BOTON PULSADO && Tiempo deteccion presencia < 10 segundos
*  E2  :  Tiempo deteccion presencia > 10 segundos
*/
/* NOTAS A LA FASE III :

    1) Esta actividad admite muchas mejoras , y entre ellas anexar maneras de sacar al sistema del bloqueo post salto de alarma , y para poner en practica
    lo estudiado en App Inventor , le añadiré una pequeña aplicación para conectar-desconectar el sistema de alarma desde el móvil .
    En una segunda fase , esa misma aplicación podría servir para encender-apagar la iluminacion de la casa , así como para abrir-cerrar la puerta del garaje

*/


/********************************* DEFINICION DE LIBRERIAS Y VARIABLES GLOBALES GARAJE **********************************************/

#include <Servo.h>              //Libreria Arduino que controla funciones especiales de los servomotores
Servo servoGaraje;              //Declaracion de variable tipo Servo

int estado_garaje = 0;          // Variable que guarda el estado de control de la puerta del garaje y el vehiculo
int angulo_puerta = 45;         // Variable que guarda el angulo del servo de la puerta en cada momento. Inicia con puerta cerrada
const int led_garaje = 13;      //Pin al que conectamos el led del garaje
const int IR_exterior = 8;      //Pin al que conectamos el sensor IR situado en el exterior del garaje
const int IR_interior = 7;      //Pin al que conectamos el sensor IR situado en el interior del garaje



/********************************** DEFINICION DE VARIABLES GLOBALES ILUMINACION **********************************************/

int estado_iluminacion = 0;      // Variable que guarda el estado de la iluminacion de la casa salvo luz garaje
const int led_cama = 12;         //Pin al que conectamos el Led del dormitorio , zona cama
const int led_escritorio = 11;   //Pin al que conectamos el Led del dormitorio , zona escritorio
const int led_cocina = 10;       //Pin al que conectamos el led de la entrada , zona cocina
const int led_porche = 9;        //Pin al que conectamos el led del jardin , zona porche
const int led_salon = 5;         // Pin al que conectamos el led del salón
const int boton = 2;             //Pin al que conectamos el botón pulsador que controla el encendido-apagado de los leds
const int ldr_luces = A0;        // Pin analógico al que conectamos el sensor de luz que controla la iluminacion exterior ambiente


/********************************** DEFINICION DE VARIABLES GLOBALES SISTEMA ALARMA **********************************************/

int estado_alarma = 0;                           // Variable que guarda el estado del sistema de alarma de la casa
const int pin_zumbador = 4;                      //Pin al que conectamos el zumbador de la alarma
const int ldr_alarma = A1;                       //Pin al que conectamos el sensor de luz de la alarma
long int t_inicial = 0;                          // Variable que guarda el momento en que comenzamos a contar los 10 segundos de salto de alarma
long int t_actual = 0;                           // Variable que guarda el valor del tiempo en cada consulta al reloj interno , una vez iniciados los 10 segundos
long int t_transcurrido = t_actual - t_inicial;  //Variable que guarda el tiempo transcurrido desde que detectamos la presencia exterior



/***************************  DECLARACION DE FUNCIONES DEL SISTEMA DE CONTROL DE LA ILUMINACION *************************************/

// controlIluminacion enciende o apaga las luces de la casa dependiendo del valor del parametro valor_ilum :

void controlIluminacion(int valor_ilum, int luz_cama, int luz_escritorio, int luz_cocina, int luz_porche, int luz_salon) {

  switch (valor_ilum) {
    case 0:
      digitalWrite(luz_cama, LOW);          // Apagamos la luz de la cama
      digitalWrite(luz_escritorio, LOW);    // Apagamos la luz del escritorio
      digitalWrite(luz_cocina, LOW);        // Apagamos la luz de la cocina
      digitalWrite(luz_porche, LOW);        // Apagamos la luz del porche
      digitalWrite(luz_salon, LOW);         // Apagamos la luz del salon
      break;
    case 1:
      digitalWrite(luz_cama, LOW);          // Apagamos la luz de la cama
      digitalWrite(luz_escritorio, LOW);    // Apagamos la luz del escritorio
      digitalWrite(luz_cocina, LOW);        // Apagamos la luz de la cocina
      digitalWrite(luz_porche, LOW);        // Apagamos la luz del porche
      digitalWrite(luz_salon, LOW);         // Apagamos la luz del salon
      break;
    case 2:
      digitalWrite(luz_cama, HIGH);          // Encendemos la luz de la cama
      digitalWrite(luz_escritorio, HIGH);    // Encendemos la luz del escritorio
      digitalWrite(luz_cocina, HIGH);        // Encendemos la luz de la cocina
      digitalWrite(luz_porche, HIGH);        // Encendemos la luz del porche
      digitalWrite(luz_salon, HIGH);         // Encendemos la luz del salon
      break;
    case 3:
      digitalWrite(luz_cama, HIGH);          // Encendemos la luz de la cama
      digitalWrite(luz_escritorio, HIGH);    // Encendemos la luz del escritorio
      digitalWrite(luz_cocina, HIGH);        // Encendemos la luz de la cocina
      digitalWrite(luz_porche, HIGH);        // Encendemos la luz del porche
      digitalWrite(luz_salon, HIGH);         // Encendemos la luz del salon
      break;
  }
}

/* logicaIluminacion es la funcion que controla la logica de estados de la programacion de la Iluminacion de la casa.
   Dependiendo del valor del parámetro que le enviemos de valor_ilum , y las lecturas que realice en pulsador y sensor_luz
   en un momento dado,nos retorna un nuevo valor del estado de la iluminacion de la casa */

int logicaIluminacion(int valor_ilum, int pulsador, int sensor_luz) {

  if (valor_ilum == 0 && digitalRead(pulsador) == 1) {              // Si valor_ilum = 0 y pulsador presionado ...
    while (digitalRead(pulsador) == 1) {                          // Bucle para evitar efecto rebote del pulsador
      delay(10);
    }
    return 2;                                                     // Devuelve estado logico S2
  }
  else if ( valor_ilum == 0 && analogRead(sensor_luz) < 200) {      // Si valor_ilum = 0 y se hace de noche ....
    return 3;                                                     // Devuelve estado logico S3
  }
  else if (valor_ilum == 1 && digitalRead(pulsador) == 1) {         // Si valor_ilum = 1 y pulsador presionado ...
    while (digitalRead(pulsador) == 1) {                          // Bucle para evitar efecto rebote del pulsador
      delay(10);
    }
    return 3;                                                     // Devuelve estado logico S3
  }
  else if (valor_ilum == 1 && analogRead(sensor_luz) >= 200) {      // Si valor_ilum = 1 y se hace de dia ...
    return 0;                                                     // Devuelve estado logico S0
  }
  else if (valor_ilum == 2 && digitalRead(pulsador) == 1) {         // Si valor_ilum = 2 y pulsador presionado ...
    while (digitalRead(pulsador) == 1) {                          // Bucle para evitar efecto rebote del pulsador
      delay(10);
    }
    return 0;                                                     // Devuelve estado logico S0
  }
  else if ( valor_ilum == 2 && analogRead(sensor_luz) < 200) {      // Si valor_ilum = 2 y se hace de noche ...
    return 3;                                                     // Devuelve estado logico S3
  }
  else if (valor_ilum == 3 && digitalRead(pulsador) == 1) {         // Si valor_ilum = 3 y pulsador presionado ...
    while (digitalRead(pulsador) == 1) {                          // Bucle para evitar efecto rebote del pulsador
      delay(10);
    }
    return 1;                                                     // Devuelve estado logico S1
  }
  else if (valor_ilum == 3 && analogRead(sensor_luz) >= 200) {      // Si valor_ilum = 3 y se hace de día ...
    return 0;                                                     // Devuelve estado logico S0
  }
  else {
    return valor_ilum;                                            /* En caso de no coincidir con ninguna situacion anterior ,
                                                                        devuelve el mismo valor de estado logico de entrada */
  }
}

/***************************    DECLARACION DE FUNCIONES DEL SISTEMA DE CONTROL DEL GARAJE     *************************************/


/* moverPuerta abre o cierra el objeto motor , en aproximadamente 4.5 seg, en funcion de la posicion anterior a ser llamada y de la
  posicion final a la que queremos llevarla a la salida de la misma . Retorna el valor del angulo del servomotor motor al acabar el movimiento */

int moverPuerta(Servo motor, int posicion_final, int ultima_posicion) {

  if (posicion_final > ultima_posicion) {             // Si Angulo final > Angulo actual , el servo incrementa 1 grado su angulo cada 50 mS hasta alcanzar Angulo final
    while (posicion_final > ultima_posicion) {
      motor.write(ultima_posicion);
      delay(50);
      ultima_posicion ++;
    }
    return ultima_posicion;                         // Devuelve la posicion del servo al terminar
  }
  else if (posicion_final < ultima_posicion) {        // Si Angulo final < Angulo actual , el servo decrementa 1 grado su angulo cada 50 mS hasta alcanzar Angulo final
    while (posicion_final < ultima_posicion) {
      motor.write(ultima_posicion);
      delay(50);
      ultima_posicion --;
    }
    return ultima_posicion;                         // Devuelve la posicion del servo al terminar
  }
  else {                                              // Si fuesen iguales , el servo no se mueve y y la funcion devuelve el mismo angulo que recibió
    return ultima_posicion;
  }
}
/* controlGaraje abre o cierra la puerta del garaje , enciende o apaga la luz del garaje ,
en funcion del valor que tome el parametro valor_garaje. Retorna el valor actual del angulo de la puerta */

int controlGaraje(Servo servo, int angPuerta, int valor_garaje, int luz_garaje) {

  switch (valor_garaje) {

    case 0: // Coche fuera , IRext=0 , IRint=0
      digitalWrite(luz_garaje, LOW);                   // Apagamos la luz del garaje
      angPuerta = moverPuerta(servo, 60, angPuerta);   // Angulo del servo correspondiente a PUERTA CERRADA
      return angPuerta;                                // Retorna valor actual de la posicion de la puerta
      break;
    case 1: // Coche fuera , IRext=1 , IRint=0
      digitalWrite(luz_garaje, HIGH);                  // Encendemos la luz del garaje
      angPuerta = moverPuerta(servo, 130, angPuerta);  // Angulo del servo correspondiente a PUERTA ABIERTA
      return angPuerta;                                // Retorna valor actual de la posicion de la puerta
      break;
    case 2: // Coche bajo puerta , IRext=1 , IRint=1
      digitalWrite(luz_garaje, HIGH);                  // Encendemos la luz del garaje
      angPuerta = moverPuerta(servo, 130, angPuerta);  // Angulo del servo correspondiente a PUERTA ABIERTA
      return angPuerta;                                // Retorna valor actual de la posicion de la puerta
      break;
    case 3: // Coche dentro , IRext=0 , IRint=1
      digitalWrite(luz_garaje, HIGH);                  // Encendemos la luz del garaje
      angPuerta = moverPuerta(servo, 130, angPuerta);  // Angulo del servo correspondiente a PUERTA ABIERTA
      return angPuerta;                                // Retorna valor actual de la posicion de la puerta
      break;
    case 4: // Coche dentro , IRext=0 , IRint=0
      digitalWrite(luz_garaje, LOW);                   // Apagamos la luz del garaje
      angPuerta = moverPuerta(servo, 60, angPuerta);   // Angulo del servo correspondiente a PUERTA CERRADA
      return angPuerta;                                // Retorna valor actual de la posicion de la puerta
      break;
  }
}

/* logicaGaraje es la funcion que controla la logica de estados de la programacion del Garaje.
   Dependiendo del valor del parámetro que le enviemos de valor_garaje , y las lecturas que realice en ext_IR , int_IR
   en un momento dado,nos retorna un nuevo valor de la logica de estados del Garaje */

int logicaGaraje(int valor_garaje, int ext_IR, int int_IR) {

  if (valor_garaje == 0 && digitalRead(ext_IR) == 1) {
    return 1;             // Coche fuera se acerca a la puerta , marcando sobre IR exterior
  }
  else if (valor_garaje == 1 && digitalRead(ext_IR) == 0) {
    return 0;             // Coche fuera , se aleja del garaje y deja de marcar sobre IR exterior
  }
  else if (valor_garaje == 1 && digitalRead(int_IR) == 1) {
    return 2;             // Coche cruzando puerta garaje , y marcando ambos sensores IR
  }
  else if (valor_garaje == 2 && digitalRead(int_IR) == 0) {
    return 1;             // Coche cruzando puerta garaje , sale hacia fuera y deja de marcar IR interior
  }
  else if (valor_garaje == 2 && digitalRead(ext_IR) == 0) {
    return 3;             // Coche dentro del garaje pero marcando sobre el IR interior
  }
  else if (valor_garaje == 3 && digitalRead(ext_IR) == 1) {
    return 2;             // Coche cruzando puerta garaje , y marcando ambos sensores IR
  }
  else if (valor_garaje == 3 && digitalRead(int_IR) == 0) {
    return 4;             // Coche completamente dentro del garaje
  }
  else if (valor_garaje == 4 && digitalRead(int_IR) == 1) {
    return 3;             // Coche dentro del garaje pero marcando sobre el IR interior
  }
  else {
    return valor_garaje; /* En caso de no coincidir con ninguna situacion anterior , devuelve el mismo valor
                               de estado logico de entrada */
  }
}

/***************************    DECLARACION DE FUNCIONES DEL SISTEMA DE ALARMA     *************************************/

/*La funcion logicaAlarma controla el estado en el que se encuentra el sistema de alarma en funcion del estado previo del sistema y las lecturas 
del sensor de luz laser , el pulsador y el tiempo transcurrido desde la deteccion de presencia*/

int logicaAlarma(int estAlarma, int ldrAlarma, int pulsador, int incr_t) {

  if (estAlarma == 0 && analogRead(ldrAlarma) < 200) {                          // Si alarma conectada y deteccion de presencia de intruso
    t_inicial = millis();                                                       // Iniciamos la cuenta de los 10 segundos
    return 1;                                                                   // devolvemos estado = 1 , prealerta > contamos 10 segundos
  }
  else if (estAlarma == 1 && incr_t > 10000) {                                  // Una vez detectado intruso , si tiempo contabilizado > 10 seg
    return 2;                                                                   // devolvemos estado = 2 , casa totalmente bloqueada
  }
  else if (estAlarma == 1 && incr_t <= 10000 && digitalRead(pulsador) == 1) {   // Una vez detectado intruso , si se pula boton antes de contar 10 segundos
    while (digitalRead(pulsador) == 1) {                                        // devolvemos estado = 3 , casa en funcionamiento normal
      delay(10);
    }
    return 3;
  }
  else  {
    return estAlarma;                                                            // En caso contrario devolvemos mismo estado de entrada
  }
}

/* La funcion controlAlarma regula las acciones que realiza el sistema de alarma en funcion del estado del mismo . 
Devuelve el tiempo transcurrido desde la deteccion de intruso*/

long int controlAlarma(int estAlarma, int luz_cama, int luz_escritorio, int luz_cocina, int luz_porche, int luz_salon, int luz_garaje, int zumbador, Servo servo, long int T0, long int Tf, long int Tdif) {

  switch (estAlarma) {

    case 0:                                  // Alarma conectada , intruso no detectado
      noTone(zumbador);                      // Zumbador apagado
      return Tdif;
      break;
    case 1:                                  // Alarma conectada , intruso detectado , cuenta 10 seg , enciende todas las luces de casa
      Tf = millis();                         // Guarda el valor actual del cronometro
      Tdif = Tf - T0;                        // Calcula la diferencia con el valor guardado al principio del control
      digitalWrite(luz_cama, HIGH);          // Encendemos la luz de la cama
      digitalWrite(luz_escritorio, HIGH);    // Encendemos la luz del escritorio
      digitalWrite(luz_cocina, HIGH);        // Encendemos la luz de la cocina
      digitalWrite(luz_porche, HIGH);        // Encendemos la luz del porche
      digitalWrite(luz_salon, HIGH);         // Encendemos la luz del salon
      digitalWrite(luz_garaje, HIGH);        // Encendemos la luz del garaje
      return Tdif;
      break;
    case 2:                                 // Tiempo transcurrido desde deteccion de intruso mayor que 10 seg , casa bloqueada , alarma suena indefinidamente
      tone(zumbador, 261, 0);               // Sonido de la alarma por tiempo indefinido
      delay(0);
      digitalWrite(luz_cama, LOW);          // Apagamos la luz de la cama
      digitalWrite(luz_escritorio, LOW);    // Apagamos la luz del escritorio
      digitalWrite(luz_cocina, LOW);        // Apagamos la luz de la cocina
      digitalWrite(luz_porche, LOW);        // Apagamos la luz del porche
      digitalWrite(luz_salon, LOW);         // Apagamos la luz del salon
      digitalWrite(luz_garaje, LOW);        // Apagamos la luz del garaje
      servo.detach();                       // Desconectamos el servomotor del pin
      while (1) {
        delay(10);                          // Bucle infinito de bloqueo de la casa
                                            // Solamente sale de aqui con un RESET
      }
      return Tdif;
      break;
    case 3:
      noTone(zumbador);                      // Zumbador apagado
      return Tdif;
      break;
  }
}


void setup() {

  /******************************** DECLARACION DE PINES Y ESTADO INICIAL RELATIVOS AL CONTROL DE LA ILUMINACION **********************/

  pinMode(led_cama, OUTPUT);         // Declaramos led_cama como salida
  pinMode(led_escritorio, OUTPUT);   // Declaramos led_escritorio como salida
  pinMode(led_cocina, OUTPUT);       // Declaramos led_cocina como salida
  pinMode(led_porche, OUTPUT);       // Declaramos led_porche como salida
  pinMode(led_salon, OUTPUT);        // Declaramos led_salon como salida
  pinMode(boton, INPUT);             // Declaramos boton como entrada
  pinMode(ldr_luces, INPUT);         // Declaramos ldr como entrada

  /*
  Iniciamos la iluminacion de la casa en estado S0 : Todos los leds apagados
  */
  digitalWrite(led_cama, LOW);           // Apagamos la luz de la cama
  digitalWrite(led_escritorio, LOW);     // Apagamos la luz del escritorio
  digitalWrite(led_cocina, LOW);         // Apagamos la luz de la cocina
  digitalWrite(led_porche, LOW);         // Apagamos la luz del porche
  digitalWrite(led_salon, LOW);          // Apagamos la luz del salon

  /*********************************DECLARACION DE PINES Y ESTADO INICIAL RELATIVOS AL CONTROL DEL GARAJE *****************************/

  servoGaraje.attach(6);              // Pin en el que se encuentra conectado el servomotor

  pinMode(led_garaje, OUTPUT);       //  Declaramos led_garaje como salida
  pinMode(IR_exterior, INPUT);       //  Declaramos IR_exterior como entrada
  pinMode(IR_interior, INPUT);       //  Declaramos IR_interior como entrada

  /*
  Iniciamos el control del garaje en estado S0 : Puerta cerrada y led apagado
  */
  digitalWrite(led_garaje, LOW);     // Apagamos la luz del garaje
  servoGaraje.write(45);             // Angulo del servo correspondiente a PUERTA CERRADA

  /*********************************DECLARACION DE PINES Y ESTADO INICIAL RELATIVOS AL CONTROL DE LA ALARMA *****************************/

  pinMode(ldr_alarma, INPUT);         // Declaramos el sensoer de luz de la alarma como entrada
  pinMode(pin_zumbador, OUTPUT);      // Declaramos el pin al que conectamos el zumbador como salida
  noTone(pin_zumbador);               // Inicializamos el zumbador apagándolo

}



void loop() {


  /**********************************************************************************************************************************/
  /********************************************   CONTROL DE LA ILUMINACION DE LA CASA DOMOTICA  ************************************/
  /**********************************************************************************************************************************/


  estado_iluminacion = logicaIluminacion(estado_iluminacion, boton, ldr_luces);                         /* Gestion de la logica de control de la iluminacion
                                                                                                    de la casa mediante la funcion logicaIluminacion */

  controlIluminacion(estado_iluminacion, led_cama, led_escritorio, led_cocina, led_porche, led_salon);     /* Gestion de las acciones a realizar con la iluminacion
                                                                                                    de la casa con la funcion controlIluminacion */


  /****************************************         FIN PROGRAMA CONTROL ILUMINACION CASA           ********************************/


  /**********************************************************************************************************************************/
  /****************************************   CONTROL DE LA PUERTA DEL GARAJE  DE LA CASA DOMOTICA  *********************************/
  /**********************************************************************************************************************************/


  estado_garaje = logicaGaraje(estado_garaje, IR_exterior, IR_interior);                                /* Gestion de la logica de control del Garaje mediante la
                                                                                                    funcion logicaGaraje */

  angulo_puerta = controlGaraje(servoGaraje, angulo_puerta, estado_garaje, led_garaje);                  /* Gestion de las acciones en el Garaje mediante la
                                                                                                    funcion controlGaraje */


  /****************************************         FIN PROGRAMA CONTROL PUERTA DEL GARAJE           ********************************/


  /**********************************************************************************************************************************/
  /****************************************   CONTROL DEL SISTEMA DE ALARMA  DE LA CASA DOMOTICA   **********************************/
  /**********************************************************************************************************************************/


  estado_alarma = logicaAlarma(estado_alarma, ldr_alarma, boton, t_transcurrido);                          /* Gestion de la lógica de control del sistema de Alarma
                                                                                                           mediante la función logicaAlarma*/

  t_transcurrido = controlAlarma(estado_alarma,led_cama,led_escritorio,led_cocina,led_porche,led_salon,led_garaje,pin_zumbador,servoGaraje,t_inicial,t_actual,t_transcurrido);
                                                                                                           /* Gestion de las acciones de control del sistema de Alarma
                                                                                                           mediante la funcion controlAlarma */
                                                                                                           

  /****************************************       FIN PROGRAMA CONTROL DEL SISTEMA DE ALARMA       **********************************/


  /****************************************                       FIN DEL PROGRAMA                   ********************************/

}
