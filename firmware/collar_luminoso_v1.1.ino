/*
#       _\|/_   A ver..., ¿que tenemos por aqui?
#       (O-O)        
# ---oOO-(_)-OOo---------------------------------
 
 
##########################################################
# ****************************************************** #
# *           DOMOTICA PARA PRINCIPIANTES              * #
# *     COLLAR LUMINOSO  PARA NORMA (CON MPU6050)      * #
# *          Autor:  Eulogio López Cayuela             * #
# *                                                    * #
# *       Versión  1.0       Fecha: 25/03/2018         * #
# *       Revisión 1.1       Fecha: 02/07/2021         * #
# ****************************************************** #
##########################################################
*/

#define _VERSION_ "INOPYA - Collar Norma v1.1  MPU6050,   revision: 02/07/2021"


#define DEBUG_MODE            false



/* 

 ===== NOTAS DE LA VERSION =====
 


 1 - Collar que cambia de color segun la actividad del perro. (sosegado, activo, saltando o parado).
	Funcion para monitorizar el estado de la bateria cada vez que se conecta y que en funcionamiento le permite ajustar el brillo y el patron luminoso para asi alargar la duracion de la bateria.
    el porcentaje de carga se mostrara con 10 de los LEDs, un led por cada 10% de carga restante
	Del 10 al 50% color azul. El 60,70 y 80 amarillos. El 90 y el 100 en verde (para facilitar la lectura).
	Optimizado el consumo sustituyendo casi todos los delay por SLEEP.
	Modo baliza que se activa tras 2 minutos de inactividad. Al no permanecer encendido si no en un parpadeose reduce 
	de forma sustancil el consumo, lo que facilita su localizacion (visual) si Norma pierde el collar en alguna de sus correrias y vuelve alguna mañana sin el, como ya ha sucedido, permitiendo encontrarlo la noche siguente.
	De dicho modo baliza/collar perdido se sale automaticamente si se detecta actividad continuada durante unos segundos.

  
  CONEXIONES:
  
 =======================
  ARDUINO     ADXL345/MPU6050  (acelerometro de tres ejes con sensibilidad entre +-2g y +-16g)
 =======================
  5V          5V
  ----        3.3v
  GND         GND
  ----        VS
  ----        CS

  SCL         SCL
  SDA         SDA
  GND         SDO
  PIN2        INT2   Conectado, pero sin usar por ahora
  ----        INT1 
  
 =======================
  ARDUINO     TIRA_LED
 =======================
  PIN9        DATOS PARA TIRA_LED

*/




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCIONPARA IMPORTACION DE LIBRERIAS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#include <Wire.h>                   // libreria para comunicaciones I2C
#include "I2Cdev.h"
#include "MPU6050.h"


#include <Adafruit_NeoPixel.h>      // Incluir biblioteca Adafruit NeoPixel
#include <LowPower.h>               // biblioteca para manejar el modo sleep




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        ALGUNAS DEFINICIONES PERSONALES PARA MI COMODIDAD AL ESCRIBIR CODIGO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define AND         &&
#define OR          ||
#define NOT          !
#define ANDbit       &
#define ORbit        |
#define XORbit       ^
#define NOTbit       ~

#define getBit(data,y)      ((data>>y) & 1)           // Obtener el valor  del bit (data.y)
#define setBit(data,y)      data |= (1 << y)          // Poner a 1 el bit (data.y) 
#define clearBit(data,y)    data &= ~(1 << y)         // Poner a 0 el bit (data.y)
#define togleBit(data,y)    data ^= (1 << y)          // Invertir el valor del bit (data.y)
#define togleByte(data)     data = ~data              // Invertir el valor del byte (data)

#define SERIAL_BEGIN        Serial.begin
#define PRINTLN             Serial.println
#define PRINT               Serial.print



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

/* VARIABLES Y CONSTANTES PARA EL MPU6050 */

bool FLAG_MPU6050_presente = false;      // ...en el setup detectaremos o no al MPU y su acelerometro
byte _buff[6];                           // buffer para datos
 
#define MPU6050_ADDR       0x68          // direccion i2c del MPU6050

/* Direcciones de algunos de los registros del MPU6050 */
#define GIRO_SENS           0x1B    //control de la sensibilidad del acelerometro
#define ACEL_SENS           0x1C    //control de la sensibilidad del giroscopio
#define WHO_AM_I            0x75    //almacena un ID conocido (0x34), Ayuda a saber si esta conectado
#define PWR_MGMT            0x6B

#define DATA_AX_H           0x3B    //Aceleracion eje X MSB
#define DATA_AX_L           0x3C    //Aceleracion eje X LSB
#define DATA_AY_H           0x3D    //Aceleracion eje Y MSB
#define DATA_AY_L           0x3E    //Aceleracion eje Y LSB
#define DATA_AZ_H           0x3F    //Aceleracion eje Z MSB
#define DATA_AZ_L           0x40    //Aceleracion eje Z LSB

//Temperature en ºC = (valor del registro con signo)/340.0 + 36.53
#define DATA_TEMP_H         0x41    //temperatura MSB
#define DATA_TEMP_L         0x42    //temperatura LSB

#define DATA_GX_H           0x43    //Giro eje X MSB
#define DATA_GX_L           0x44    //Giro eje X LSB
#define DATA_GY_H           0x45    //Giro eje Y MSB
#define DATA_GY_L           0x46    //Giro eje Y LSB
#define DATA_GZ_H           0x47    //Giro eje Z MSB
#define DATA_GZ_L           0x48    //Giro eje Z LSB

int contador_inactividad_total = 0;


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE ENUMERADORES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


/* ENUM NIVELES ACELERACION */
enum umbralesAceleracion 
{ 
  UMBRAL_ACEL_PARADA      =    0,  //     0
  UMBRAL_ACEL_NORMAL      =  250,  //   250
  UMBRAL_ACEL_CARRERA     = 3100,  //  2700-3200
  UMBRAL_ACEL_AZOGUE      = 4500,  //  4000-4500
};



/* ENUM NIVELES ACELERACION */
enum estadosMovimiento 
{ 
  ESTADO_PARADA      = 0,
  ESTADO_NORMAL      = 1,
  ESTADO_CARRERA     = 2,
  ESTADO_AZOGUE      = 3,
};


/* ENUM NIVELES DE BATERIA */
enum nivelesBateria 
{
  BATERIA_NIVEL_FULL      =  80,    // 
  BATERIA_NIVEL_MEDIO     =  15,    // por debajo de este porcentaje se considera nivel medio y se cambia de modo
  BATERIA_NIVEL_BAJO      =  10,    // por debajo, se considera nivel bajo y se cambia de modo     
  BATERIA_NIVEL_CRITICO   =   8,    // por debajo,se considera nivel critico y se cambia de modo    
};


/* ENUM NIVELES DE BRILLO LEDS */
enum nivelesBrillo 
{
  BRILLO_MODO_BATERIA_FULL       =  40,
  BRILLO_MODO_BATERIA_BIEN       =  20, 
  BRILLO_MODO_BATERIA_MEDIA      =  10,
  BRILLO_MODO_BATERIA_BAJA       =  10,
  BRILLO_MODO_BATERIA_CRITICA    =   9,
  BRILLO_MODO_COLLAR_PERDIDO     =  20,
};


/* ENUM MODOS DE OPERACION DEL COLALR */
enum modosOperacion 
{
  MODE_FULL       =  0,
  MODE_OK         =  1,
  MODE_MID        =  2,
  MODE_LOW        =  3,
  MODE_BAD        =  4,
  MODE_PERDIDO    =  5,
  MODE_ACEL_FAIL  =  6,
};

  
/* ENUM NIVELES DE BATERIA */
enum tiemposUtiles 
{
  TIEMPO_PARA_BALIZA          =  300,    //  segundos (aproximadamente 5 minutos, ya que se cuenta con periodos sleep) 
  TIEMPO_REVISION_BATERIA     = 1200,    //  1/4 segundos (aproximadamente 5 minutos, ya que se cuenta con periodos sleep) 
};







/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE CREACION DE OBJETOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

MPU6050 accelgyro;



/* Crear el 'objeto' para controlar los led */
#define PIN_TIRA_LED     9                    // patilla para la tira
#define TIPO_LED         NEO_GRB + NEO_KHZ800 // tipo controlador de los led
#define LONGITUD_TIRA    15                   // longitud real, por ahora

#define virtualGND      A6  //A6 nano version
#define virtualVCC      A7  //A7 nano version
Adafruit_NeoPixel tiraLEDS = Adafruit_NeoPixel(LONGITUD_TIRA, PIN_TIRA_LED, TIPO_LED);





/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   Prototipado de funciones (algunas lo necesitan para poder usar argumentos por defecto)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void init_MPU6050( void );

uint8_t control_de_movimientos( void );  //adaptada apra el nuevo acelerometro (MPU6050)
void writeTo(int device, byte address, byte val ); 
void readFrom(int device, byte address, int num, byte _buff[] );
void modo_bateria_bien( void );
void modo_bateria_media( void );
void modo_bateria_baja( void );
void modo_bateria_baja( void );
void mostrarColor( uint8_t red, uint8_t green, uint8_t blue );
void colorearLEDS( uint16_t inicio, uint16_t fin, uint8_t red, uint8_t green, uint8_t blue );
void colorearPIXEL( uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue );
void circulo( uint16_t framePausa, uint8_t red, uint8_t green, uint8_t blue );
void policia_sleep( uint8_t inicio, uint8_t fin );
void modo_collar_perdido( uint8_t first, uint8_t fin );
void flash_rojo( uint8_t inicio, uint8_t fin );
void flash_verde( uint8_t inicio, uint8_t fin );
void flash_azul( uint8_t inicio, uint8_t fin );
void arcoIRIS_movimiento( uint8_t pausa=8, uint8_t repeticiones=3 );  
uint32_t RuedaDeColorSimple( uint8_t colorPos );
void apagarTira( void );
uint8_t comprobar_estado_bateria( uint8_t modo=1 );  // 0 devuelve milivoltios, >0 devuelve porcentaje de carga
void mostrar_carga_bateria( void );



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
   ****************************************************************************************************** 
                                    FUNCION DE CONFIGURACION
   ****************************************************************************************************** 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void setup()  
{
  pinMode(virtualGND, INPUT_PULLUP);          //A6 gnd virtual
  //digitalWrite(virtualGND, LOW);
  pinMode(virtualVCC, INPUT_PULLUP);          //A7 vcc virtual
  //digitalWrite(virtualVCC, HIGH);

  Serial.begin(115200);           // Se inicia el puerto serie para depuracion
  
  delay(1000);
  accelgyro.initialize();
  
  pinMode(PIN_TIRA_LED, OUTPUT);  // pin para la linea DATA de los leds
  tiraLEDS.begin();               // Initializar el objeto 'tiraLEDS'
  
  pinMode(13, OUTPUT);            // led OnBoard de la placa Arduino Nano
  digitalWrite(13, LOW);
  
  Serial.println(F(_VERSION_));

 
  if(!DEBUG_MODE){
    Serial.flush();
    Serial.end(); 
    PRINTLN(F("DEBUG Esto no debe mostrarse!!"));
  }


    accelgyro.setXAccelOffset(-1313);
    accelgyro.setXAccelOffset(-37);
    accelgyro.setXAccelOffset(667);
    accelgyro.setXGyroOffset(113);
    accelgyro.setYGyroOffset(-48);
    accelgyro.setZGyroOffset(7);


  accelgyro.initialize();
  
  /* establecer la cantidad de brillo */
  tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_BIEN );


  /* Iniciar el MPU6050 conectado al bus i2c */
  init_MPU6050();

  
  /* TEST, probar el modo baliza, etc */
  //while(true){ modo_collar_perdido(0, LONGITUD_TIRA); } 
  //while(true){ control_de_movimientos_MPU6050(); delay (100); }
  //FLAG_MPU6050_presente = false;  para test simulando error del MPU

  /* Mostrar con leds iluminados el estado de la bateria. Cada LED es un 10% de carga restante */
  mostrar_carga_bateria();
  delay(7000);
  apagarTira();

  
  /* muestra de colores demo en el setup */
  arcoIRIS_movimiento(6,2);
  apagarTira();
  
  circulo(80, 255, 0, 255);  // pausa,r,g,b  (un pixel rotando por el collar)
  circulo(80, 255, 0, 255);  // pausa,r,g,b  (un pixel rotando por el collar)
  apagarTira(); 

  mostrarColor(255, 000, 000); //rojo tira completa (color para reposo)
  control_de_movimientos_MPU6050();
  delay(100);
  control_de_movimientos_MPU6050();

//  while(true){
//    control_de_movimientos_MPU6050();
//    delay(100);
//  }
 
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
   ****************************************************************************************************** 
                                  BUCLE PRINCIPAL DEL PROGRAMA
   ****************************************************************************************************** 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void loop()
{   
  uint8_t estado_bateria = comprobar_estado_bateria(); // 0 devuelve milivoltios, >0 porcentaje de carga (por defecto)
  uint8_t modo_actual = MODE_OK;

  if( estado_bateria >= BATERIA_NIVEL_FULL ) { modo_actual = MODE_FULL; }
  else if( estado_bateria >= BATERIA_NIVEL_MEDIO ) { modo_actual = MODE_OK; }
  else if( estado_bateria >= BATERIA_NIVEL_BAJO ) { modo_actual = MODE_MID; }
  else if( estado_bateria >= BATERIA_NIVEL_CRITICO ) { modo_actual = MODE_LOW; }  
  else{ modo_actual = MODE_BAD; }  

  if(FLAG_MPU6050_presente==false && modo_actual != MODE_BAD){ 
    modo_actual = MODE_ACEL_FAIL;
  }
  
  switch (modo_actual) {
    case MODE_FULL:
      tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_FULL ); 
      modo_bateria_bien();
      break;
      
    case MODE_OK:
      tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_BIEN );
      modo_bateria_bien();
      break;
      
    case MODE_MID:
      tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_MEDIA );
      modo_bateria_media();
      break;
      
    case MODE_LOW:
      tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_BAJA );
      modo_bateria_baja();
      
    case MODE_BAD:
      tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_CRITICA );
      modo_bateria_critica();
      break;
      
    case MODE_ACEL_FAIL:
      tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_BIEN );
      modo_acel_fail();
      break;
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx 
        BLOQUE DE FUNCIONES: LECTURAS DE SENSORES, COMUNICACION SERIE, INTERRUPCIONES...
   xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DEL MPU6050 SIN LIBRERIA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void init_MPU6050()
{
  /* Iniciar el MPU6050 conectado al bus i2c */
  Wire.begin();
  /* buscar el ID del MPU6050 para saber si esta conectado */  
  readFrom(MPU6050_ADDR, WHO_AM_I, 1, _buff);
  _buff[0] = _buff[0]>>1;
  _buff[0]&=B00111111;
  
  //Serial.print(F("ID acelerometro: "));Serial.println(_buff[0]);
  
  if( _buff[0]==0x34 ){ 
    FLAG_MPU6050_presente = true;
    PRINTLN(F("MPU6050 ok"));

    /* activar fuente de reloj */
    readFrom(MPU6050_ADDR, PWR_MGMT, 1, _buff);
    _buff[0]&=B11111000;
    _buff[0]|=B00000001;
    writeTo(MPU6050_ADDR, PWR_MGMT, _buff[0]); 
  
    /* establecer la sensibilidad del acelerometro bits 3 y 4 */  
    readFrom(MPU6050_ADDR, ACEL_SENS, 1, _buff);
    //_buff[0]&=~B00011000; //poner a cero los dos bits  usando bitmasking
    _buff[0]&= B11100111;   //poner a cero los dos bits en los que se guarda la sensibilidad del acelerometro
    _buff[0]|= B00000000;   //Poner sensibilidad en (Binario):  (00)+-2G, (01)+-4G,(10)+-8G, (11)+-16G,
    writeTo(MPU6050_ADDR, ACEL_SENS, _buff[0]); 

    /* establecer la sensibilidad del giroscopio bits 3 y 4 */
    readFrom(MPU6050_ADDR, GIRO_SENS, 1, _buff);
    //_buff[0]&=~B00011000; //poner a cero los dos bits  usando bitmasking
    _buff[0]&= B11100111;   //poner a cero los dos bits en los que se guarda la sensibilidad del giroscopio
    _buff[0]|= B00000000;   //Poner sensibilidad en:  (00)+-250%, (01)+-500%,(02)+-1000%, (03)+-2000%,
    writeTo(MPU6050_ADDR, GIRO_SENS, _buff[0]);
     
    /* Activar el MPU6050 forzando el modo sleep a false bit6=0 */
    readFrom(MPU6050_ADDR, PWR_MGMT, 1, _buff);
    _buff[0]&=B10111111;
    writeTo(MPU6050_ADDR, PWR_MGMT, _buff[0]); 

    /* leer la temperatura desde el MPU (chorrada) */ 
    readFrom(MPU6050_ADDR, DATA_TEMP_H, 2, _buff);
    float temperatura = _buff[0]| _buff[1];
    temperatura = temperatura/340.0 + 36.53;
    PRINT(F("Temperatura: ")); PRINTLN(temperatura-6);  //este sensor parece dar unos 6 grados de mas
  }
  else{
    PRINTLN(F("MPU6050 No presente!"));
  }  
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    LECTURA DE UN  ACELEROMETRO PARA EL CONTROL DE MOVIMIENTO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/



uint8_t control_de_movimientos_MPU6050()
{
  uint8_t FLAG_estado = 0;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  int16_t ax_scr, ay_scr, az_scr;
  int16_t gx_scr, gy_scr, gz_scr;
  
  static int16_t ax_old, ay_old, az_old;
  static int16_t gx_old, gy_old, gz_old;


  
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  
//  // display tab-separated accel/gyro x/y/z values
//  Serial.print("a/g:\t");
//  Serial.print(ax); Serial.print("\t");
//  Serial.print(ay); Serial.print("\t");
//  Serial.print(az); Serial.print("\t");  
//  Serial.print(gx); Serial.print("\t");
//  Serial.print(gy); Serial.print("\t");
//  Serial.println(gz);
       

  int8_t histeresis = 8;
  int8_t divisor = 15;
  ax_scr = (ax-ax_old)/divisor; if(abs(ax_scr)<histeresis){ ax_scr=0; } 
  ay_scr = (ay-ay_old)/divisor; if(abs(ay_scr)<histeresis){ ay_scr=0; } 
  az_scr = (az-az_old)/divisor; if(abs(az_scr)<histeresis){ az_scr=0; } 
  gx_scr = (gx-gx_old)/divisor; if(abs(gx_scr)<histeresis){ gx_scr=0; } 
  gy_scr = (gy-gy_old)/divisor; if(abs(gy_scr)<histeresis){ gy_scr=0; } 
  gz_scr = (gz-gz_old)/divisor; if(abs(gz_scr)<histeresis){ gz_scr=0; } 
  
  ax_old = ax, ay_old = ay, az_old = az;
  gx_old = gx, gy_old = gy, gz_old = gz;
  
  int32_t Aceleracion_MAX = abs(ax_scr) + abs(ay_scr) + abs(az_scr);
  
  /* clasificar el tipo de movimiento que se ha producido */
  if( Aceleracion_MAX >= UMBRAL_ACEL_AZOGUE ) { FLAG_estado = ESTADO_AZOGUE; }
  else if( Aceleracion_MAX >= UMBRAL_ACEL_CARRERA ) { FLAG_estado = ESTADO_CARRERA; }
  else if( Aceleracion_MAX >= UMBRAL_ACEL_NORMAL ) { FLAG_estado = ESTADO_NORMAL; }
  else { FLAG_estado = ESTADO_PARADA; }

  if(DEBUG_MODE){
    PRINT(Aceleracion_MAX); 
    if( FLAG_estado== UMBRAL_ACEL_AZOGUE ) { PRINTLN(F(" AZOGUE")); }
    else if( FLAG_estado== UMBRAL_ACEL_CARRERA ) { PRINTLN(F(" CARRERA")); }
    else if( FLAG_estado== UMBRAL_ACEL_NORMAL ) { PRINTLN(F(" NORMAL")); }
    else { PRINTLN(F(" STOP")); }  
  }
  return FLAG_estado;
}




//========================================================
//  Funcion auxiliar de escritura en el bus i2c
//========================================================

void writeTo(int device, byte address, byte val) 
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission(); 
}



//========================================================
//  Funcion auxiliar de lectura en el bus i2c
//========================================================

void readFrom(int device, byte address, int num, byte _buff[]) 
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();
 
  Wire.beginTransmission(device);
  Wire.requestFrom(device, num);
 
  int i = 0;
  while(Wire.available())
  { 
    _buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    MODOS DE FUNCIONAMIENTO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//   LUZ CON BATERIA FULL y OK (nivel alto, muy alto)
//========================================================

void modo_bateria_bien()
{ 
  uint16_t contador_reloj = 0;      // contabiliza los periodos en 'sleep' para tener una idea aproximada del tiempo
                                    // sumaremos una unidad por cada 250MS

  boolean FLAG_movimiento = true;   // controla si estamos en movimiento o no
  int contador_eventos = 0;         // contador_reloj de los cambios de estado movimiento/reposo
  int min_paradas = 3;              // (4) numero de detecciones de reposo necesarias para decicir que esta parado

  while (true){
    /* control periodico del estado de bateria (cada 5 minutos aprox.)*/
    if (contador_reloj >= TIEMPO_REVISION_BATERIA){
      contador_reloj = 0;
      int porcentage_carga = comprobar_estado_bateria(100);     //una LiPo al 100% tiene 4'2V, al 0% --> 3'2V
      if(porcentage_carga < BATERIA_NIVEL_MEDIO){
        for (int i=0; i< 8 ; i++){ policia_sleep(0, LONGITUD_TIRA); }
        break;  //entramos en nivel medio de bateria
      }
    }

    /* Control de movimientos */
    uint8_t estado_acelerometro = control_de_movimientos_MPU6050();
    
    if (estado_acelerometro != ESTADO_PARADA){
      contador_eventos -=1;
      if (contador_eventos <=min_paradas-2){
        FLAG_movimiento = true; //el perro se mueve
        contador_eventos = 0; 
        if(DEBUG_MODE){ Serial.println("movimiento"); }
      }      
    } 
    
    else{
      contador_eventos +=1;
      if (contador_eventos >= min_paradas){
        FLAG_movimiento = false; // perro se queda quieto
        contador_eventos =  min_paradas;
        if(DEBUG_MODE){ Serial.print("reposo"); }
      }
    } 

    /* color a mostrar en MOVIMIENTO */
    if (FLAG_movimiento == true){
      if (estado_acelerometro == ESTADO_NORMAL ){  // actividad normal
        mostrarColor(000, 255, 000); // verde
      }
      else if (estado_acelerometro == ESTADO_CARRERA ){  //bastante actividad
        mostrarColor(255, 000, 255);  //rosa
        LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
        contador_reloj+=8;
      }	  
      else if (estado_acelerometro == ESTADO_AZOGUE ){  //mucha, mucha actividad
        tiraLEDS.setBrightness( 2*BRILLO_MODO_BATERIA_BIEN );
        arcoIRIS_movimiento();  //mostrar alcoiris
        tiraLEDS.setBrightness( BRILLO_MODO_BATERIA_BIEN );
      }
    } 
    
    /* color a mostrar en PARADA y control de collar perdido */
    if (FLAG_movimiento == false){
      contador_inactividad_total++;
      if(contador_inactividad_total > TIEMPO_PARA_BALIZA){
        contador_inactividad_total=0;  
	      modo_collar_perdido(0, LONGITUD_TIRA);
        tiraLEDS.setBrightness(BRILLO_MODO_BATERIA_BIEN); //al salir del modo collar perdido, reestablecer el nivel de brillo
	    }
      mostrarColor(255, 000, 000); //(rojo) tira completa
      LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
      //delay(500);
      contador_reloj+=2;
    }
    else{ contador_inactividad_total = 0; }
    
    //unsigned long tiempo_fin_ciclo = millis();  //DEBUG
    //Serial.print("tiempo ciclo: ");Serial.println(tiempo_fin_ciclo-tiempo_inicio_ciclo);  //DEBUG
    
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    //delay(250);
    contador_reloj+=2; 
  }
}


//========================================================
//   LUZ DE POSICION CON BATERIA LOW (nivel medio-bajo)
//========================================================

void modo_bateria_media()
{
  /* mientras no se alcance el nivel bajo */
  while ( true ){
    mostrarColor(000, 240, 000);    // VERDE 

    /* dormimos el micro durante 16*8seg = 128 segundos */ 
    for (int i=0; i< 16 ; i++){ LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); }
    
    /* control periodico del estado de bateria */
    if(comprobar_estado_bateria() < BATERIA_NIVEL_BAJO){ 
      for (int i=0; i< 8 ; i++){ policia_sleep(0, LONGITUD_TIRA); }
      break;
    }    
  }
}



//========================================================
//   LUZ DE POSICION CON BATERIA NIVEL BAJO 
//========================================================

void modo_bateria_baja()
{ 
  boolean FLAG_voltaje_critico = false;   //para mantenernos en este modo si el voltaje es el adecuado
  tiraLEDS.setBrightness(BRILLO_MODO_BATERIA_BAJA); //establecemos el brillo en 10
  
  /* mientras no se alcance el nivel critico */
  while( true ){
    mostrarColor(000, 000, 240);  // azul 

    /* dormimos el micro durante 16*8seg = 256 segundos */ 
    for (int i=0; i< 32 ; i++){ LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); }
    
    /* control periodico del estado de bateria */
    if(comprobar_estado_bateria() < BATERIA_NIVEL_CRITICO){
      //for (int i=0; i< 8 ; i++){ policia_sleep(0, LONGITUD_TIRA); }
      break;
    }      
  }
}


//========================================================
//   LUZ DE POSICION CON BATERIA NIVEL CRITICO 
//========================================================
void modo_bateria_critica()
{
  /* se acabo lo que se daba, ¡Llegamos a nivel critico! */
  while(true){
    /* control del parpadeo del led */
    mostrarColor(255, 000, 000);    // rojo
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
    apagarTira();
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
  }
}



void modo_acel_fail()
{
  /* no hay acelerometro mostramos un color fijo y dormimos */
  mostrarColor(000, 000, 255);    // azul
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
}

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DE LA TIRA DE LEDS PARA CREAR LOS EFECTOS DE COLOR
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  MOSTRAR UN COLOR (tira completa)
//========================================================

void mostrarColor(uint8_t red, uint8_t green, uint8_t blue)
{
  for (int pixel=0; pixel< tiraLEDS.numPixels(); pixel++){
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(red, green, blue)); //color unico 
  } 
  tiraLEDS.show();
}



//========================================================
//   MOSTRAR UN COLOR EN UN RANGO DE LED
//========================================================

void colorearLEDS(int inicio, int fin, byte red, byte green, byte blue)
{ 
  
  for (int pixel = inicio; pixel < fin; pixel++){   
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(red, green, blue));
    //tiraLEDS.show();  //actualizar el nuevo estado de cada pixzel al modificarlo
  }
  tiraLEDS.show();  //actualizar 'en bloque' el nuevo estado de los leds que se han de modificar 
}



//========================================================
//   MOSTRAR UN COLOR EN UN LED UNICO
//========================================================

void colorearPIXEL(uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue)
{ 
  tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(red, green, blue));
  tiraLEDS.show();  //actualizar 'en bloque' el nuevo estado de los leds que se han de modificar 
}



//========================================================
//   circulo
//========================================================

void circulo(uint16_t framePausa, uint8_t red, uint8_t green, uint8_t blue) 
{
  delay(framePausa);
  int i=0;
    for (i=0; i< tiraLEDS.numPixels(); i++){
      //tiraLEDS.setPixelColor(i-1, tiraLEDS.Color(red-10, green-10, 0));
      tiraLEDS.setPixelColor(i, tiraLEDS.Color(red, green, blue));    
      tiraLEDS.setPixelColor(i-1, tiraLEDS.Color(0, 0, 0));
      tiraLEDS.show();
      delay(framePausa);    
    }
    tiraLEDS.setPixelColor(i-1, tiraLEDS.Color(0, 0, 0));
}



//========================================================
//     FUNCION PARA GENERAR UN PARPADEO TIPO SIRENA (MODO AHORRO)
//========================================================

void policia_sleep(uint8_t inicio, uint8_t fin)
{
  /* Muestra un parpadeo azul similar a una sirena de policia version mejorada (v2) */
  for (int pixel=inicio; pixel< fin; pixel++){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 180, 255));    // azul cielo
  }
  tiraLEDS.show();
  //delay (40);
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

  
  for (int pixel=inicio; pixel< fin; pixel++){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 000, 002));    // azul oscuro
  }
  tiraLEDS.show();
  //delay (90);
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);

  for (int pixel=inicio; pixel< fin; pixel++){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 000, 128));    // navy 128
  }
  tiraLEDS.show();
  //delay (45);
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
  for (int pixel=inicio; pixel< fin; pixel++){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 000, 000));    // apagar led
  }
  tiraLEDS.show();
  //delay (pausa_ok); // 430 OK
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
}


//========================================================
//  FUNCION PARA GENERAR UNA BALIZA SI EL COLLAR SE EXTRAVIA
//========================================================

void modo_collar_perdido(uint8_t first, uint8_t fin)
{
  uint8_t inicio=first;
  uint8_t contador_reloj_despertar = 0;
  
  tiraLEDS.setBrightness( BRILLO_MODO_COLLAR_PERDIDO );
  while(true){
    for(uint8_t i=0;i<5;i++){ flash_rojo(inicio, fin); }
    inicio++;
    if(inicio>1){ inicio=0; }
    
    for(uint8_t i=0;i<5;i++){ flash_verde(inicio, fin); }
    inicio++;
    if(inicio>1){ inicio=0; }
    
    for(uint8_t i=0;i<5;i++){ flash_azul(inicio, fin); }
    
    inicio++;
    if(inicio>1){ inicio=0; }
    if(control_de_movimientos_MPU6050()!=ESTADO_PARADA ){
      contador_reloj_despertar++;
	    if( contador_reloj_despertar>=2){ return; }
	  }	  
  }
}



//========================================================
//   FLASH ROJO
//========================================================

void flash_rojo(uint8_t inicio, uint8_t fin)
{
  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(200, 0, 0));    // azul cielo
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

  
  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(002, 000, 000));    // 
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);

  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(128, 000, 0));    // 
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
  apagarTira();
  LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF); 
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);  
}

//========================================================
//   FLASH VERDE
//========================================================

void flash_verde(uint8_t inicio, uint8_t fin)
{
  /* Muestra un parpadeo azul similar a una sirena de policia version mejorada (v2) */
  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 200, 000));    // azul cielo
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

  
  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 002, 000));    // 
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);

  for (int pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 90, 0));    // 128
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
  apagarTira();
  LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);  
}


//========================================================
//   FLASH AZUL
//========================================================

void flash_azul(uint8_t inicio, uint8_t fin)
{
  /* Muestra un parpadeo azul similar a una sirena de policia version mejorada (v2) */
  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 180, 255));    // azul cielo
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

  
  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 000, 002));    // 
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);

  for (uint8_t pixel=inicio; pixel< fin; pixel+=2){  
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(000, 000, 128));    // 
  }
  tiraLEDS.show();
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
  apagarTira();
  LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);  
}



//========================================================
//   CREAR UN AROIRIS EN MOVIMIENTO
//========================================================

void arcoIRIS_movimiento(uint8_t pausa, uint8_t repeticiones) 
{
  uint16_t i, j;

  for(j=0; j<256*repeticiones; j++) { // repeticiones, veces que se repiten los colores
    for(i=0; i< tiraLEDS.numPixels(); i++) {
      tiraLEDS.setPixelColor(i, RuedaDeColorSimple(((i * 256 / tiraLEDS.numPixels()) + j) & 255));
    }
    tiraLEDS.show();
    delay(pausa);
  }
}



//========================================================
//   FUNCION PARA GENERAR VARIACION DE COLOR (APOYO A LOS ARCOIRIS)
//========================================================

uint32_t RuedaDeColorSimple(uint8_t colorPos) 
{
/*
  Recibe un valor de  0 a 255 y lo convierte en un color RGB.
  Los colores generados son transiciones de rojo >> Verde >> azul y nuevamente al rojo.
  El parametro "intensidad" fija la luminosidad del color
*/
  colorPos = 255 - colorPos; //"colorPos": posicion dentro de la paleta rotativa R>>G>>B>>
  byte brillo = 1; //rango(0-1)ahora no se usa, pero lo mantengo por no cambiar las formulas de abajo
  if(colorPos < 85) {
    return tiraLEDS.Color(byte((255 - colorPos*3)*brillo), 0, byte(colorPos*3*brillo));
  }
  if(colorPos < 170) {
    colorPos -= 85;
    return tiraLEDS.Color(0, byte(colorPos*3*brillo), byte((255 - colorPos*3)*brillo));
  }
  colorPos -= 170;
  return tiraLEDS.Color(byte(colorPos*3*brillo), byte((255 - colorPos*3)*brillo), 0);
}



//========================================================
//   APAGAR LA TIRA DE LED POR COMPLETO
//========================================================

void apagarTira()
{ 
  /* 
    esta funcion se encarga de apagar la tira de led recorriendo 
    todos los pixeles que la forman y los pone a Cero en sus tres componentes
  */

  for (int pixel=0; pixel< 2 * LONGITUD_TIRA; pixel++){ 
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(0, 0, 0));
  } 
  tiraLEDS.show(); 
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//     CONTROL DEL ESTADO DE LA BATERIA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//   FUNCION PARA MONITORIZAR EL ESTADO DE LA BATERIA
//========================================================

uint8_t comprobar_estado_bateria(uint8_t modo) 
{
  /*  0 devuelve miliviltios, >0 devuelve porcentaje de carga  */
  /* leer la referecia interna de  1.1V  para calcular Vcc */
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2);                         // pausa para que Vref se estabilice
  ADCSRA |= _BV(ADSC);              // iniciar medicion
  while (bit_is_set(ADCSRA,ADSC));  // proceso de medicion propiamente dicho
 
  uint8_t low  = ADCL; // leer ADCL
  uint8_t high = ADCH; // leer ADCH
 
  long lecturaACD = (high<<8) | low;
 
  long milivoltios = 1125300L / lecturaACD;           // Calcular Vcc en mV (1125300 = 1.1*1023*1000)
  float voltaje = milivoltios/1000.0;                 // estado de la bateria en mV 
  float porcentage_carga = (100-(4.2-voltaje)*100);   // una LiPo al 100% tiene 4'2V, al 0% --> 3'2V
  
  if(porcentage_carga<0){ porcentage_carga = 0;}      // ojito la bateria estaria por debajo de 3'2 voltios
  if(porcentage_carga>100){ porcentage_carga = 100;}  // ojito la bateria estaria por encima de 4'2 voltios

  if(modo == 0){ return uint8_t(milivoltios); }           // Voltaje de la bateria en milivoltios
  if(modo > 0){ return uint8_t(porcentage_carga); }       // carga restante en porcentaje  
}



//========================================================
//   FUNCION PARA MONITORIZAR EL ESTADO DE LA BATERIA
//========================================================

void mostrar_carga_bateria()
{
  /* realiza una consulta sobre el estado de la bateria y colorea los led acorde con la garga restante*/
  //  0 devuelve miliviltios, >0 devuelve porcentaje de carga
  int porcentage_carga = comprobar_estado_bateria(1);     //una LiPo al 100% tiene 4'2V, al 0% --> 3'2V
  int carga = (porcentage_carga + 5)/10;                  //representa el numero de leds a encender, 
                                                          //Un led por cada 10% de carga
  apagarTira();

  //coloreado en azul entre el 10% y %carga
  colorearLEDS(0, carga, 0, 0, 255); // azul  un rango de leds (entre 0 y carga)


 if(carga>5){
  //sustituimos por naranja los porcentajes: entre 60% y el 80%
  colorearLEDS(5, carga, 155, 50, 0); // naranja
 }
  //(000, 255, 255);    // celeste/agua
  //(000, 180, 255);    // azul cielo

  
 if(carga>8){
  //sustituimos por verde los porcentajes: 90% y el 100%
  colorearLEDS(8, carga, 0, 255, 0);
 }

  if(carga<LONGITUD_TIRA){
    //apaga desde un led por encima de la cifra hasta la longitud de la tira, por si acaso
    colorearLEDS(carga+1, LONGITUD_TIRA, 0, 0, 0);        
  }
  
//  Serial.print("voltaje: "); Serial.println(voltaje);  
//  Serial.print("porcentage_carga: "); Serial.println(porcentage_carga);
//  Serial.print("carga: ");Serial.println(carga);
//  Serial.println((2*LONGITUD_TIRA-carga));
}


//***************************************************************************************************
//                    FIN DE PROGRAMA
//***************************************************************************************************
