/* Desarrollado por Juan Blanc */
/****  Noviembre de 2018   ****/

/* Aqui unos caracteres especiales para la pantalla */
byte kora[8] = {
  0b00000,
  0b01010,
  0b10101,
  0b10101,
  0b10001,
  0b01010,
  0b00100,
  0b00000
};
byte hdown[8] = {
  0b00100,
  0b01110,
  0b01010,
  0b00100,
  0b01110,
  0b10101,
  0b01010,
  0b01010
};
byte hup[8] = {
  0b00100,
  0b01110,
  0b01010,
  0b10101,
  0b01110,
  0b00100,
  0b01010,
  0b01010
};
byte quiz[8] = {
  0b00000,
  0b01110,
  0b11011,
  0b10001,
  0b00010,
  0b00100,
  0b00000,
  0b00100
};

// Librerías requeridas
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Screen
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
/* NO te olvides de cambiar esto por si tenes múltiples pantallaassss */
#define direccionLCD 0x27

LiquidCrystal_I2C lcd(direccionLCD,20,4);

// Parametros wifi
#define WLAN_SSID       "nombreDeTuWifi"
#define WLAN_PASS       "TuContraseña"


// Adafruit IO, conseguir Credenciales en https://io.adafruit.com
#define AIO_SERVER      "Conseguir"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "en"
#define AIO_KEY         "Adafruit IO"

// Funciones
void connect();

WiFiClient client;

/*Se guarda el servidor MQTT, El ID de Cliente, el Usuario y la Key en la flash
  Esto es requerido para usar la librería de Adafruit  */
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
/* seteamos un ID de cliente MQTT usando la key de Adafruit IO + la fecha y hora del sketch que fue compilado, así se sabra
   a que dispositivo corresponde y no debemos asignarle un ID único a cada dispositivo.
   de ultima podes setear un ID de cliente vos, con un valor aleatorio
*/
const char MQTT_CLIENTID[] PROGMEM  = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

/* Se prepara el Cliente MQTT, pasandole como argumentos la clase "ClienteWiFi", El servidor MQTT y detalles de login*/
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds de Adafruit***************************************/

/* Aca nos vamos a conectar a un feed de Adafruit IO, debemos definir el nombre que vamos a usar antes en el feed por si las dudas
   Tené en cuenta que los paths de MQTT para Adafruit IO siguen la forma: <nombreUsuario>/feeds/<nombreFeed>
   En este caso, yo lo llame display-lcd */
 
/*PARA PUBLICAR */
Adafruit_MQTT_Publish confirmacion= Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/confirmacion");

/*PARA RECIBIR */
const char pantallita[] PROGMEM = AIO_USERNAME "/feeds/display-lcd";
Adafruit_MQTT_Subscribe Display_lcd = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/display-lcd");

/*************************** Sketch Code ************************************/

void setup() {
   Serial.begin(115200);
  /* Se inicializa el LCD */
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(F("AdaIO By Juanstdio "));
 
  //Creamos los caracteres
  lcd.createChar(0, kora);
    lcd.createChar(1, hup);
      lcd.createChar(2, hdown);
        lcd.createChar(3, quiz);

  Serial.println(F("AdaIO By Juanstdio"));

  // Conectamos al Wifi
  Serial.println(); Serial.println();
  delay(10);
    lcd.setCursor(0,1);
      lcd.print(F("Conectando a "));
        lcd.setCursor(0,2);
          lcd.print(WLAN_SSID);
 // Iniciamos la conexión
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("Conectado a Wifi"));
  
  Serial.println(F("IP: "));
  lcd.setCursor(0,3);
  lcd.print(WiFi.localIP());
  Serial.println(WiFi.localIP());

  // aca ponemos en "escucha" a los feeds que estén relacionados a Display_lcd
  mqtt.subscribe(&Display_lcd);

  // Conectar a Adafruit.io
  connect();

}

void loop() {

  Adafruit_MQTT_Subscribe *subscription;

  // Esto es unicamente para que en caso que pierda la conexión, no se altere nada.
  if(! mqtt.ping(2)) {
    // Se reconecta por si pierde la conexión
    lcd.setCursor(0,0);
      lcd.print("Conexion perdida!");
        connect();
    if(! mqtt.connected()){}
  }

  
  // este es nuestro loop "Esperando paquetes del feed", podría ser mejorado
  while (subscription = mqtt.readSubscription(1000)) {

    // Solo nos interesan los cambios que esten relacionados a Display_lcd
    if (subscription == &Display_lcd) {

      
      //Convertimos el MQTT ASCII recibido a int 
      char *value = (char *)Display_lcd.lastread;
      Serial.print(F("Recibimos: "));
      Serial.println(value);

      // Convertimos a String el valor que nos acaba de llegar para poder procesarlo
      String message = String(value);
      message.trim();
      
       if(message =="CLEAR") {lcd.clear();}
      else if (message == "ON") { for (int fadeValue = 20 ; fadeValue >= 0; fadeValue -= 1) {lcd.setCursor(fadeValue,0); delay(3); lcd.write((uint8_t)0); lcd.setCursor(20-fadeValue,1); delay(3); lcd.write((uint8_t)0);lcd.setCursor(fadeValue,2); delay(3); lcd.write((uint8_t)0);lcd.setCursor(20-fadeValue,3); delay(3); lcd.write((uint8_t)0);}}
      else if (message == "OFF") {lcd.setCursor(0,1); lcd.write((uint8_t)2);}
      else if(message =="ANIMA"){ for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 1){lcd.setCursor(0,1);lcd.write((uint8_t)1);delay(3);lcd.setCursor(0,1);delay(3);lcd.write((uint8_t)2);}}
    else if (message.indexOf("Temp") >= 0) { 
      lcd.clear();
              String submensaje = message.substring(0,12 );
      lcd.setCursor(0,0); 
          lcd.print(submensaje);
                     
              String submensaje_2 = message.substring(18, 31);
      lcd.setCursor(0,1);
           lcd.print(submensaje_2);
          
              String submensaje_3 = message.substring(36);
      lcd.setCursor(0,2);
           lcd.print(submensaje_3);
         
      }
      else  /* Todo lo que sea distinto a lo definido, se muestra en pantalla */
      {
         lcd.setCursor(0,0); 
          lcd.print(message);
      }
         /* Estas lineas son para enviar una publicacion en un feed llamado CONFIRMACION*/     
  Serial.println(F("\nEnviando Confirmación de recepcion... "));
  char *respuesta = value;
  if (! confirmacion.publish(respuesta)) {
    Serial.println(F("Fallo!"));
  } else {
    Serial.println(F("Verificacion enviada!"));
  }
    }

  }

}

// Conectar a Adafruit Con MQTT
void connect() {

  Serial.print(F("Conectando a Adafruit IO... "));

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {

    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Reconectando...."));
    delay(5000);

  }

  Serial.println(F("Conectado a IO"));
  lcd.setCursor(0,1); lcd.print("Conectado a IO");
  delay(100);
  lcd.clear();

}
