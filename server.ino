#include <SoftwareSerial.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Keypad.h>
/*
 * Projeto: Central de Alarme Sem Fio Open Source
 * Nome: Erick Eden Fróes
 */

 //Configurações do SIM900 - tx 51(vermelho) e rx 50 (verde) 
int tx = 41;
int rx = 40;
SoftwareSerial mySerial(tx,rx);
char flag=0; 
//Sim900 

//configurações gerais
const int pin_relay8 = 45;   //pino do reley que liga a sirene
boolean alarme = false;      //flag que verifica se o alarme está ou nao ativado
int meuNumero = 00000000;    //numero de telefone 
const int yellowPin = 13;    // pino para led amarelo
const int redPin = 15;       // pino para led vermelho
const int greenPin = 14;     // pino para led verde
const int audioPin = 31;     // pino do buzzer
const int duration = 200;    // pino do buzzer

//configurações KEYPAD
int count = 0;
char pass [6] = {'0', '0', '0', '0', '0' ,'0'}; //senha
const byte ROWS = 4; //Quatro linhas
const byte COLS = 3; //Três colunas
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {27, 26, 25, 24}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {30, 29, 28}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
//keypad



//Configurações do Ethernet Shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,0,177);
EthernetServer server(80); // Cria o servidor na porta 8081
// String que representa o estado dos dispositivos
char Luz[7] = "0000L#";
// String que guarda as msgs recebidas
char msg[7] = "0000L#";


//pisca led & buzzer
byte blnk = 0;           // variable used to track LED flip-flop
unsigned long tme = 0;   // the last time we processed doBlink()
unsigned long slc = 500; // milliseconds between doBlink() calls
unsigned long ms = 0;    // a millis() time-slice
byte buzz = false;



const byte pinRF = 21;  // Pino Rf

struct rfControl        //Struct for RF Remote Controls
{
   unsigned long addr;  //Endereço do RF
   boolean btn1;        //botão 1
   boolean btn2;        //Bbotão 2
 };

 boolean ACT_HT6P20B_RX(struct rfControl &_rfControl){ 

  static boolean startbit;      //checks if start bit was identified
  static int counter;           //received bits counter: 22 of Address + 2 of Data + 4 of EndCode (Anti-Code)
  static unsigned long buffer;  //buffer for received data storage
  
  int lambda;      // on pulse clock width (if fosc = 2KHz than lambda = 500 us)
  int dur0, dur1;  // pulses durations (auxiliary)
  
  if (!startbit)
  {// Check the PILOT CODE until START BIT;
    dur0 = pulseIn(pinRF, LOW);  //Check how long DOUT was "0" (ZERO) (refers to PILOT CODE)

    //If time at "0" is between 9200 us (23 cycles of 400us) and 13800 us (23 cycles of 600 us).
    if((dur0 > 9200) && (dur0 < 13800) && !startbit)
    {    
      //calculate wave length - lambda
      lambda = dur0 / 23;
      
      //Reset variables
      dur0 = 0;
      buffer = 0;
      counter = 0;
      
      startbit = true;
    }
  }

  //If Start Bit is OK, then starts measure os how long the signal is level "1" and check is value is into acceptable range.
  if (startbit && counter < 28)
  {
    ++counter;
    
    dur1 = pulseIn(pinRF, HIGH);
    
    if((dur1 > 0.5 * lambda) && (dur1 < (1.5 * lambda)))  //If pulse width at "1" is between "0.5 and 1.5 lambda", means that pulse is only one lambda, so the data ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â© "1".
    {
      buffer = (buffer << 1) + 1;   // add "1" on data buffer
    }
    else if((dur1 > 1.5 * lambda) && (dur1 < (2.5 * lambda)))  //If pulse width at "1" is between "1.5 and 2.5 lambda", means that pulse is two lambdas, so the data ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â© "0".
    {
      buffer = (buffer << 1);       // add "0" on data buffer
    }
    else
    {
      //Reset the loop
      startbit = false;
    }
  }
  
  //Check if all 28 bits were received (22 of Address + 2 of Data + 4 of Anti-Code)
  if (counter==28) 
  { 
    // Check if Anti-Code is OK (last 4 bits of buffer equal "0101")
    if ((bitRead(buffer, 0) == 1) && (bitRead(buffer, 1) == 0) && (bitRead(buffer, 2) == 1) && (bitRead(buffer, 3) == 0))
    {     
      counter = 0;
      startbit = false;
      
      //Get ADDRESS CODE from Buffer
      _rfControl.addr = buffer >> 6;
      
      //Get Buttons from Buffer
      _rfControl.btn1 = bitRead(buffer,4);
      _rfControl.btn2 = bitRead(buffer,5);

      //Serial.print("Address: "); Serial.println(_rfControl.addr, HEX);
      //Serial.print("Button1: "); Serial.println(_rfControl.btn1, BIN);
      //Serial.print("Button2: "); Serial.println(_rfControl.btn2, BIN);
      //Serial.println();
      
      //If a valid data is received, return OK
      return true;
    }
    else
    {
      //Reset the loop
      startbit = false;
    }
  }
  
  //If none valid data is received, return NULL and FALSE values 
  _rfControl.addr = NULL;
  _rfControl.btn1 = NULL;
  _rfControl.btn2 = NULL; 
  
  return false;
}



void setup() {
  //define os pinos
  pinMode(pinRF, INPUT);
  pinMode(audioPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(pin_relay8, OUTPUT);
  
  mySerial.begin(19200); //inicia o SIM900
  Serial.begin(19200); 
  key_init(); //inicializa o teclado
  Ethernet.begin(mac,ip); //prepara o ethernet
  server.begin(); //inicia o ethernet
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  digitalWrite(pin_relay8, HIGH); //bloqueia a sirene ao iniciar
  
}

void loop() {

  servidor();
  
  buzzer();
  
  teclado();
  
  lerSensores();
  
  
}


void servidor(){

  interrupts();
  EthernetClient client = server.available();
  
  if (client) {
    // guarda o caracter na string 'msg'
    
    msg[1]=msg[2]; msg[2]=msg[3]; msg[3]=msg[4]; msg[4]=msg[5]; msg[5]=msg[6];
    msg[6] = client.read();
    Serial.print(msg[6]);
    if (msg[6]=='#') {

     switch(msg[5]) {
      case 'R':
          // Se receber o comando 'R#' envia de volta o status dos
          //   dispositivos. (Que será a string 'Luz')
          client.write(Luz);
          break;
          case 'P':
          // Caso P#, aciona o pino do portão pequeno por 1s.
          digitalWrite(A4,HIGH);
          delay(1000);
          digitalWrite(A4,LOW);
          break;
          case 'G':
          // Caso G#, aciona o pino do portão pequeno por 1s.
          digitalWrite(A5,HIGH);
          delay(1000);
          digitalWrite(A5,LOW);        
          break;
          case 'L':
          // Caso L#, ele copia os 4 bytes anteriores p/ a
          //   string 'Luz' e cada byte representa um
          // dispositivo, onde '1'=ON e '0'=OFF
          Luz[0]=msg[1];
          Luz[1]=msg[2];
          Luz[2]=msg[3];
          Luz[3]=msg[4];
          Serial.print(Luz[0]);
          if (Luz[0]=='1') {
           ativaAlarme();   
           }else{
           desativaAlarme(); 
           }

           break;

         }
       }
     } 
     noInterrupts(); 
   }


   void teclado(){
    interrupts();
    char key = keypad.getKey();

    if (key != NO_KEY){
      Serial.println(key);
      if (key == '#') {
        code_entry_init();
        int entrada = 0;
        while (count < 6 ){
          char key = keypad.getKey();
          if (key != NO_KEY){
            entrada += 1;
            tone(audioPin, 1080, 100);
            delay(duration);
            noTone(audioPin);
            if (key == pass[count])count += 1;
            if ( count ==  6) unlocked();
            if ((key == '#') || (entrada == 6)){
              break;
            }
          }
        }
      }
    }
    noInterrupts(); 
  }

  void lerSensores(){

   struct rfControl rfControl_1;    //Set variable rfControl_1 as rfControl type

   if(ACT_HT6P20B_RX(rfControl_1))
   {
    //If a valid data is received, print ADDRESS CODE and Buttons values    
    Serial.print("Address: "); Serial.println(rfControl_1.addr);
    Serial.print("Button1: "); Serial.println(rfControl_1.btn1);
    Serial.print("Button2: "); Serial.println(rfControl_1.btn2);
    Serial.println();
    switch(rfControl_1.addr)
    { 

case 00000000:   // Codigo do Sensor Sem Fio 

if (rfControl_1.btn1 == 1) // Botao 1 do Sensor sem fio acionado 
{ 
  if(alarme == true){
    Serial.println("Alarme Disparado!");
    buzz = true; 
    //mandaSMS();
    //ligar();
    disparaSirene();

  }
}
if (rfControl_1.btn1 == 0) // Botao 2 do Sensor sem Fio acionado
{ 
  Serial.println(" Bateria fraca no Sensor sem Fio"); 
}
break; 

default: 
break;
}



}


}

void mandaSMS(){
  interrupts();    
  if(flag==0)
  {
      mySerial.print("AT+CMGF=1\r"); //mandando SMS em modo texto
      delay(1000);
      mySerial.print("AT+CMGS=\"+998016219\"\r"); // numero que vamos mandar o SMS
      delay(1000);
      Serial.println("SMS Armado");
      flag=1;
    }

    if(flag==1)
    {
        mySerial.print("Perimetro invadido!Alarme disparado!\r"); // corpo da msg
        delay(1000);
        mySerial.write(0x1A); //equivalente a mandar Ctrl+Z(finaliza corpo do SMS)
        delay(1000);
        Serial.println("Pronto pra enviar");
        flag=2;
      }
      if(flag==2)
      {
        Serial.println("SMS Enviado com sucesso!");
        delay(5000);
        flag=3;
      }
      noInterrupts(); 
    }

    void ligar(){
      interrupts();
      mySerial.println("\r");
      delay(1000);
      mySerial.println("AT+CMGF=1\r");
      Serial.println("iniciando a ligação...");
      delay(1000);


      Serial.println("discando...");
      if(ligarACobrar == true){
       mySerial.println("ATD 9090998016219;"); 
       }else if(ligarACobrar == true){
         mySerial.println("ATD 998016219;");
       }  
         // numero a ser discado
    delay(10000);                           //completar licação
    delay(6000);                           //receber ligação, esperar.
    mySerial.println("ATH0");              //finaliza ligação
    Serial.println("ligação finalizada...");
    noInterrupts();
  }

  void key_init (){
    count = 0;
    digitalWrite(redPin, HIGH);
    digitalWrite(yellowPin, LOW);
    digitalWrite(greenPin, LOW);
    tone(audioPin, 1080, 100);
    delay(duration);
    noTone(audioPin);
    tone(audioPin, 980, 100);
    delay(duration);
    noTone(audioPin);
    tone(audioPin, 770, 100);
    delay(duration);
    noTone(audioPin);
  }

  void code_entry_init(){
    count = 0;
    tone(audioPin, 1500, 100);
    delay(duration);
    noTone(audioPin);
    tone(audioPin, 1500, 100);
    delay(duration);
    noTone(audioPin);
    tone(audioPin, 1500, 100);
    delay(duration);
    noTone(audioPin);
    digitalWrite(redPin, LOW);
    digitalWrite(yellowPin, HIGH);
    digitalWrite(greenPin, LOW);
  }

  void unlocked(){

    if(alarme==false){
      ativaAlarme();
      }else if(alarme==true){
      desativaAlarme();
      }
    }


    void ativaAlarme(){
      digitalWrite(pin_relay8, LOW);
      delay(500);
      digitalWrite(pin_relay8, HIGH);
      alarme = true;
      Serial.println(alarme);
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      Serial.println("Alarme Ativado");

    }

    void desativaAlarme(){
      delay(500);
      digitalWrite(pin_relay8, LOW);
      delay(500);
      digitalWrite(pin_relay8, HIGH);
      delay(500);
      digitalWrite(pin_relay8, LOW);
      delay(500);
      digitalWrite(pin_relay8, HIGH);
      delay(500);
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      alarme= false;
      buzz = false;
      Serial.write("0000L#");
      Serial.println(alarme);
      Serial.println("Alarme Desativado!");
    }

    void buzzer(){
      ms = millis();
  if(buzz == true){
  if ( tme + slc < ms ) {
    tme = ms;
    doBlink();
    soundAlarm();
  } 

} 
}

void doBlink() {
  if ( blnk == 0 ) {
    digitalWrite( greenPin, HIGH );
    digitalWrite( redPin, LOW );
    blnk = 1;
  }
  else {
    digitalWrite( greenPin, LOW );
    digitalWrite( redPin, HIGH );
    blnk = 0;  
  }
}


void soundAlarm() {
 float alarmFrequency=1400; 
 float period = (1.0 / alarmFrequency) * 1000000;
 long beepDuration=250000; 
 long elapsedTime = 0;
 while (elapsedTime < beepDuration) {
   digitalWrite(audioPin,HIGH);
   delayMicroseconds(period / 2);
   digitalWrite(audioPin, LOW);
   delayMicroseconds(period / 2);
   elapsedTime += (period);
 }
 digitalWrite(audioPin, LOW); 
 delayMicroseconds(beepDuration);
}

void disparaSirene(){

  digitalWrite(pin_relay8, LOW); 
  
  ms = millis();
  if(buzz == true){
  if ( tme + slc < ms ) {
    tme = ms;
    mandaSMS();
  } 

} 

}