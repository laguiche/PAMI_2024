#include "Adafruit_VL53L0X.h" // librairie pour les capteurs laser

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// capteur laser
/*
les deux capteurs lasers ont leurs pin SDA et SCL connecté sur les pin 20 et 21 càd les pin SDA et SCL de la carte arduino
*/
int const CAPTEUR_1 = 22; // pin Xshut du capteur (mise a LOW pour reset et HIGH pour les activer)
int const CAPTEUR_2 = 24; // pin Xshut du capteur (mise a LOW pour reset et HIGH pour les activer)

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t mesure_1;
VL53L0X_RangingMeasurementData_t mesure_2;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// codeur

volatile signed int counter1 = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile signed int counter2 = 0;  //This variable will increase or decrease depending on the rotation of encoder

float dcodeur = 4; // diamètre de la roue codeuse  (à changer) en cm

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////déplacement

float distance = 100; // distance à parcourir en cm (à changer pour faire des tests)

int consigne1 = 0; // consigne du moteur1
int consigne2 = 0; // cosnigne du moteur 2

//calculs

int vitesse_1; // consigne de vitesse pour le moteur 1
int vitesse_2; // consigne de vitesse pour le moteur 2

float pi = 3.14159265359;
float perimetre_roue_codeuse = pi*dcodeur;
float ntour_par_pas = 1200/perimetre_roue_codeuse;
float unite_distance = perimetre_roue_codeuse/ntour_par_pas;
float erreur1 = 0; // erreur pour le codeur 1
float erreur2 = 0; // erreur pour le codeur 2
float k = 10; // coefficient de proportionalité

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Moteur

//  Moteur A def pin
int const VITESSE_A = 10;  
int const DIRECTION_A = 12;
//  Moteur B def pin
int const VITESSE_B = 11;  
int const DIRECTION_B = 13;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// tirette

int TIRETTE = 32; // def pin tirette

uint32_t tDebut;




void setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// capteur laser
  // mise en reset des deux capteurs
  digitalWrite(CAPTEUR_1, LOW);    
  digitalWrite(CAPTEUR_2, LOW);
  delay(10);
  
  // on sort du reset
  digitalWrite(CAPTEUR_1, HIGH);    
  digitalWrite(CAPTEUR_2, HIGH);
  delay(10);

  // on met en reset le capteur_2 et on active le capteur_1
  digitalWrite(CAPTEUR_1, HIGH);    
  digitalWrite(CAPTEUR_2, LOW);
  delay(10);
  lox1.begin(0x30); // on définit l'adresse du capteur_1

  // on active le capteur_2
  digitalWrite(CAPTEUR_2, HIGH);
  delay(10);
  lox2.begin(0x31); // on définit l'adresse du capteur_2

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////réception de l'information des roues codeuses
 
  pinMode(32, INPUT_PULLUP);           // set pin to input et pullup résistance
  pinMode(34, INPUT_PULLUP);           // set pin to input et pullup résistance
  
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);

  pinMode(38, INPUT_PULLUP);           // set pin to input et pullup résistance
  pinMode(40, INPUT_PULLUP);           // set pin to input et pullup résistance
  
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(5, ai2, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(4, ai3, RISING);

}

void loop() {
  tDebut = millis();

  if(tDebut == 90000)
  {
    if (mesure_1.RangeMilliMeter<=30 || mesure_2.RangeMilliMeter<=30)
      {
        delay(500);
      }
    else if (tDebut == 100000 )
      {
        ARRET();
        while(1);
      }
    else
      {
        switch()
        {
          case 1:
            
            distance = ?;            
            
            erreur1 = distance - (counter1*unite_distance); // calcul de l'erreur pour le moteur1 
            erreur2 = distance - (counter2*unite_distance); // calcul de l'erreur pour le moteur2

          //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Boucle de rétroaction moteur 1

            if (erreur1 > 0.5) // marge d'erreur de 5mm pour l'instant --> test
              {            
                digitalWrite(DIRECTION_A, HIGH);; // on fais tourner le moteur pour s'approcher de l'erreur nul
                consigne1 = k*erreur1 ;

                if (consigne1 > 255) // pour que la consigne ne parte pas en vrille
                  {
                    consigne1 = 255;
                  }
                vitesse_1 = consigne1; // vitesse du moteur
                analogWrite (VITESSE_A, vitesse_1); // On donne la consigne
              }
            else
              {
                digitalWrite (DIRECTION_A, LOW); // s'il ne se passe rien le moteur n'avance pas
                analogWrite(VITESSE_A, 0); // mise de la vitesse à 0
              }

            if (erreur1 < -0.5) // s'il y a dépassement on fais l'inverse
              {
                digitalWrite (DIRECTION_A, LOW);
                consigne1 = -k*erreur1 ;

                if (consigne1 > 255) 
                  {
                    consigne1 = 255;
                  }
                vitesse_1 = consigne1;
                analogWrite (vitesse_1, vitesse_1);
              }
            else 
              {
                digitalWrite (DIRECTION_A, LOW); // se passe rien
                analogWrite (VITESSE_A, 0);
              }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Boucle de rétroaction moteur 2

            if (erreur2 > 0.5) { // marge d'erreur de 5mm pour l'instant --> test
              digitalWrite (DIRECTION_B, HIGH); // on fais tourner le moteur pour s'approcher de l'erreur nul
              consigne2 = k*erreur2 ;

              if (consigne2 > 255) { // pour que la consigne ne parte pas en vrille
                consigne2 = 255;
              }
              vitesse_2 = consigne2; // vitesse du moteur
              analogWrite (VITESSE_B, vitesse_2); // On donne la consigne
            }
            else {
              digitalWrite (DIRECTION_B, LOW); // s'il ne se passe rien le moteur n'avance pas
              analogWrite(VITESSE_B, 0); // mise de la vitesse à 0
            }

            if (erreur2 < -0.5) { // s'il y a dépassement on fais l'inverse
              digitalWrite (DIRECTION_B, HIGH);
              consigne2 = -k*erreur2 ;

              if (consigne2 > 255) {
                consigne2 = 255;
              }
              vitesse_2 = consigne2;
              analogWrite (VITESSE_B, vitesse_2);
            }
            else {
              digitalWrite (DIRECTION_B, LOW); // se passe rien
              analogWrite(VITESSE_B, 0)
            }

            break;

          case 2;
            /*
            tourner a droite ou a gauche
            */
            



        }
      
      }
  }


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////// tirette

int DEPART(void){
int debut;
debut = 0;
if (digitalRead(TIRETTE)==HIGH){
  return 1;
}
else{
  return 0;
}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////// codeur 1
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter1++;
  }else{
    counter1--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter1--;
  }else{
    counter1++;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////// codeur 2
void ai2() {
  if(digitalRead(19)==LOW) {
    counter2++;
  }else{
    counter2--;
  }
}

void ai3() {
  if(digitalRead(18)==LOW) {
    counter2--;
  }else{
    counter2++;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////// sous fonction moteur

void ARRET(void){
  digitalWrite(DIRECTION_A, HIGH);
  digitalWrite(DIRECTION_B, HIGH);
  analogWrite(VITESSE_A, 0);
  analogWrite(VITESSE_B, 0);
  delay(3000);
}

