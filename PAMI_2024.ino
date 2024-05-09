#include "Adafruit_VL53L0X.h"  // librairie pour les capteurs laser
#include "math.h"



/*****************************************************************************************************************************************
    DEFINITION ENTREES SORTIES
*/
#define CODEUR_GAUCHE_A 2
#define CODEUR_GAUCHE_B 3
#define CODEUR_DROIT_A 18
#define CODEUR_DROIT_B 19

#define TIRETTE 5

//Capteurs laser
//#define CAPTEUR_LASER_G 40
//#define CAPTEUR_LASER_D 41
#define SHT_LOX1 40
#define SHT_LOX2 41

//Moteur A
#define VITESSE_A 10
#define DIRECTION_A 12

//Moteur B
#define VITESSE_B 11
#define DIRECTION_B 13



/*****************************************************************************************************************************************
    DEFINITION PARAMETRES
*/
#define SEUIL_OBSTACLE 200  //[mm]

//capteurs lasers
//adresses à programmer pour cahque capteur dans le setup
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define CONSIGNE_MAX 200  //entre 1 et 255
#define SEUIL_CONVERGENCE_DISTANCE 1

#define DUREE_DEBUT 90000
#define DUREE_FIN 100000


/*****************************************************************************************************************************************
    DECLARATION OBJETS UTILISES
*/

//capteurs laser
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

/*****************************************************************************************************************************************
    VARIABLES GLOBALES
*/

//Pour stocker la valeur mesurée des capteurs laser
VL53L0X_RangingMeasurementData_t mesure_1;
VL53L0X_RangingMeasurementData_t mesure_2;

// codeur
volatile signed int counter1 = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile signed int counter2 = 0;  //This variable will increase or decrease depending on the rotation of encoder

//calculs d'asservissement
bool convergence = true;
float distance_consigne = 0.;
float angle_consigne = 0.;
float dcodeur = 4.625;  // diamètre de la roue codeuse  (à changer) en cm
float pi = 3.14159265359;
float perimetre_roue_codeuse = pi * dcodeur;
float ntour_par_pas = 1200;
float unite_distance = perimetre_roue_codeuse / ntour_par_pas;
float erreur1 = 0;  // erreur pour le codeur 1
float erreur2 = 0;  // erreur pour le codeur 2
float k = 10;       // coefficient de proportionalité

//Moteur
int consigne1 = 0;  // consigne du moteur1
int consigne2 = 0;  // cosnigne du moteur 2

//temps match
uint32_t t0 = 0;
uint32_t temps_match = 0;
uint32_t duree_match = 0;


//stratégie
uint8_t ETAPE = 0;

/*****************************************************************************************************************************************
    FONCTIONS
*/

void avancer(void) {
  digitalWrite(DIRECTION_A, HIGH);
  analogWrite(VITESSE_A, 80);
  digitalWrite(DIRECTION_B, HIGH);
  analogWrite(VITESSE_B, 65);
}

//Fonctions incréments et décréments Codeur Gauche
void ai0(void) {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(CODEUR_GAUCHE_B) == LOW) {
    counter1++;
  } else {
    counter1--;
  }
}
void ai1(void) {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(CODEUR_GAUCHE_A) == LOW) {
    counter1--;
  } else {
    counter1++;
  }
}

//Fonctions incréments et décréments Codeur Droit
void ai2(void) {
  if (digitalRead(19) == LOW) {
    counter2++;
  } else {
    counter2--;
  }
}
void ai3(void) {
  if (digitalRead(18) == LOW) {
    counter2--;
  } else {
    counter2++;
  }
}

//Sous fonctions moteur
void arret(void) {
  digitalWrite(DIRECTION_A, HIGH);
  digitalWrite(DIRECTION_B, HIGH);
  analogWrite(VITESSE_A, 0);
  analogWrite(VITESSE_B, 0);
}


//Fonction de programmation des capteurs laser
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);


  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
}

//Fonction de lecture des distances des capteurs laser
void distanceObstacles() {

  lox1.rangingTest(&mesure_1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&mesure_2, false);  // pass in 'true' to get debug data printout!

  // print sensor one reading
  /*Serial.print(F("1: "));
  if (mesure_1.RangeStatus != 4) {    // if not out of range
    Serial.print(mesure_1.RangeMilliMeter);
  } 
  else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if (mesure_2.RangeStatus != 4) {
    Serial.print(mesure_2.RangeMilliMeter);
  } 
  else {
    Serial.print(F("Out of range"));
  }*/

  Serial.println();
}

//fonction de consigne
void setDistance(float distance) {
  distance_consigne = distance;
  convergence = false;
  counter1 = 0;
  counter2 = 0;
}

//fonction de calcul de l'asservissement
void calculs_asservissement(void) {
  /*if (angle_consigne != 0.)
    {
    //on corrige l'angle
    }*/

  /*
    MODE CRADE
    digitalWrite(DIRECTION_A, HIGH);
      analogWrite (VITESSE_A, CONSIGNE_MAX);
           digitalWrite(DIRECTION_B, HIGH);
      analogWrite (VITESSE_B, CONSIGNE_MAX);
      
      convergence=true;
      
      analogWrite (VITESSE_A, 0);
      analogWrite (VITESSE_B, 0);
      */
  if ((distance_consigne != 0.) && (!convergence)) {

    //calcul de l'erreur
    erreur1 = distance_consigne - (counter1 * unite_distance);  // calcul de l'erreur pour le moteur B
    erreur2 = distance_consigne - (counter2 * unite_distance);  // calcul de l'erreur pour le moteur A


    //MOTEUR A
    if (erreur2 >= SEUIL_CONVERGENCE_DISTANCE) {
      consigne2 = k * erreur2;
      //écrêtage de la consigne
      if (consigne2 >= CONSIGNE_MAX) { consigne2 = CONSIGNE_MAX; }

      digitalWrite(DIRECTION_A, HIGH);
      analogWrite(VITESSE_A, consigne2);
    } else
      analogWrite(VITESSE_A, 0);

    if (erreur2 <= -SEUIL_CONVERGENCE_DISTANCE) {
      consigne2 = k * erreur2;
      //écrêtage de la consigne
      if (consigne2 >= CONSIGNE_MAX) { consigne2 = CONSIGNE_MAX; }

      digitalWrite(DIRECTION_A, LOW);
      analogWrite(VITESSE_A, consigne2);
    } else
      analogWrite(VITESSE_A, 0);

    //MOTEUR B
    if (erreur1 >= SEUIL_CONVERGENCE_DISTANCE) {
      consigne1 = k * erreur1;
      //écrêtage de la consigne
      if (consigne1 >= CONSIGNE_MAX) { consigne1 = CONSIGNE_MAX; }

      digitalWrite(DIRECTION_B, HIGH);
      analogWrite(VITESSE_B, consigne1);
    } else
      analogWrite(VITESSE_B, 0);

    if (erreur1 <= -SEUIL_CONVERGENCE_DISTANCE) {
      consigne1 = k * erreur1;
      //écrêtage de la consigne
      if (consigne1 >= CONSIGNE_MAX) { consigne1 = CONSIGNE_MAX; }

      digitalWrite(DIRECTION_B, LOW);
      analogWrite(VITESSE_B, consigne1);
    } else
      analogWrite(VITESSE_B, 0);

    //Flag de convergence
    if (((fabs)(erreur1) <= SEUIL_CONVERGENCE_DISTANCE) && (fabs(erreur2) <= SEUIL_CONVERGENCE_DISTANCE))
      convergence = true;



  }  //fin correction distance

}  //fin asserv

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                            SETUP
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() {

  Serial.begin(115200);



  //Init des entrées et sorties
  pinMode(TIRETTE, INPUT);

  //codeur Gauche
  pinMode(CODEUR_GAUCHE_A, INPUT_PULLUP);
  pinMode(CODEUR_GAUCHE_B, INPUT_PULLUP);

  //programmation des interruptions pour le codeur Gauche
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);

  //codeur Droit
  pinMode(CODEUR_DROIT_A, INPUT_PULLUP);
  pinMode(CODEUR_DROIT_B, INPUT_PULLUP);

  //programmation des interruptions pour le codeur Droit
  attachInterrupt(5, ai2, RISING);
  attachInterrupt(4, ai3, RISING);

  //Initialisation des capteurs laser
  // wait until serial port opens for native USB devices
  while (!Serial) {
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));


  Serial.println(F("Lancement des mesures laser..."));
  setID();
}

void loop() {
  if (digitalRead(TIRETTE) == LOW) {
    if (t0 == 0) {
      //je commence le match (init du temps t0)
      t0 = millis();
      Serial.println("Tirette retirée, initialisation du t0");
    } else {
      temps_match = millis();
      duree_match = temps_match - t0;
      //Serial.print(temps_match);Serial.print(" - ");Serial.print(t0);Serial.print(" = ");Serial.println(duree_match);

      //on est à 90 secondes on lance le match
      if ((duree_match >= DUREE_DEBUT) && (duree_match <= DUREE_FIN)) {
        Serial.print(temps_match);Serial.print(" - ");Serial.print(t0);Serial.print(" = ");Serial.println(duree_match);
        distanceObstacles();
        //on teste les obstacles si on n'est pas hors range sur au moins un des 2
        if ((mesure_1.RangeMilliMeter <= SEUIL_OBSTACLE) || (mesure_2.RangeMilliMeter <= SEUIL_OBSTACLE)) {
          arret();
        }
        else
        {
          avancer();
          /*if (convergence)
            {
              switch (ETAPE)
              {
                case 0:
                  //avancer de 10 cm
                  Serial.println("Première étape: avancer de 10 cm");
                  setDistance(150);
                  ETAPE++;
                  break; //fin de la première étape

                case 1:
                  //tourner de 90 deg
                  break;//fin de la deuxième étape

                case 2:
                //avancer jusqu'aux pots c'est à dire distance énorme 200 cm

                default:
                  break; //étape par défaut : on ne fait rien

              }//fin de la machine à états
            }//fin test convergence
            else
            {
              calculs_asservissement();
            }//fincalcul asser

        }  //fin execution corps de match*/
      }    //fin test fin de match
    }
    else  //fin test 90 et 100 s
    {
      arret();
    }
  }  //fin test init match
}  //fin tirette


}  //fin Loop
