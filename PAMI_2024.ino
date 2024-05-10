#include "Adafruit_VL53L0X.h"  // librairie pour les capteurs laser
#include "math.h"



/*****************************************************************************************************************************************
    DEFINITION ENTREES SORTIES
*/
#define CODEUR_GAUCHE_A 2
#define CODEUR_GAUCHE_B 3
#define CODEUR_DROIT_A 18
#define CODEUR_DROIT_B 19

#define LED 42

#define TIRETTE 5

//Capteurs laser
//#define CAPTEUR_LASER_G 40
//#define CAPTEUR_LASER_D 41
#define SHT_LOX1 40
#define SHT_LOX2 41

//Moteur D
#define VITESSE_D 10
#define DIRECTION_D 12

//Moteur G
#define VITESSE_G 11
#define DIRECTION_G 13

//contacteur
#define CONTACTEUR A3

/*****************************************************************************************************************************************
    DEFINITION PARAMETRES
*/
#define SEUIL_OBSTACLE 200  //[mm]

//capteurs lasers
//adresses à programmer pour cahque capteur dans le setup
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define CONSIGNE_MAX 80  //entre 1 et 255
#define CONSIGNE_ANGLE_MAX1 160 //entre 1 et 255
#define CONSIGNE_ANGLE_MAX2 220 //entre 1 et 255
#define SEUIL_CONVERGENCE_DISTANCE 5.0
#define SEUIL_CONVERGENCE_ANGLE 0.04

#define DUREE_DEBUT 90000
#define DUREE_FIN 100000

#define BLEU 1
#define JAUNE 2


/*****************************************************************************************************************************************
    DECLARATION OBJETS UTILISES
*/

//capteurs laser
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

/*****************************************************************************************************************************************
    VARIABLES GLOBALES
*/

//couleur équipes
int couleur_equipe = 1; 
int signe_equipe=-1; //1 Jaune et -1 Bleu

//Pour stocker la valeur mesurée des capteurs laser
VL53L0X_RangingMeasurementData_t mesure_1;
VL53L0X_RangingMeasurementData_t mesure_2;

// codeur
volatile signed int counter1 = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile signed int counter2 = 0;  //This variable will increase or decrease depending on the rotation of encoder

//calculs d'asservissement
bool convergence_distance = true;
bool convergence_angle = true;
bool convergence = true;
float distance_consigne = 0.;
float angle_consigne = 0.;
float dcodeur = 4.625;  // diamètre de la roue codeuse  (à changer) en cm
float pi = 3.14159265359;
float perimetre_roue_codeuse = pi * dcodeur;
float ecartement_codeur = 22.4;
float perimetre_rotation_robot = ecartement_codeur * pi;
float ntour_par_pas = 1200;
float unite_distance = perimetre_roue_codeuse / ntour_par_pas;
float unite_rotation = 2 * pi * (unite_distance / perimetre_rotation_robot);
float erreur_gauche = 0.0;  // erreur pour le codeur 1
float erreur_droite = 0.0;  // erreur pour le codeur 2
float erreur_angle = 0.0;
float Kd = 10;  // coefficient de proportionalité distance
float Ka = 60;  // coefficient de proportionalité angle

//Moteur
int consigne_gauche = 0;  // consigne du moteur1
int consigne_droite = 0;  // cosnigne du moteur 2
int consigne_angle = 0;

//temps match
uint32_t t0 = 0;
uint32_t timeout_init=0;
uint32_t timeout_angle=0;
uint32_t timeout_distance=0;
uint32_t temps_match = 0;
uint32_t duree_match = 0;


//stratégie
uint8_t ETAPE = 0;

/*****************************************************************************************************************************************
    FONCTIONS
*/

void avancer(void) {
  digitalWrite(DIRECTION_D, HIGH);
  analogWrite(VITESSE_D, 80);
  digitalWrite(DIRECTION_G, HIGH);
  analogWrite(VITESSE_G, 65);
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
  digitalWrite(DIRECTION_D, HIGH);
  digitalWrite(DIRECTION_G, HIGH);
  analogWrite(VITESSE_D, 0);
  analogWrite(VITESSE_G, 0);
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

//fonction de consigne distance en cm
void setDistance(float distance, uint32_t duree_max) {
  Serial.print("consigne distance à ");
  Serial.println(distance);
  timeout_distance=duree_max;
  distance_consigne = distance;
  convergence_distance = false;
  convergence = false;
  counter1 = 0;
  counter2 = 0;
}

//fonction de consigne d'angle en radian
void setAngle(float angle, uint32_t duree_max) {
  Serial.print("consigne angle à ");
  Serial.println(angle);
  angle_consigne = angle;
  timeout_angle=duree_max;
  convergence_angle = false;
  convergence = false;
  counter1 = 0;
  counter2 = 0;
}

//fonction de calcul de l'asservissement
void calculs_asservissement(void) {
  if (!convergence_angle) {
    if(timeout_init==0)
      timeout_init=duree_match;

    //on corrige l'angle
    erreur_angle = angle_consigne - ((counter2 - counter1) / 2) * unite_rotation;
    //Serial.println(erreur_angle);

    if (fabs(erreur_angle) >= SEUIL_CONVERGENCE_ANGLE) {
      consigne_angle = (int)fabs(Ka * erreur_angle);
      Serial.println(consigne_angle);
      //écrêtage de la consigne
      if (erreur_angle > 0) {
        digitalWrite(DIRECTION_D, HIGH);
        digitalWrite(DIRECTION_G, LOW);
        if (consigne_angle >= CONSIGNE_ANGLE_MAX1) { consigne_angle = CONSIGNE_ANGLE_MAX1; }

      
      } else {
        digitalWrite(DIRECTION_D, LOW);
        digitalWrite(DIRECTION_G, HIGH);
        if (consigne_angle >= CONSIGNE_ANGLE_MAX2) { consigne_angle = CONSIGNE_ANGLE_MAX2; }
      }

      analogWrite(VITESSE_D, consigne_angle);
      analogWrite(VITESSE_G, consigne_angle);
    } else {
      analogWrite(VITESSE_D, 0);
      analogWrite(VITESSE_G, 0);
    }
  }

//Flag de convergence_angle
    if ((fabs(erreur_angle) <= SEUIL_CONVERGENCE_ANGLE) || ((duree_match-timeout_init)>timeout_angle))
    {
      convergence_angle = true;
      timeout_init=0;
    }

  if ((!convergence_distance) && (convergence_angle)) {
    if(timeout_init==0)
      timeout_init=duree_match;

    //calcul de l'erreur
    erreur_gauche = distance_consigne - (counter1 * unite_distance);  // calcul de l'erreur pour le moteur B
    erreur_droite = distance_consigne - (counter2 * unite_distance);  // calcul de l'erreur pour le moteur A


    if (fabs(erreur_droite) >= SEUIL_CONVERGENCE_DISTANCE) {
      consigne_droite = (int)fabs(Kd * erreur_droite);
      //écrêtage de la consigne
      if (consigne_droite >= CONSIGNE_MAX) { consigne_droite = CONSIGNE_MAX; }

      if (erreur_droite < 0)
        digitalWrite(DIRECTION_D, LOW);
      else
        digitalWrite(DIRECTION_D, HIGH);

      analogWrite(VITESSE_D, consigne_droite);
    } else
      analogWrite(VITESSE_D, 0);

    if (fabs(erreur_gauche) >= SEUIL_CONVERGENCE_DISTANCE) {
      consigne_gauche = (int)fabs(Kd * erreur_gauche);
      //écrêtage de la consigne
      if (consigne_gauche >= CONSIGNE_MAX) { consigne_gauche = CONSIGNE_MAX; }

      if (erreur_gauche < 0)
        digitalWrite(DIRECTION_G, LOW);
      else
        digitalWrite(DIRECTION_G, HIGH);

      analogWrite(VITESSE_G, consigne_gauche);
    } else
      analogWrite(VITESSE_G, 0);

    //Flag de convergence_distance
    if (((fabs(erreur_gauche) <= SEUIL_CONVERGENCE_DISTANCE) && (fabs(erreur_droite) <= SEUIL_CONVERGENCE_DISTANCE))|| ((duree_match-timeout_init)>timeout_distance))
    {
      convergence_distance = true;
      timeout_init=0;
    }



  }  //fin correction distance
  convergence = convergence_distance && convergence_angle;
}  //fin asserv

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                            SETUP
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() {

  Serial.begin(115200);



  //Init des entrées et sorties
  pinMode(TIRETTE, INPUT);

  pinMode(CONTACTEUR, INPUT);

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
        distanceObstacles();
        //on teste les obstacles si on n'est pas hors range sur au moins un des 2
        if ((mesure_1.RangeMilliMeter <= SEUIL_OBSTACLE) || (mesure_2.RangeMilliMeter <= SEUIL_OBSTACLE)) {
          arret();
        } else {
          //avancer();
          if (convergence) {
            switch (ETAPE) {
              case 0:
                //avancer de 10 cm
                setDistance(170,4000);
                ETAPE++;
                break;  //fin de la première étape

              case 1:
                //tourner de 90 deg
                setAngle(signe_equipe*1.57,2000);
                ETAPE++;
                break;  //fin de la deuxième étape

              case 2:
                //avancer jusqu'aux pots c'est à dire distance énorme 200 cm
                setDistance(115, 4000);
                ETAPE++;
                //setDistance(2000);

              default:
                break;  //étape par défaut : on ne fait rien

            }  //fin de la machine à états
          }    //fin test convergence_distance
          else {
            calculs_asservissement();
          }  //fincalcul asser

        }   //fin execution corps de match
      }     //fin test fin de match
      else  //fin test 90 et 100 s
      {
        arret();
      }
    }
  }
else{
  if(analogRead(CONTACTEUR)<=500){
    if(couleur_equipe==BLEU){
    couleur_equipe = JAUNE;
    signe_equipe=1;
    delay(1000);
    }
    else{
      couleur_equipe = BLEU;
      signe_equipe=-1;
      delay(1000);
    }
  }
if(couleur_equipe==JAUNE)
digitalWrite(LED, LOW);
else
digitalWrite(LED, HIGH);

}
}
