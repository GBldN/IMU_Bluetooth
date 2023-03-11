/* ################# PROGRAMME DE PILOTAGE D'UN MOTOREDUCTEUR AVEC LE DRIVER TB6612FNG  #################
 * Programme pour piloter en vitesse un moto réducteur à partir d'un signal PWM et mesurer la fréquence de rotation avec son codeur.
 * 
 * Le mini driver moteur utilisé est un Sparkfun TB6612FNG https://www.sparkfun.com/products/14451 qui permet de controler 2 moteurs CC jusqu'à 1,2A par canal
 * Le moteur utilisé est un POLOLU #4753 avec une réduction de 50:1 et 64CPR (rise and fall)  https://www.pololu.com/product/4753
 * Son codeur doit être connecté aux 2 entrées d'interruption INT0 sur D2 et INT1 sur D3.
 * Mais il est possible d'adapter le programme à n'importe quel moteur et codeur.

 * Ce programme permet :
 *  - De controler la variation de vitesse ainsi que le sens de rotation par un potentiomètre (arrêt en position médiane) 
 *  - De Mesurer les 2 impulsions du codeur grâce à la lecture des interruptions
 *  - De calculer la fréquence de rotation DE LA SORTIE DU REDUCTEUR en tenant compte CPR (nombre d'implusions par tour) ainsi que le rapport de réduction du réducteur
 *  
 * Un exemple d'application est disponible sur le github https://github.com/GBldN/IMU_Bluetooth
 * 
 * Une vidéo du programme en situation est disponible sur youtube https://youtu.be/u4rxh-nPM8k
 *  
 * version=1.0
 * author=gael.balduini@gmail.com
 * licence=CC-by-nc-sa
 * maintainer=Arduino <info@arduino.cc>
 * sentence=PWM motor's control with rotation frequency calculation
 * paragraph= This program is used for control motor with PWM and read CPR Encoder for calculate freqyency 
 * category=Data Processing
 * url=https://github.com/GBldN/IMU_Bluetooth
 * architectures=**/

/* ##### PARAMETRES POUR PERSONNALISER LE MATERIEL ##### */
  #define f_ech             50                //Choix d'une fréquence en Hz pour le calcul à intervalle régulier
  #define prediviseur       64                //Prédiviseur du timer 2 pour un réglage de l'interruption à 976,5625 Hz
  #define TCNT_init         6                 //Choix d'une valeur de départ du comptage pour régler une fréquence de 1000 Hz
  #define f_timer()         (16000000 / prediviseur / ( 256 - TCNT_init))   //Calcul de la fréquence d'interruption
  
  #define CPRtick           64                //Nombre de ticks par tour du codeur (fonction du codeur utilisé)
  #define reduction         50                //Rapport de réduction du réducteur entre le codeur et l'arbre de sortie
  #define N_max             220               //Fréquence de rotation maxi de l'arbre de sortie ( ± en tr/min)
  #define Offset            10                 //Offset +/- pour l'arrêt du piltage de l'arbre de sortie (en tr/min) nécessaire pour la stabilité à l'arrêt
 
/* ##### DEFINITION DES CONNEXIONS ##### */
  #define An0_Pin           A0                //Port de connexion de l'entrée du CAN An0
  #define Bp1Pin            8                 //Port de connexion de l'entrée du bouton poussoir de calibrage (en option)

  #define MotSTBYPin        4                 //Port de connexion du signal STANDBY du driver de puissance
  #define MotPWMAPin        5                 //Port de connexion du signal PWMA du driver de puissance
  #define MotAIN1Pin        6                 //Port de connexion du signal AIN1 du driver de puissance
  #define MotAIN2Pin        7                 //Port de connexion du signal BIN1 du driver de puissance



/* ##### DEFINITION DES BIBLIOTHEQUES ET DES VARIABLES ##### */
/* -- Intégrations des bibiothèques -- */
 #include <Wire.h>                            //Intégration de la bibliothèque "wire" pour la gestion de l'I2C
 #include "rgb_lcd.h"                         //Intégration de la biblitothèque "rgb_lcd" pour la commande de l'écran GROVE I2C
 rgb_lcd lcd;                                 //Création de la fonction nommée ici "lcd"

/* -- Définition des variables pour l'état TOR ou les valeurs numériques des entrées -- */
  uint16_t           An0            = 0;      //Variable pour la position du potentiomètre de réglage de la consigne de 0 à 1024 (CAN 10 bits)
  bool               etat_BP        = false;  //Variable pour connaitre l'état du bouton poussoir
  volatile int32_t   tics            = 0;      //Variable pour le nombre de tics du capteur
  int32_t            tics_precedent  = 0;      //Variable pour le calcul de la fréquence
  uint32_t           t0             = 0 ;     //Variable pour le calcul du temps

  /* -- Définition des variables pour le pilotage -- */
  float              N_consigne     = 0;      //Variable pour la consigne de fréquence de rotation en tr/min
  float              N_mesure       = 0;      //Variable pour la fréquence réelle mesurée en tr/min
  float              erreur         = 0;      //Variable pour la mémorisation de la vitesse
  int16_t            MLI_mot        = 0;      //Variable pour le réglage du rapport cyclique du moteur

/* -- Déclaration des fonctions utiles au programme -- */
  uint8_t            compteur       = 0;      //Compteur pour le régalge de l'opération à lancer à la fréquence choisie
  uint8_t            CouleurR       = 0;      //Variable pour la gestion de la couleur Rouge de l'écran LCD
  uint8_t            CouleurV       = 255;    //Variable pour la gestion de la couleur Verte de l'écran LCD
  uint8_t            CouleurB       = 0;      //Variable pour la gestion de la couleur Bleu de l'écran LCD


  
/**************************************************************
      ROUTINE D'INITIALISATION  (exécutée une seule fois)
 **************************************************************/
void setup()
{
/* -- Configuration des broches en ENTREE ou SORTIE -- */
  pinMode(An0_Pin,          INPUT);
  pinMode(Bp1Pin,           INPUT);
  pinMode(MotAIN1Pin,       OUTPUT);
  pinMode(MotAIN2Pin,       OUTPUT);
  pinMode(MotPWMAPin,       OUTPUT);
  pinMode(MotSTBYPin,       OUTPUT);

  pinMode(2,       INPUT_PULLUP);
  pinMode(3,       INPUT_PULLUP);

  TWBR = ((F_CPU / 400000L) - 16) / 2;          //Réglage de la fréquence de fonctionnement du bus I2C à 400kHz
  setup_timer2();                               //Appel de la routine de configuration du timer 2
  


/* -- configuration des fonctions spéciales nécessaires au programme */
  Serial.begin(115200);                       //Initialisation de la bibliothèque Moniteur série
  lcd.begin(16, 2);                             //Démarrage et configuration de la fonction LCD pour un écran 2 lignes 16 caractères
  lcd.clear();                                  //Effacement de l'écran
  lcd.setRGB(0, 255, 0);                        //Réglage de la couleur de l'écran en VERT
  lcd.setCursor(0, 0);                          //Positionnement du curseur (caractère,ligne)
  lcd.print("PILOTAGE VITESSE");                //Ecriture du titre du programme sur le LCD
  lcd.setCursor(0, 1);
  lcd.print(" CODEUR ROTATION");                //Explication du role du bouton d'initialisation
  delay(2000);                                  //Pause pour l'affichage du nom du programme
  lcd.clear();

  attachInterrupt(digitalPinToInterrupt(2), Comptage, CHANGE);  //Création de l'interruption sur la broche INT0 : D2
  attachInterrupt(digitalPinToInterrupt(3), Comptage2,CHANGE);  //Création de l'interruption sur la broche INT1 : D3                                
  
  digitalWrite(MotSTBYPin,HIGH);                //mise à 1 du signal STANDBY du driver pour son activation
  t0 = millis();
}

/**************************************************************
           BOUCLE PRINCIPALE (exécutée en permanence)
 **************************************************************/
void loop()
{ 
/* -- Lecture des valeurs de entrées -- */
  An0  = analogRead(An0_Pin);                   //Lecture de la valeur du CAN de l'entrée A0
  N_consigne = map(An0,0,1023,-N_max,N_max);    //Définition de la valeur ± de fréquence de consigne en tr/min
 
/* -- Arret du moteur en cas d'appui sur le bouton poussoir -- */
 /* if ( digitalRead(Bp1Pin) == HIGH && etat_BP == false )
  {
    digitalWrite(MotSTBYPin,LOW);                //mise à 0 du signal STANDBY du driver pour son activation
    etat_BP = true;
  }
  
  if ( digitalRead(Bp1Pin) == LOW )
  {
    digitalWrite(MotSTBYPin,HIGH);                //mise à 1 du signal STANDBY du driver pour son activation
    etat_BP = false;
  }
*/
/* -- Affichage sur le moniteur série -- */
  Serial.print("N_consigne= " + String(N_consigne) + " tr/min ; N_mesure = " + String(N_mesure) + " tr/min"  );
  Serial.print(" | Pilotage moteur : " + String(MLI_mot));  Serial.print(" | tics = " + String(tics));
  Serial.println();

 PILOTAGE_MOTEUR();                             //Appel de la routine de pilotage du moteur

/* -- Gestion de la couleur de l'écran en fonction de la valeur du CAN  -- */
  
/* -- Affichage des valeurs sur l'écran tous les 200 ms-- */
  if ( ( millis() - t0 ) > 400 )
  {
    if ( abs(N_mesure) <= Offset )                      //Si la valeur du CAN est médiane (512) l'écran est VERT
    {
      CouleurB = 0 ;
      CouleurV = 255;
      CouleurR = 0;
    }
    if ( N_mesure < -Offset  )                //Si la valeur est dans la moitié inférieure l'écran tend vers le BLEU
    {
      CouleurB = (N_max - float(N_mesure)) * 255 / N_max;
      CouleurV = 255 * float(N_mesure) / N_max;
      CouleurR = 0;
    }
    if ( N_mesure > Offset )                  //Si la valeur est dans la moitié supérieure l'écran tend vers le ROUGE
    {
      CouleurB = 0;
      CouleurV = 255 * ( N_max - float(N_mesure)) / N_max;
      CouleurR = 255 * ( float(N_mesure) - N_max) / N_max;
    }
  
    lcd.setRGB(CouleurR, CouleurV, CouleurB); //Définition de la couleur de l'écran en fonction des valeurs
    lcd.clear();                              //Effacer l'écran
    lcd.setCursor(0, 0);                      //Ecriture sur la première ligne
    lcd.print("Nc= ");
    lcd.print(N_consigne,0);
    lcd.print(" tr/min");
    lcd.setCursor(0, 1);                      //Ecriture sur la seconde ligne, 2ème caractère
    lcd.print("Nm= "); lcd.print(N_mesure*6,0); lcd.print(" deg/s");
    //lcd.print("Nm= "); lcd.print(N_mesure,0); lcd.print(" tr/min");
    //lcd.print("tics");   lcd.print(tics);
    t0 = millis();
  }


}


/***********************************************************************
 *         ROUTINE EXECUTEE A CHAQUE PERIODE DEFINIE PAR LA FREQUENCE  *
 *            Insérer ici les calculs à faire à fréquence régulière    *
 ***********************************************************************/
void calcul()
{
 N_mesure = float( tics - tics_precedent) * f_ech * 60 / CPRtick / reduction;  //Calcul de la fréquence de rotation réelle de l'arbre de sortie en Hz
 tics_precedent = tics;                                                     //Mémorisation du nombre de tics pour le calcul suivant
   
 //erreur = N_consigne - N_mesure;                                            //Calcul de l'erreur
      /* -- Calcul de la commande MLI moteur -- */
 MLI_mot = map(N_consigne,-N_max,N_max,255,-255);
 //MLI_mot = Kp * erreur + Ki * somme_erreur + Kd * delta_erreur;           //Calcul de la valeur de pilotage asservie par correction PID ATTENTION MANQUE LES VARIABLES Somme et delta erreur)
 
      /* -- Normalisation et contrôle de dépassement du pilotage (si asservissement) -- */
 //if (MLI_mot >  255) MLI_mot =  255;             //Limitation du pilotage à la valeur mini admissible
 //if (MLI_mot < -255) MLI_mot = -255;             //Limitation du pilotage à la valeur mini admissible

}


/* ##### ROUTINE DE PILOTAGE DU MOTEUR  ##### */
void PILOTAGE_MOTEUR()
{
 if (abs(MLI_mot) <= Offset )                   //Si l'erreur est inférieure à l'offset de pilotage du moteur : ARRET du moteur
 {
  digitalWrite(MotAIN1Pin,LOW);                 //Mise à 0 du signal AIN1
  digitalWrite(MotAIN2Pin,LOW);                 //Mise à 0 du signal BIN1
  MLI_mot = 0;                                  //Arrêt du moteur
 }
 
 if (MLI_mot < -Offset )                        //Si la consigne est négative : ROTATION DANS LE SENS NEGATIF
 {
  digitalWrite(MotAIN1Pin,HIGH);                 //Mise à 0 du signal AIN1
  digitalWrite(MotAIN2Pin,LOW);                //Mise à 1 du signal BIN1
 }
 if (MLI_mot > Offset )                         //Si la consigne est positive  : ROTATION DANS LE SENS POSITIF
 {
  digitalWrite(MotAIN1Pin,LOW);                //Mise à 0 du signal AIN1
  digitalWrite(MotAIN2Pin,HIGH);                 //Mise à 1 du signal BIN1
 }

 analogWrite(MotPWMAPin,abs(MLI_mot));               //Réglage du rapport cyclique du MLI pour le moteur

}


/* ---------------------------------------------------------------------- *
    ROUTINE D'INCREMENTATION OU DECREMENTATION DE LA POSITION DU CODEUR
   ---------------------------------------------------------------------- */
void Comptage(void)
{
  if ( (PIND & B11111011) == 0) tics--;   //Lecture du portD si D2 == 1 et D3==0 
  else tics++;
}

void Comptage2(void)
{
  if ( (PIND & B11110111) == 0) tics--;     //Lecture du portD si D3 == 0 et D2==1 
  else tics++;
}


/***************************************
 * ROUTINE D'INITIALISATION DU TIMER 2 *
 ***************************************/
void setup_timer2()
{
  cli();                                                                //Désactive l'interruption globale
  TCCR2A = 0b00000000;                                                  //Réglage du timer 2 sur le Mode normal
  
  if (prediviseur==1)    TCCR2B = 0b00000001;
  if (prediviseur==8)    TCCR2B = 0b00000010;
  if (prediviseur==32)   TCCR2B = 0b00000011;
  if (prediviseur==64)   TCCR2B = 0b00000100;
  if (prediviseur==128)  TCCR2B = 0b00000101;
  if (prediviseur==256)  TCCR2B = 0b00000110;
  if (prediviseur==1024) TCCR2B = 0b00000111;
  
  TIMSK2 = 0b00000001;                                                  //Interruption locale autorisée par TOIE2
  TCNT2  = TCNT_init;                                                   //Démarrage du compteur de timer à la valeur TCNT_init
  sei();                                                                //Active l'interruption globale}

}

/***********************************************************************
 *  ROUTINE EXECUTEE A CHAQUE PERIODE DEFINIE PAR LA FREQUENCE  *
 *            Insérer ici les calculs à faire à fréquence régulière    *
 ***********************************************************************/
ISR(TIMER2_OVF_vect)
{        
  if ( compteur++ == ( f_timer() / f_ech  ) -1 ) 
  {
    compteur=0;   
    calcul();                                                          //Appel de la fonction de calculs à intervalle régulier réglé par la macro f_ech
  } 
  TCNT2 = TCNT_init ;
 }
