/* ################# PROGRAMME POUR ENVOYER EN BLUETOOTH LES VALEURS D'UNE CENTRALE INERTIELLE #################
 * 
 * Ce programme permet d'envoyer en Bluetooth les valeurs d'accélérations et de vitesses angulaires lues grâce à la centrale inertielle 
 *   - La centrale utilisée est une Grove_IMU_6 DOF : https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/ 
 *   - Le module Bluetooth utilisé est un module Grove - Serial Bluetooth v3.0 https://wiki.seeedstudio.com/Grove-Serial_Bluetooth_v3.0/
 * 
 * Pour simplifier le décodage le programme envoie en BT une trame de caractères ASCII : "Debut,ax:±##.##;g,ay:±##.##;g,ay:±##.##;g,gx:±####;deg/s,gy:±####;deg/s,gz:±####;deg/s,Fin  "
 *            dont les #### correspondent aux valeurs
 *  
 * La réception des informations est prévue avec un smartphone. L'application Android développé avec APP INVENTOR https://gallery.appinventor.mit.edu/?galleryid=7bed003b-4152-40f3-b9e8-77ac0583e04f
 * 
 * L'ensemble des descriptions et ressources sont disponibles sur le github https://github.com/GBldN/IMU_Bluetooth
 * 
 * Une vidéo du programme en situation est disponible https://youtu.be/u4rxh-nPM8k
 *  
 * version=1.0
 * author=gael.balduini@gmail.com
 * licence=CC-by-nc-sa
 * maintainer=Arduino <info@arduino.cc>
 * url=https://github.com/GBldN/IMU_Bluetooth
 */

/* ##### DECLARATION DES MACROS ##### */
   
/* ##### DECLARATION DES CONSTANTES ##### */
  #define BtTxPin                   4             //Port de connexion Tx-O du module BT
  #define BtRxPin                   5             //Port de connexion Rx-I du module BT
  
  #define       f_ech               100           //Choix d'une fréquence en Hz pour le calcul à intervalle régulier entre 1 et 500 Hz (ATTENTION ! 1000 doit être un multiple de f_ech)
  
  #define       pleine_echelle_acc  16            //réglage de la gamme de mesure de l'accéléromètre ±2g ; ±4g ; ±8g ou ±16g
  #define       sensi_acc           488;          //Sensibilité de l'accéléromètre en g/LSB   (réglé par défault pour 4g mais se personnalise en fonction de la pleine échelle
  #define       pleine_echelle_gyr  2000          //réglage de la gamme de mesure du gyromètre ± 250; 500; 1000; 2000 °/s (2; 41; 42; 83; 167; 333 tr/min) ou ±125°/s pour le LM6DS3 
  #define       sensi_gyr           70000;        //Sensibilité du gyromètre       en dps/LSB (réglé par défault pour 2000 °/s mais se personnalise en fonction de la pleine échelle)
  

/* ##### DEFINITION DES BIBLIOTHEQUES ET DES VARIABLES ##### */
/* -- Intégrations des bibiothèques -- */
  #include <SoftwareSerial.h>                     //Emulateur d'une liaison série pour l'utilisation sur d'autres broches que les 0 et 1
  #include <Wire.h>                               //Intégration de la bibliothèque "wire" pour la gestion de l'I2C

/* -- Définition des variables pour l'état TOR ou les valeurs numériques des entrées -- */

/* -- Déclaration des variable pour les calculs intermédiares -- */
  char          chaine_recue[60];                  //Tableau des caractères reçus en bluetooth non synchronisée
  uint16_t      longueur;                          //nombre de caractères lus sur la liaison série
  String        mot;                               //Message extrait des caractères recus
  
  unsigned long t0                = 0;             //Permet de stocker le temps
  uint8_t      compteur           = 0;             //Compteur pour le régalge de l'opération à lancer à la fréquence choisie
   
  int16_t       ax_brut, ay_brut, az_brut;         //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
  int16_t       gx_brut, gy_brut, gz_brut =0;      //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)
  int32_t       ax_offset, ay_offset, az_offset;   //Variables pour le stockage des valeurs d'offsets de l'accéléromètre
  int32_t       gx_offset, gy_offset, gz_offset;   //Variables pour le stockage des valeurs d'offsets du gyrocope

  float         Ax_reel, Ay_reel, Az_reel;         //Variables pour le calcul des valeurs réelle de l'accéléromètre
  float         Gx_reel, Gy_reel, Gz_reel;         //Variables pour le calcul des valeurs réelle du gyromètre

/* -- Déclaration des fonctions utiles au programme -- */
 SoftwareSerial Bluetooth(BtTxPin,BtRxPin);        //Configure la fonction série émulé Bluetooth avec les broches (Rx,Tx)


/* ---------------------------------------------------- *
 *  ROUTINE D'INITIALISATION  (exécutée une seule fois) *
 * ---------------------------------------------------- */
void setup()
{
/* -- Configuration des broches en ENTREE ou SORTIE -- */

/* -- configuration des fonctions spéciales nécessaires au programme */
  //Serial.begin(115200);                          //Initialisation de la bibliothèque Moniteur série
  Wire.begin();

  Bluetooth.begin(9600);                           //Réglage de la vitesse de transmission
  Bluetooth.setTimeout(200);                       //Timeout en ms (temps au bout duquel la réception stoppe si aucune donnée n'est reçue)##

  Configuration_Bluetooth_GROVE();                 //appel de la routine de configuration de liaison série bluetooth (module grove) voir "4 ROUTINES"#
  delay(500);
   
  reglage_LM6DS3();                                //Appel de la routine de réglage du capteur
  delay(200);

  etalonnage();                                    //Appel de la routine pour l'étalonnage du capteur au début du programme

}

/* -------------------------------------------- *
 *  BOUCLE PRINCIPALE (exécutée en permanence)  *
 * -------------------------------------------- */
void loop()
{ 
/* -- Lecture des valeurs de entrées -- */
  lecture_valeurs_brutes();
  
/* -- Calcul des valeurs réelles d'accélérations en g et de vitesses angulaires en °/s en fonction des valeurs brutes -- 
 *  Les valeurs brutes sont codées sur 16 bits (0 et 65536). Les sensibilités (en g/LSB ou dps/LSB) dépendendent de la valeur pleine échelle
 */ 
  Ax_reel = float(ax_brut - ax_offset) * sensi_acc / 1000000;
  Ay_reel = float(ay_brut - ay_offset) * sensi_acc / 1000000; 
  Az_reel = float(az_brut - az_offset) * sensi_acc / 1000000;
  Gx_reel = float(gx_brut - gx_offset) * sensi_gyr / 1000000;
  Gy_reel = float(gy_brut - gy_offset) * sensi_gyr / 1000000;
  Gz_reel = float(gz_brut - gz_offset) * sensi_gyr / 1000000;


/* -- Envoie du message sur la liaison Bluetooth pour l'application smartphone qui doit ressembler 
 *    à "Debut,ax:±##.##;g,ay:±##.##;g,ay:±##.##;g,gx:±####;deg/s,gy:±####;deg/s,gz:±####;deg/s,Fin  "         -- 
 */
  String chaine = "Debut,ax:" + String(Ax_reel,1) + ":g,ay:" + String(Ay_reel,1) + ":g,az:" + String(Az_reel,1) + ":g,gx:" + String(Gx_reel,0) + ":deg/s,gy:" + String(Gy_reel,0) + ":deg/s,gz:" + String(Gz_reel,0) + ":deg/s,Fin";
  Bluetooth.print(chaine);

}

/* ------------------------------------------------- *
*   ROUTINE DE LECTURE DES VALEURS BRUTES DU CAPTEUR *
* -------------------------------------------------- */
void lecture_valeurs_brutes()
{ 
  //Variable locale pour déclarer un pointeur vers un tableau dynamique de 6 valeurs
  uint8_t pTampon[6];

  LECTURE_REGISTRES( 0x6A , 0x28 , 6 , pTampon );
  ax_brut= (pTampon[0] << 8 | pTampon[1] );
  ay_brut= (pTampon[2] << 8 | pTampon[3] );
  az_brut= (pTampon[4] << 8 | pTampon[5] ); 

  LECTURE_REGISTRES( 0x6A , 0x22 , 6 , pTampon );
  gx_brut=( pTampon[0] << 8 | pTampon[1] );
  gy_brut=( pTampon[2] << 8 | pTampon[3] );
  gz_brut=( pTampon[4] << 8 | pTampon[5] );
}
/* -------------------------------------------- *
 * ROUTINE DE LECTURE DES VALEUR DE X REGISTRES *
 * -------------------------------------------- */
/* Remarque *pTableau correspond au pointeur vers le tableau voir https://www.locoduino.org/spip.php?article106 */
void LECTURE_REGISTRES(uint8_t Add_module, uint8_t Add_Registre, uint8_t Nregistres, uint8_t * pTableau) //Lecture, sur la liaison l'I2C, de plusieurs registres successifs
{
  Wire.beginTransmission(Add_module);                                                  //Initialisation de la connexion avec le module (Adresse : du module)
  Wire.write(Add_Registre);                                                            //Définition de l'adresse du PREMIER registre
  Wire.endTransmission();                                                              //Arrêt de l'envoie d'informations sur l'I2C
   
  Wire.requestFrom(Add_module, Nregistres);                                            //Requette de lecture des n octects
  uint8_t index=0;                                                                     //initilisation d'une variable de comptage
  while (Wire.available()) pTableau[index++]=Wire.read();                              //Tant que des octects sont disponibles enregistrer la valeur de chaque octect dans le tableau et incrémenter l'index
}
   
/* ------------------------------------------------ *
 * ROUTINE D'ECRITURE D'UNE VALEUR DANS UN REGISTRE *
 * ------------------------------------------------ */
void ECRITURE_REGISTRE(uint8_t Add_module, uint8_t Add_Registre, uint8_t Valeur)     //Ecriture, sur la liaison l'I2C, d'un une valeur d'un octect dans un registre
{ 
  Wire.beginTransmission(Add_module);                                                  //Initialisation de la connexion avec le module (Adresse : du module)
  Wire.write(Add_Registre);                                                            //Définition de l'adresse du registre
  Wire.write(Valeur);                                                                  //Ecriture de la valeur
  Wire.endTransmission();                                                              //Arrêt de l'envoie d'informations sur l'I2C
}

/* ------------------------------------------------------------------------ *
 * ROUTINE DE REGLAGE DE LA CENTRALE INERTIELLE STMicroelectronics LSM6DS3  *
 *      Utilise la fonction de communication I2C ECRITURE_REGISTRE()        *
 * ------------------------------------------------------------------------ */
void reglage_LM6DS3()
{
  /* -- REGISTRE : FUNC_CFG_ACCESS (01h) : Désactive (/active) les fonctions embarquées */
  ECRITURE_REGISTRE( 0x6A , 0x01 , 0b00000000 );
  /* -- REGISTRE : ORIENT_CFG_G (0Bh) Angular rate sensor sign and orientation register */
  ECRITURE_REGISTRE( 0x6A , 0x0B , 0b00000000 );
  /* -- REGISTRE : CTRL1_XL (10h) Linear acceleration sensor control register 1 */
  ECRITURE_REGISTRE( 0x6A , 0x10 , 0b01100100 ); /* ±16g et autres réglages par défaut */
  /* -- REGISTRE : CTRL2_G (11h) Angular rate sensor control register 2 */
  ECRITURE_REGISTRE( 0x6A , 0x11 , 0b01101100 ); /* ±2000°/s et autres réglages par défaut */
  /* -- REGISTRE : CTRL3_C (12h) Control register 3 */
  ECRITURE_REGISTRE( 0x6A , 0x12 , 0b00000110 );
  /* -- REGISTRE : CTRL4_C (13h) Control register 4 */
  ECRITURE_REGISTRE( 0x6A , 0x13 , 0b10000000 );
  /* -- REGISTRE : CTRL5_C (14h) Control register 5 */
  ECRITURE_REGISTRE( 0x6A , 0x14 , 0b00000000 );
  /* -- REGISTRE : CTRL6_C (15h) Control register 6 */
  ECRITURE_REGISTRE( 0x6A , 0x15 , 0b00000000 );
  /* -- REGISTRE : CTRL7_G (16h) Angular rate sensor control register 7 */
  ECRITURE_REGISTRE( 0x6A , 0x16 , 0b00000000 );
  /* -- REGISTRE : CTRL8_XL (17h) Linear acceleration sensor control register 8 */
  ECRITURE_REGISTRE( 0x6A , 0x17 , 0b00000000 );     
  /* -- REGISTRE : CTRL9_XL (18h) Linear acceleration sensor control register 9 (r/w) */
  ECRITURE_REGISTRE( 0x6A , 0x18 , 0b00111000 );
  /* -- REGISTRE : CTRL10_C (19h) Control register 10 (r/w) */
  ECRITURE_REGISTRE( 0x6A , 0x19 , 0b00111000 );
}

/* ------------------------------------------------ *
 *     ROUTINE D'ETALONNAGE = calcul des offsets    *
 * ------------------------------------------------ */
void etalonnage()
{
  int nombre_iterations = 1000;                                                        //Variable pour définir le nombre d'itération du calcul
  
  ax_offset = 0;
  ay_offset = 0;
  az_offset = 0;
  gx_offset = 0;
  gy_offset = 0;
  gz_offset = 0;
  
  
  for (int i = 1; i <= nombre_iterations; i++) 
  {
    lecture_valeurs_brutes(); 
    ax_offset += ax_brut;                                                              //Sommation de chaque valeur pour le calcul de la moyenne
    ay_offset += ay_brut;
    az_offset += az_brut;
    gx_offset += gx_brut;
    gy_offset += gy_brut;
    gz_offset += gz_brut;
  }
  
  ax_offset /= nombre_iterations;  //Calcul de la moyenne des valeurs mesurées pendant les itérations
  ay_offset /= nombre_iterations;
  az_offset /= nombre_iterations;
  az_offset -= 32767 / pleine_echelle_acc;
  gx_offset /= nombre_iterations;
  gy_offset /= nombre_iterations;
  gz_offset /= nombre_iterations;

}

/* ------------------------------------------------------------------ *
    ROUTINE DE CONFIGURATION DE LA LIAISON SERIE POUR LA CARTE Grove
   ------------------------------------------------------------------ */
void Configuration_Bluetooth_GROVE()
{   
  Bluetooth.begin(9600);               //Réglage de la vitesse de transmission
  Bluetooth.print("AT");               //Démarrage des commandes AT (possible uniquement quand le bluetooth est déconnecté)
  delay(400);                          //Temporisation nécessaire pour les commandes AT
  Bluetooth.print("AT+AUTH1");         //Commande AT d'activation de l'autentification           
  delay(400);
  Bluetooth.print("AT+NAMEIMU_Bluetooth"); //Commande AT de changement de nom AT+NAME suivi du nom sans espace ! ATTENTION ! Pensez à modifier le numéro pour chaque module différent.
  delay(400);
}
