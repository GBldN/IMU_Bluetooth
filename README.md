# IMU_Bluetooth

<center> <img src="platine%20Banc%20moteur.jpg" alt="texte alternatif" height=200 aling="center"> </center>

## Présentation
Platine expérimentale pour afficher, en temps réel, les données lues par une centrale inertielle incluant un accéléromètre et un gyromètre.
Il est possible de piloter la fréquence avec un potentiomètre linéaire.
L'écran LCD permet d'afficher la fréquence de consigne (en tr/min) et la mesure de la vitesse angulaire en °/s.
Les données de la centrale sont envoyées en Bluetooth vers un smartphone et une application spécifique.

Une vidéo du programme en situation est disponible  https://youtu.be/u4rxh-nPM8k

## Matériel utilisé
### Centrale inertielle :
- Un Arduino UNO
- Une centrale inertielle Grove_IMU_6 DOF : https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/ 
- Un module Bluetooth Grove - Serial Bluetooth v3.0 https://wiki.seeedstudio.com/Grove-Serial_Bluetooth_v3.0/

### Banc moteur
- Un Arduino UNO
- Un écran LCD RGB BLACKLIGHT : https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/
- Un potentiomètre linéaire Grove Slide Potentiometer : https://wiki.seeedstudio.com/Grove-Slide_Potentiometer/
- Un Driver moteur Sparkfun TB6612FNG : https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide?_ga=2.155362748.600943259.1678452372-1239022160.1678452372
- Un moto réducteur POLOLU #4753 avec une réduction de 50:1 et 64CPR (rise and fall) : https://www.pololu.com/product/4753

### Application Android
L'application permet d'afficher en temps réel :
- les 3 valeurs Ax, Ay et Az de l'accéléromètre
- Les 3 valeurs Gx, Gy et Gz du gyromètre

L'Arduino envoie ne bluetooth une chaine de caractères : "Debut,ax:±##.##;g,ay:±##.##;g,ay:±##.##;g,gx:±####;deg/s,gy:±####;deg/s,gz:±####;deg/s,Fin  "
Dont les ### sont les valeurs de chaque composante.

L'application a été développé avec APP INVENTOR :  https://gallery.appinventor.mit.edu/?galleryid=7bed003b-4152-40f3-b9e8-77ac0583e04f
Le fichier apk est fourni

Pour tout renseignement complémentaire gael.balduini@gmail.com

