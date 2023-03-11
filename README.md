# IMU_Bluetooth
<p align="center">
<img src="Platine IMU_Bleutooth.jpg" alt="Moteur à l'arret" height=200>
<img src="Platine IMU_Bluetooth 2.jpg" alt="Moteur en rotation" height=200> 
</p>

## Présentation

Platine expérimentale pour afficher, en temps réel, les données lues par une centrale inertielle incluant un accéléromètre et un gyromètre.
Il est possible de piloter la fréquence avec un potentiomètre linéaire.
L'écran LCD permet d'afficher la fréquence de consigne (en tr/min) et la mesure de la vitesse angulaire en °/s.
Les données de la centrale sont envoyées en Bluetooth vers un smartphone et une application spécifique.

Une vidéo du programme en situation est disponible  [youtu.be/u4rxh-nPM8k](https://youtu.be/_Ql979rKE8E)

## Matériel utilisé
### Centrale inertielle :
- Un Arduino UNO
- Une centrale inertielle Grove_IMU_6 DOF : [wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope](https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/) 
- Un module Bluetooth Grove - Serial Bluetooth v3.0 [wiki.seeedstudio.com/Grove-Serial_Bluetooth_v3.0](https://wiki.seeedstudio.com/Grove-Serial_Bluetooth_v3.0/)
- Une pile 9V pour l'alimentation

### Banc moteur
- Un Arduino UNO
- Un écran LCD RGB BLACKLIGHT : [wiki.seeedstudio.com/Grove-LCD_RGB_Backlight](https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/)
- Un potentiomètre linéaire Grove Slide Potentiometer : [wiki.seeedstudio.com/Grove-Slide_Potentiometer](https://wiki.seeedstudio.com/Grove-Slide_Potentiometer/)
- Un Driver moteur Sparkfun TB6612FNG : [sparkfun.com/tutorials/tb6612fng-hookup-guide](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide?_ga=2.155362748.600943259.1678452372-1239022160.1678452372)
- Un moto réducteur POLOLU #4753 avec une réduction de 50:1 et 64CPR (rise and fall) : [pololu.com/product/4753](https://www.pololu.com/product/4753)
- Une équerre de renfort Zingué 100x100x22mm
- Une interface de fixation du motoréducteur sur l'équerre, imprimée en 3D (voir fichier stl)
- Un plateau à fixer sur la sortie du motoréducteur imprimé en 3D (voir fichier stl)
- Une alimentation CC 12V 3A ou plus

### Application Android
L'application permet d'afficher en temps réel :
- les 3 valeurs Ax, Ay et Az de l'accéléromètre
- Les 3 valeurs Gx, Gy et Gz du gyromètre

L'Arduino envoie, par la liaison bluetooth, une chaine de caractères :  
"Debut,ax:±##.##;g,ay:±##.##;g,ay:±##.##;g,gx:±####;deg/s,gy:±####;deg/s,gz:±####;deg/s,Fin  "  
Dont les ### sont les valeurs de chaque composante.

L'application a été développé avec APP INVENTOR :  [appinventor.mit.edu/](https://gallery.appinventor.mit.edu/?galleryid=7bed003b-4152-40f3-b9e8-77ac0583e04f)
Le fichier apk est fourni dans les fichiers


### Explications :
Le principe est le suivant :
- Le µC de la centrale envoi en permanence les 6 composantes d’accélérations et de vitesses angulaires en temps réel par l’intermédiaire du Bluetooth
- Le smartphone affiche les 6 valeurs.
- On peut contrôler « précisément » la fréquence de rotation du motoréducteur et lire sur la platine la valeur « réelle » en °/S
- Il est donc possible de vérifier les valeurs mesurées par le gyromètre en °/S et éventuellement celle de l’accéléromètre (accélération centripète) a= ω²∙ Rayon


Pour tout renseignement complémentaire [gael.balduini@gmail.com](mailto:gael.balduini@gmail.com)

