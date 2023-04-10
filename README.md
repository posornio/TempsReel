# Dumber

Se rendre dans le répertoire `software/rasberry/superviseur-robot` pour accéder au travail et lire le fichier `Readme.md` de ce réprtoire pour des explications du code

On a implementé les taches suivantes:
  - Ajout d'un message qui notifie la perte de connection avec le moniteur. De plus, le robot, la caméra et le server seront arretés et on reviendra dans le même état qu'au démarage du robot. 
  - Ajout d'un mode de démarrage avec un watchdog. Le watchdog est rechargé toutes les secondes, et si le compteur expire, le robot sera arreté et le robot devra être rédemarré manuellement.  
  - Le niveau de batterie est mis a jour toutes les 500 ms. 
  - Le robot peut être localisé dans la arène et on peut dessiner sa localisation sur les captures de la caméra. Sa localisation est mis a jour toutes les 100 ms.  

## Repertoires
- hardware : contient les plans pour la partie mecanique du robot et de son chargeur, ainsi que les plans de conception des PCB du robot, du chargeur, de l'adaptateur Xbee pour la raspberry  et les plans des CAP du robot
- software: rassemble les parties logicielles du robot, du chargeur, les bibliotheques et superviseur coté raspberry et l'interface Web
- doc: contient les sujets de TD et TP
- aruco_markers: Script de generation des tags (aruco) utilisés sur les robots

