### Tache Watchdog

Une tache `StartWD´ qui propose un mode de démarrage avec un watchdog a été crée. Le watchdog est rechargé toutes les secondes, et si le compteur expire, le robot sera arreté et le robot devra être rédemarré manuellement. Cette tache est liée au sémaphore 
`sem_start_wd´ et utilise le boolean `boolWD´ qui depend des messages ´MESSAGE_ROBOT_START_WITH_WD´ et MESSAGE_ROBOT_START_WITHOUT_WD´. 

### Tache Batterie 
Le niveau de batterie est mis a jour toutes les 500 ms.



### Tache Camera

Une tache `SendCameraImages` pour gérer les opérations sur la caméra associé au sémaphore `sem_camera` a été crée.
Cette tâche périodique envoi une image toutes les 100ms. 

L'utilisation de booléens `testArena`et `addArena` nous permet de gérer l'ajout ou non de l'arène.
`testArena` indique si nous devons tester l'arène sur l'image. Il est déclenché lors de la réception d'un message de demande de confirmlation de l'arène et
passe à false lorsqu'une arène est détectée ou que l'utilisateur confirme / infirme l'arène.
`addArena` indique si l'on doit ajouter l'arène sur l'image. Il passe à true dès que l'utilisateur confirme l'arène et passe à false quand celui-ci l'infirme. 

L'utilisation du booléen `computePosition` gère de la même manière l'ajout ou non de la position du robot sur l'image. Le robot peut être localisé dans la arène et on peut dessiner sa localisation sur les captures de la caméra. Sa localisation est mis a jour toutes les 100 ms.


Un mutex `mutex_camera` a été créé pour gérer l'accès à la caméra.

### Tache Move

Nous avons amélioré la tâche move du déplacement du robot. En effet la tâche envoyait avant modifications toujours un message au robot même si le message de déplacement est le même.
Nous avons fixé le problème en sauvegardant le mouvement précédent dans la variable `last_move` et en n'envoyant un message au robot que si le `move` courant est différent du `last_move`.
Cela permet de gagner en performances et évite le bug existant de la position stop.

### Méthode Verification Communication

Nous avons créé la méthode `checkCom` qui vérifie que la communication entre le robot et le superviseur est bien opérationnelle.
A chaque appel de la méthode `Write` de la classe robot, la méthode `checkCom` est appelé avec le message renvoyé par `write`.
Si ce message est un message d'erreur ou un timeout, le compteur `errorcpt` st incrémenté de 1, sinon il est remis à 0.
Lorsque le comptur dépasse 3, la communication est considérée comme perdue, elle est donc fermé et on se replace dans un état initil prmettant de relancer la communication.

### Méthode de notification de la perte de communication 

Nous avons ajouté un message qui notifie la perte de connection avec le moniteur en utilisant le message `MESSAGE_MONITOR_LOST´. De plus, le robot, la caméra et le server seront arretés et on reviendra dans le même état qu'au démarage du robot. 
