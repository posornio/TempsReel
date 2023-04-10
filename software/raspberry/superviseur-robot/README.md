### Tache Camera

Une tache `SendCameraImages` pour gérer les opérations sur la caméra associé au sémaphore `sem_camera` a été crée.
Cette tâche périodique envoi une image toutes les 100ms. 

L'utilisation de booléens `testArena`et `addArena` nous permet de gérer l'ajout ou non de l'arène.
`testArena` indique si nous devons tester l'arène sur l'image. Il est déclenché lors de la réception d'un message de demande de confirmlation de l'arène et
passe à false lorsqu'une arène est détectée ou que l'utilisateur confirme / infirme l'arène.
`addArena` indique si l'on doit ajouter l'arène sur l'image. Il passe à true dès que l'utilisateur confirme l'arène et passe à false quand celui-ci l'infirme. 

L'utilisation du booléen `computePosition` gère de la même manière l'ajout ou non de la position du robot sur l'image.

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
