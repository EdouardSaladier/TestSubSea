# TestSubSea

Test technique contrôle ROV 

## Installation

Effectuer les commandes suivantes dans l'ordre:
```
git clone https://github.com/EdouardSaladier/TestSubSea
cd TestSubSea
catkin_make
```

## Utilisation

Sur un terminal, lancer la commande `roscore`.
Sur un autre terminal, aller dans le repertoire `TestSubSea`, et lancer les commandes:
```
source devel/setup.bash
rosrun robot_package robot_main
```
Ensuite, sur un dernier terminal, toujours dans le répertoire `TestSubSea`, lancer les commandes:
```
source devel/setup.bash
rosrun station_package station_main
```
Enfin, il suffit simplement d'entrer les valeurs des commandes désirées en suivant les instructions s'affichant sur le terminal. Une fois les 4 commandes entrées, le terminal affichera les commandes en pourcentages des trois moteurs renvoyées par le robot.

## Structure

Ce répertoire possède 3 packages différentes:
* "common_utility_package" sur lequel se trouve la définition des messages de type "robot_cmd" et "robot_data", utilisés par les deux autres packages
* "robot_package" sur lequel se trouve le noeud "robot_main", qui reçoit un message "robot_cmd" de la station, calcule la commande et renvoie à la station un message de type "robot_data"
* "station_package" sur lequel se trouve le noeud "station_main", qui sert d'interface à l'utilisateur, celui-ci entre les commandes, la station les envoie au robot sous format "robot_cmd", et reçoit du robot les commandes en pourcentages des moteurs sous format "robot_data"