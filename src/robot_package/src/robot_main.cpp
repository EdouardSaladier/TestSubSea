#include "ros/ros.h"
#include <iostream>
#include <common_utility_package/robot_cmd.h>
#include <common_utility_package/robot_data.h>

using namespace std;

common_utility_package::robot_data msg_out; //Définition du message msg_out sous format "robot_data"
bool msg_send = false; //Pour éviter d'envoyer en permanence la même commande à la station, on utilise un booléen pour ne l'envoyer qu'à chaque callback
float p_mg = 0; //Pourcentage du moteur gauche
float p_md = 0; //Pourcentage du moteur droit
float p_mv = 0; //Pourcentage du moteur vertical

void clamp(float& val, float lower, float upper) {
	val = max(lower, min(val, upper)); //Fonction qui permet de délimiter une valeur d'entrée entre deux valeurs
}

void robot_cmd_callback(const common_utility_package::robot_cmd& msg_in)
{
	cout << "Message de commande station \"robot_cmd\" reçu: "<<endl;
	//Quand on reçoit une commande de la station sous la forme d'un message "robot_cmd", on la converti en pourcentages de puissance
	float c_l = msg_in.c_l; //Commande longitudinale
	float c_v = msg_in.c_v; //Commande verticale
	float c_r = msg_in.c_r; //Commande rotation
	float c_p = msg_in.c_p; //Commande puissance

	//On passe d'une commande longitudinale de 0 à 10 à un pourcentage de 0 à 100 pour les deux moteurs
	p_mg = c_l*10; 
	p_md = c_l*10;

	//On passe d'une commande verticale de 0 à 10 à un pourcentage de 0 à 100 pour le moteur vertical
	p_mv = c_v*10;


	if (c_l == 0) { 
	//Si le robot est n'a pas de commande longitudinale, on pivote sur place
		c_r=(c_r/180)*100; //On passe d'un angle de 0 à 180 degré en un pourcentage allant de 0 à 100
		//Pour tourner à droite, il faut faire avancer le moteur gauche et reculer le moteur droit
		p_mg=c_r; //Le moteur gauche prends donc le signe de l'angle
		p_md=-c_r; //Et le moteur droit son opposé
	} else {
		//Sinon, pour ne pas interrompre la commande longitudinale, on se contente de réduire la vitesse d'un des deux moteurs
		c_r=(c_r/90)*c_l*10; //On passe d'un angle de 0 à 180 degré à un pourcentage de la commande en vitesse, pour un angle de 90° ou plus, un des moteurs fonctionnera dans le sens inverse
		if (c_r > 0) { //Si on veut tourner à droite, on réduit la vitesse du moteur droit
			p_md=p_md-c_r;
		}
		if (c_r < 0) { //Si on veut tourner à gauche, on réduit la vitesse du moteur gauche
			p_mg=p_mg-c_r;
		}
		//Si l'angle en entrée est nul, la commande longitudinale n'est pas affectée.
	}

	//Enfin, une fois la rotation prise en compte, on prends en compte la commande de puissance
	c_p = c_p/100; //On divise par 100 le pourcentage
	//Et l'on multiplie tous les pourcentages par le facteur de puissance
	p_mg = p_mg*c_p;
	p_md = p_md*c_p;
	p_mv = p_mv*c_p;

	//On délimite les variables par sécurité
	clamp(p_mg, -100, 100);
	clamp(p_md, -100, 100);
	clamp(p_mv, -100, 100);

	//Une fois toutes les commandes calculées, on les entre dans le message
	msg_out.p_mg = p_mg;
	msg_out.p_md = p_md;
	msg_out.p_mv = p_mv;

	cout << "Pourcentage du moteur gauche: "<< msg_out.p_mg <<endl;
	cout << "Pourcentage du moteur droit: "<< msg_out.p_md <<endl;
	cout << "Pourcentage du moteur vertical: "<< msg_out.p_mv <<endl;

	//On définit le booléén msg_send à true pour envoyer le message dans la boucle principale
	msg_send = true;

	
}


int main(int argc, char **argv)
{
	//Initialisation de ros
	ros::init(argc, argv, "robot_main");
	ros::NodeHandle n;
	//Initialisation du publisher et du suscriber
	ros::Publisher robot_data_pub = n.advertise<common_utility_package::robot_data>("robot_data", 1000);
	ros::Subscriber robot_cmd_sub = n.subscribe("robot_cmd", 1000, robot_cmd_callback);
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		//Si l'on a calculé les puissances moteur, le booléen msg_send a la valeur true, donc on publie les puissances sur le message robot_data
		if (msg_send==true) {
			robot_data_pub.publish(msg_out);
			cout << "Message \"robot_data\" envoyé à la station " <<endl;
			//On remet msg_send à false pour ne publier les pourcentages qu'une seule fois à chaque calcul
			msg_send = false;
		}
		ros::spinOnce();
    	loop_rate.sleep();
	}
	return 0;
}
