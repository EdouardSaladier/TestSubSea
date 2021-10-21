#include "ros/ros.h"
#include <iostream>
#include <common_utility_package/robot_cmd.h>
#include <common_utility_package/robot_data.h>

using namespace std;

common_utility_package::robot_cmd msg_out;
float c_l = 0; //Commande longitudinale
float c_v = 0; //Commande verticale
float c_r = 0; //Commande rotation
float c_p = 0; //Commande puissance

void clamp(float& val, float lower, float upper) {
	val = max(lower, min(val, upper)); //Fonction qui permet de délimiter une valeur d'entrée entre deux valeurs
}

void robot_data_callback(const common_utility_package::robot_data& msg_in) {
	//Lorsque l'on reçoit les pourcentages du robot, on les affiche sur le terminal
	cout << endl;
	cout << "Message du robot \"robot_data\" reçu:" << endl;
	cout << "Pourcentage du moteur gauche: "<< msg_in.p_mg <<endl;
	cout << "Pourcentage du moteur droit: "<< msg_in.p_md <<endl;
	cout << "Pourcentage du moteur vertical: "<< msg_in.p_mv <<endl;
	cout << endl;
}

int main(int argc, char **argv)
{
	//Initialisation de ros
	ros::init(argc, argv, "station_main");
	ros::NodeHandle n;
	//Initialisation du publisher et du suscriber
	ros::Publisher robot_cmd_pub = n.advertise<common_utility_package::robot_cmd>("robot_cmd", 1000);
	ros::Subscriber robot_data_sub = n.subscribe("robot_data", 1000, robot_data_callback);
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		//On demande à l'utilisateur via le terminal d'entrer les commandes
		cout << "Entrez la commande longitudinale (de -10 à 10, 0 par défaut): "<<endl;
		cin >> c_l;
		clamp(c_l, -10, 10);
		cout << "Commande longitudinale: "<< c_l<<endl;
		msg_out.c_l = c_l;

		cout << "Entrez la commande verticale (de -10 à 10, 0 par défaut): "<<endl;
		cin >> c_v;
		clamp(c_v, -10, 10);
		cout << "Commande verticale: "<< c_v<<endl;
		msg_out.c_v = c_v;

		cout << "Entrez la commande de rotation (en degré, sens horaire, 0 dans le sens du robot, de -180 à 180, 0 par défaut): "<<endl;
		cin >> c_r;
		clamp(c_r, -180, 180);
		cout << "Commande de rotation: "<< c_r<<endl;
		msg_out.c_r = c_r;

		cout << "Entrez la commande de puissance (de 0 à 100%, 0 par défaut): "<<endl;
		cin >> c_p;
		clamp(c_p, 0, 100);
		cout << "Commande de puissance: "<< c_p<<endl;
		msg_out.c_p = c_p;

		//Une fois les commandes entrées, on les publie sur le message robot_cmd
		robot_cmd_pub.publish(msg_out);
		cout << "Message \"robot_cmd\" envoyé au robot" <<endl;

		//On attends 0.5 secondes le temps de recevoir le message "robot_data" de retour du robot
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}

	return 0;
}
