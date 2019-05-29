# PAr 142 : Apprentissage d’un bras robotique : saisies difficiles de pièces en vrac par déplacement préalable intelligent, basé sur les méthodes de Deep Learning

ATTENTION: This project is based entirely on https://github.com/andyzeng/visual-pushing-grasping

Auteurs:
 - João Vitor Sousa Floriano (joao.sousa-floriano@ecl17.ec-lyon.fr)
 - Lucas Neto Nakadaira (lucas.neto-nakadaira@ecl17.ec-lyon.fr)

Laboratoires : Laboratoire de recherche en image et système d’information (LIRIS)

Equipe de recherche : Équipe LIRIS / Imagine (https://projet.liris.cnrs.fr/imagine/)

Encadrant(s):
- Emmanuel Dellandrea (emmanuel.dellandrea@ec-lyon.fr) 
- Maxime Petit (maxime.petit@ec-lyon.fr)


Problématique :

Le but de ce projet est d’utiliser un bras robotique (l'UR3 d'Universal Robot https://www.universal-robots.com/fr/produits/robot-ur3/) pour saisir et trier des pièces en vrac en utilisant une méthode baséesur le Deep Learning, appelée Visual Pushing and Grasping (VPG). Il s'agit d'une méthode d’apprentissage permettant au robot de saisir des objets initialement difficilement manipulables en les déplaçant légèrement dans un premier temps.

La première phase consiste à se familiariser avec le code VPG actuellement développé par l’université de Princeton et disponible sur github (https://github.com/andyzeng/visual-pushing-grasping). Cela nécessitera d’étudier des outils informatiques comme PyTorch (Deep Learning), V-Rep (simulateur) et les algorithmes pour contrôler le robot UR3.
 

Dans un second temps, les étudiants partiront du code existant pour le faire fonctionner avec l’environnement disponible au laboratoire de l’Ecole Centrale de Lyon. En particulier, des modifications seront nécessaire pour:

1) Adapter le code contrôlant le robot (UR5 pour Princeton, UR3 pour ECL)
2) Vérifier que les procédures de calibration sont adaptées avec ces différents changements
3) Tester l’apprentissage en utilisant les objets disponibles à l’ECL.

ENGLISH VERSION:

Problematic:

The goal of this project is to use a robotic arm (the Universal Robot UR3 https://www.universal-robots.com/en/products/robot-ur3/) to grab and sort loose parts into using a method based on Deep Learning, called Visual Pushing and Grasping (VPG). This is a learning method that allows the robot to grasp objects initially difficult to manipulate by moving them slightly at first.

The first phase of this project consisted in become familiar with the VPG code currently developed by Princeton University and available on github (https://github.com/andyzeng/visual-pushing-grasping) which required studying computer tools like PyTorch (Deep Learning), V-Rep (simulator) and algorithms to control the UR3 robot.

In a second step, the students will start from the existing code to make it work with the available environment in the laboratory of the Ecole Centrale de Lyon. In particular, modifications were necessary to:

1) Adapt the code controlling the robot (UR5 for Princeton, UR3 for ECL)
2) Check that the calibration procedures are adapted with these different changes
3) Test the learning algorithm using the objects available at the ECL.

Références:

Zeng, Andy, Shuran Song, Stefan Welker, Johnny Lee, Alberto Rodriguez, and Thomas Funkhouser. "Learning Synergies between Pushing and Grasping with Self-supervised Deep Reinforcement Learning." Accepted at IEEE_IROS 2018 (October), Madrid, Spain._


