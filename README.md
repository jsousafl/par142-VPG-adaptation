# PAr 142 : Apprentissage d’un bras robotique : saisies difficiles de pièces en vrac par déplacement préalable intelligent, basé sur les méthodes de Deep Learning

Auteurs:
 - João Vitor Sousa Floriano (joao.sousa-floriano@ecl17.ec-lyon.fr)
 - Lucas Neto Nakadaira (lucas.neto-nakadaira@ecl17.ec-lyon.fr)

Laboratoires : Laboratoire de recherche en image et système d’information (LIRIS)

Equipe de recherche : Équipe LIRIS / Imagine (https://projet.liris.cnrs.fr/imagine/)

Encadrant(s):
- Emmanuel Dellandrea (emmanuel.dellandrea@ec-lyon.fr) 
- Maxime Petit (maxime.petit@ec-lyon.fr)


Position du problème :

Le but de ce projet est d’utiliser un bras robotique (l'UR3 d'Universal Robot https://www.universal-robots.com/fr/produits/robot-ur3/) pour saisir et trier des pièces en vrac en utilisant une méthode baséesur le Deep Learning, appelée Visual Pushing and Grasping (VPG). Il s'agit d'une méthode d’apprentissage permettant au robot de saisir des objets initialement difficilement manipulables en les déplaçant légèrement dans un premier temps.

La première phase consistera à se familiariser avec le code VPG actuellement développé par l’université de Princeton et disponible sur github (https://github.com/andyzeng/visual-pushing-grasping). Cela nécessitera d’étudier des outils informatiques comme PyTorch (Deep Learning), V-Rep (simulateur) et les algorithmes pour contrôler le robot UR3.
 

Dans un second temps, les étudiants partiront du code existant pour le faire fonctionner avec l’environnement disponible au laboratoire de l’Ecole Centrale de Lyon. En particulier, des modifications seront nécessaire pour 1) adapter le code contrôlant le robot (UR5 pour Princeton, UR3 pour ECL), 2) vérifier que les procédures de calibration sont adaptées avec ces différents changements et 3) testerl’apprentissage en utilisant les objets disponibles à l’ECL.

En fonction de l’avancée, les étudiants pourront être amenés à développer des applications supplémentaires, par exemple en permettant de trier les pièces dans différentes boîtes de tri en fonction de leur forme ou couleur, ou encore en initialisant et résolvant le jeu des Tours de Hanoi.

Références:

Zeng, Andy, Shuran Song, Stefan Welker, Johnny Lee, Alberto Rodriguez, and Thomas Funkhouser. "Learning Synergies between Pushing and Grasping with Self-supervised Deep Reinforcement Learning." Accepted at IEEE_IROS 2018 (October), Madrid, Spain._


