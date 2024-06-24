# Humi'Varium
Humi'Varium est un projet de gestion de l'humidité et de la température utilisant divers composants matériels et protocoles de communication. Ce README fournit une vue d'ensemble de l'architecture technique, des composants utilisés, et des instructions pour configurer et exécuter le projet.

# Table des matières
Aperçu
Architecture technique
Composants utilisés
Installation
Utilisation

Aperçu
Humi'Varium est conçu pour surveiller et afficher les niveaux de température et d'humidité en temps réel. Le système utilise des capteurs connectés à une carte Nucleo L152, et les données sont affichées via le logiciel Tera Term.

# Architecture technique

GPIO
Utilisation: Boutons en interruption pour changer les données affichées.

Timer
Utilisation: Gestion d'un compteur du temps de fonctionnement d'Humi'Varium sur Tera Term.

ADC
Utilisation: Affichage de la valeur du potentiomètre pour vérifier son bon fonctionnement.

UART
Utilisation: Affichage des valeurs de températures et d'humidité sur le logiciel Tera Term.

I2C / SPI
Utilisation: Communication entre le shield IKS01A3 et la carte Nucleo L152.

IKS01A3

Utilisation: Deux capteurs pour mesurer :

Température
Humidité

#Composants utilisés
Carte Nucleo L152
Shield IKS01A3
Capteurs de température et d'humidité
Potentiomètre
Boutons pour interruptions
Logiciel Tera Term

# Installation

Prérequis
Tera Term
STM32CubeMX
STM32CubeIDE
Câbles de connexion

# Étapes d'installation
Clonez le repository GitHub sur votre machine locale
Copier le code
git clone https://github.com/votre-utilisateur/HumiVarium.git
cd HumiVarium
Ouvrez le projet dans STM32CubeIDE.
Configurez les paramètres du microcontrôleur en utilisant STM32CubeMX.
Compilez et téléversez le code sur la carte Nucleo L152.

# Utilisation
Connectez les capteurs et le potentiomètre à la carte Nucleo L152 selon le schéma de câblage fourni.
Ouvrez Tera Term et connectez-vous à la carte Nucleo via UART.
Utilisez les boutons pour changer les données affichées.
Surveillez les valeurs de température et d'humidité en temps réel sur Tera Term.
