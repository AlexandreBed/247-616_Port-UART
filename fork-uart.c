// /**
//  * @file    SerialPort_read.c
//  * 
//  * @brief Serial Port Programming in C (Serial Port Read)  
//  * Non Cannonical mode   
//  * Sellecting the Serial port Number on Linux   
//  * /dev/ttyUSBx - when using USB to Serial Converter, where x can be 0,1,2...etc 
//  * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc  
//  * termios structure -  /usr/include/asm-generic/termbits.h  
//  * use "man termios" to get more info about  termios structure
//  * @author  Kevin Cotton
//  * @date    2024-08-02
//  */	
// #define _GNU_SOURCE

// #include <stdio.h>
// #include <fcntl.h>   // File Control Definitions
// #include <termios.h> // POSIX Terminal Control Definitions 
// #include <unistd.h>  // UNIX Standard Definitions 
// #include <errno.h>   // ERROR Number Definitions
// #include <stdlib.h>  // Standard Library Definitions
// #include <sys/wait.h> // for wait()

// // device port série à utiliser 
// //const char *portTTY = "/dev/ttyGS0";
// //const char *portTTY = "/dev/ttyS0";
// const char *portTTY = "/dev/ttyS1";
// //const char *portTTY = "/dev/ttyS2";
// //const char *portTTY = "/dev/ttyS3";
// //const char *portTTY = "/dev/ttyS4";
// //const char *portTTY = "/dev/ttyS5";
// //const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter
// int fd; // File Descriptor

// void vInitialisation();
// void codeDuProcessusParent();
// void codeDuProcessusEnfant();

// void main(void)
// {
//      vInitialisation();
     
//      pid_t pid = fork(); // Création du processus fils
//     if (pid < 0) {
//         // Erreur lors de la création du processus
//         perror("Erreur lors de fork");
//         exit(EXIT_FAILURE);
//     } else if (pid == 0) {
//         // Processus fils : écriture sur le port série
//         printf("Je suis le processus Fils, j'écris sur le port série ce que j'entends sur la console (terminal)...\n");
//         codeDuProcessusEnfant();
//         printf("Fin du Fils\n");
//     } else {
//         // Processus père : lecture depuis le port série
//         printf("Je suis le processus Père, j'écris sur la console (terminal) ce que j'entends sur le port série...\n");
//         codeDuProcessusParent();
//         printf("Fin du Père\n");
//         wait(NULL); // Attente que le fils se termine
//     }

//     return 0;
// }

// void vInitialisation ()
// {
    
//     const char* processusPereOuFils;
	
// 	printf("\n Lecture Port Serie");

// 	// Opening the Serial Port 
// 	fd = open(portTTY, O_RDWR | O_NOCTTY);  
// 							// O_RDWR   - Read/Write access to serial port 
// 							// O_NOCTTY - No terminal will control the process
// 							// Open in blocking mode,read will wait 
// 	if(fd == -1) // Error Checking
//     {
// 		printf("\n Erreur! ouverture de %s ", portTTY);
//         exit(EXIT_FAILURE);
//     }
// 	else
//     {
// 		printf("\n Ouverture de %s reussit ", portTTY);
//     }

// 	// Setting the Attributes of the serial port using termios structure 
// 	struct termios SerialPortSettings;	// Create the structure 
// 	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 
// 	// Setting the Baud rate
// 	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
// 	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
// 	// 8N1 Mode 
// 	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
// 	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
// 	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
// 	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
// 	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
// 	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
	
// 	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p

// 	SerialPortSettings.c_lflag &= (ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  

// 	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

// 	// Setting Time outs 
// 	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
// 	SerialPortSettings.c_cc[VTIME] = 0; // Wait 3sec (0 for indefinetly) 

// 	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
// 		printf("\n  Erreur! configuration des attributs du port serie"); 

//     tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
// }



// void codeDuProcessusParent()
// {
//     char read_buffer[32]; // Tampon pour stocker les données reçues
//     int nbytes;
    
//     while (1) {
//         // Lecture des données du port série
//         nbytes = read(fd, read_buffer, sizeof(read_buffer));
//         if (nbytes < 0) {
//             perror("Erreur de lecture");
//             exit(EXIT_FAILURE);
//         }

//         read_buffer[nbytes] = '\0'; // Ajouter le caractère de fin de chaîne
//         printf("Processus Père: nombre d'octets reçus : %d --> %s\n", nbytes, read_buffer);

//         // Si le caractère '!' est reçu, quitter
//         if (read_buffer[0] == '!') {
//             break;
//         }
//     }
//     close(fd); // Fermer le port série
// }

// void codeDuProcessusEnfant()
// {
//     char c;
//     while (1) {
//         // Lecture d'un caractère depuis le terminal
//         c = getchar();

//         // Écriture sur le port série
//         if (write(fd, &c, 1) < 0) {
//             perror("Erreur d'écriture");
//             exit(EXIT_FAILURE);
//         }

//         // Si le caractère 'q' est entré, quitter
//         if (c == 'q') {
//             break;
//         }
//     }
//     close(fd); // Fermer le port série
// }

#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <stdlib.h>  // Standard Library Definitions
#include <sys/wait.h> // for wait()

const char *portTTY = "/dev/ttyS1"; // Port série utilisé
int fd;  // File Descriptor

void vInitialisation();
void codeDuProcessusParent();
void codeDuProcessusEnfant();

int main(void)
{
    vInitialisation(); // Initialisation du port série

    pid_t pid = fork(); // Création du processus fils
    if (pid < 0) {
        // Erreur lors de la création du processus
        perror("Erreur lors de fork");
        exit(EXIT_FAILURE);
    } else if (pid == 0) {
        // Processus fils : écriture sur le port série
        printf("Je suis le processus Fils, j'écris sur le port série ce que j'entends sur la console (terminal)...\n");
        codeDuProcessusEnfant();
        printf("Fin du Fils\n");
    } else {
        // Processus père : lecture depuis le port série
        printf("Je suis le processus Père, j'écris sur la console (terminal) ce que j'entends sur le port série...\n");
        codeDuProcessusParent();
        printf("Fin du Père\n");
        wait(NULL); // Attente que le fils se termine
    }

    return 0;
}

void vInitialisation()
{
    // Ouverture du port série
    fd = open(portTTY, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Erreur lors de l'ouverture du port série");
        exit(EXIT_FAILURE);
    }

    // Configuration du port série
    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings); // Récupérer les attributs actuels du port

    // Configurer la vitesse en bauds
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    // Configurer 8N1 (8 bits de données, pas de parité, 1 bit d'arrêt)
    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;

    // Désactiver le contrôle de flux matériel
    SerialPortSettings.c_cflag &= ~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Activer la lecture et ignorer les lignes de contrôle du modem

    // Désactiver le contrôle de flux logiciel
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Mode non-canonique, sans écho ni signaux
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Pas de traitement de sortie
    SerialPortSettings.c_oflag &= ~OPOST;

    // Configurer VMIN et VTIME pour une lecture d'au moins 1 caractère avec un délai infini
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;

    // Appliquer les paramètres
    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur lors de la configuration des attributs du port série");
        exit(EXIT_FAILURE);
    }

    tcflush(fd, TCIFLUSH); // Vider le tampon d'entrée du port série
}

void codeDuProcessusParent()
{
    char read_buffer[32]; // Tampon pour stocker les données reçues
    int nbytes;
    
    while (1) {
        // Lecture des données du port série
        nbytes = read(fd, read_buffer, sizeof(read_buffer));
        if (nbytes < 0) {
            perror("Erreur de lecture");
            exit(EXIT_FAILURE);
        }

        read_buffer[nbytes] = '\0'; // Ajouter le caractère de fin de chaîne
        printf("Processus Père: nombre d'octets reçus : %d --> %s\n", nbytes, read_buffer);

        // Si le caractère '!' est reçu, quitter
        if (read_buffer[0] == '!') {
            break;
        }
    }
    close(fd); // Fermer le port série
}

void codeDuProcessusEnfant()
{
    char c;
    while (1) {
        // Lecture d'un caractère depuis le terminal
        c = getchar();

        // Écriture sur le port série
        if (write(fd, &c, 1) < 0) {
            perror("Erreur d'écriture");
            exit(EXIT_FAILURE);
        }

        // Si le caractère 'q' est entré, quitter
        if (c == 'q') {
            break;
        }
    }
    close(fd); // Fermer le port série
}
