

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
void codeDuProcessusEnfant2();
void codeDuProcessusEnfant1();

int main(void)
{
    vInitialisation(); // Initialisation du port série

    pid_t pid = fork(); // Création du processus fils
    if (pid < 0) 
    {
        // Erreur lors de la création du processus
        perror("Erreur lors de fork");
        exit(EXIT_FAILURE);
    } 
    else if (pid == 0) 
    {
        // Processus fils : écriture sur le port série
        printf("Je suis le processus Fils, j'écris sur le port série ce que j'entends sur la console (terminal)...\n");
        codeDuProcessusEnfant1();
        printf("Fin du Fils\n");
    } else 
    {
        pid_t pid2 = fork(); // Création du processus fils
        if(pid2 == 0)
        {
         printf("Je suis le processus Fils2, j'écris sur la console (terminal) ce que j'entends sur le port série...\n");
         codeDuProcessusEnfant2();
        }
        else 
        {
          int n=1; 
          while(n<10) 
          { 
            n++; 
            sleep(3); 
          } 
            wait(NULL); // attendre la fin de enfant 1 
            wait(NULL); // attendre la fin de enfant 2 
            close(fd); // Fermer le port série 
            printf("Fin du processus Principal\n"); 
        }
        
        // Processus père : lecture depuis le port série
        printf("Je suis le processus Père\n");
       
        printf("Fin du Père\n");
        wait(NULL); // Attente que le fils se termine
    }

    return 0;
}

void vInitialisation()
{
    // Ouverture du port série
    fd = open(portTTY, O_RDWR | O_NOCTTY);
    if (fd == -1) 
    {
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
    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) 
    {
        perror("Erreur lors de la configuration des attributs du port série");
        exit(EXIT_FAILURE);
    }

    tcflush(fd, TCIFLUSH); // Vider le tampon d'entrée du port série
}

void codeDuProcessusEnfant2()
{
    char read_buffer[32]; // Tampon pour stocker les données reçues
    int nbytes;
    
    while (1) 
    {

        // Lecture des données du port série
        nbytes = read(fd, read_buffer, sizeof(read_buffer));
        if (nbytes < 0) 
        {
            perror("Erreur de lecture");
            exit(EXIT_FAILURE);
        }

        read_buffer[nbytes] = '\0'; // Ajouter le caractère de fin de chaîne
        printf("Processus Enfant2: nombre d'octets reçus : %d --> %s\n", nbytes, read_buffer);

        // Si le caractère '!' est reçu, quitter
        if (read_buffer[0] == '!') 
        {
            break;
        }
    }
    close(fd); // Fermer le port série
}

void codeDuProcessusEnfant1()
{
    char c;
    while (1) {
        // Lecture d'un caractère depuis le terminal
        c = getchar();

        // Écriture sur le port série
        if (write(fd, &c, 1) < 0) 
        {
            perror("Erreur d'écriture");
            exit(EXIT_FAILURE);
        }

        // Si le caractère 'q' est entré, quitter
        if (c == 'q') 
        {
            break;
        }
    }
    close(fd); // Fermer le port série
}