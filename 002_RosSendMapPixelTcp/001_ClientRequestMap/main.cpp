#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#define PORT 2002
   
int main(int argc, char const *argv[])
{
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char *hello = "Hello from client";
    char buffer[32000] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }
   
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
       
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    char sendDat[5] = {1,2,3,4,5};
    while(true){
        send(sock , "{Request}" , 9 , 0 );
        
        unsigned long lengthReceived = 0;
        while(true){
            valread = read(sock, buffer, 32000);
            buffer[valread] = 0;
            //printf("%s\n",buffer);
            printf("%d\n",valread);

            lengthReceived += valread;

            if(buffer[valread - 1] == '}'){
                break;
            }
        }
        printf("Received data: %d\n", lengthReceived);
        usleep(2000000);
    }
    
    return 0;
}