#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "keyboard.h"
#define BUF_SIZE 4096
#define RLT_SIZE 4
#define OPSZ 4

using namespace std;

void print_manual()
{

    printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
    printf("$$                   How to use(Don't hit enter)                    $$\n");
    printf("$$------------------------------------------------------------------$$\n");
    printf("$$  ECAT UP = o/ ON = c/ Debug = d/-> Use when U want to move motor $$\n");
    printf("$$              GO = w/ RIGHT = d/ LEFT = a/ BACK = s               $$\n");
    printf("$$                           TRACKING = t                           $$\n");
    printf("$$                             QUIT = q                             $$\n");
    printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
}

int main(int argc, char *argv[])
{
    int sock;
    char key;
    char result[BUF_SIZE];
    struct sockaddr_in server_addr;

    if (argc != 3)
    {
        printf("Usage: %s <IP> <port>\n", argv[0]);
        exit(1);
    }

    sock = socket(AF_INET, SOCK_STREAM, 0);

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(argv[1]);
    server_addr.sin_port = htons(atoi(argv[2]));

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
        perror("connect() error!");
    else
        puts("Connected........");

    do
    {
        print_manual();
        printf("Enter the key: ");
        scanf("%c", &key);
        fgetc(stdin);
        printf("key = %c\n", key);

        if (key == (char)10)
            fgetc(stdin);

        *result = 0;
        write(sock, &key, sizeof(key));
        //printf("WRITE SUCCEED \n");
        read(sock, &result, RLT_SIZE);
        //printf("read succeed \n");

        printf("result: %c \n", (*result));
    }while(key!='q');

    close(sock);
    return 0;
}
