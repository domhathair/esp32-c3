#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "../raygui/src/raygui.h"

#define HOST "0.0.0.0"
#define PORT 80
#define BUFF_SIZE 256

char* get_local_ip() {
    char* local_ip = (char*)malloc(INET_ADDRSTRLEN * sizeof(char));
    int temp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(80);
    inet_pton(AF_INET, "8.8.8.8", &(server.sin_addr));

    connect(temp_socket, (struct sockaddr*)&server, sizeof(server));
    struct sockaddr_in local;
    socklen_t len = sizeof(local);
    getsockname(temp_socket, (struct sockaddr*)&local, &len);
    close(temp_socket);

    inet_ntop(AF_INET, &(local.sin_addr), local_ip, INET_ADDRSTRLEN);
    return local_ip;
}

int main() {
    char* local_ip = get_local_ip();

    if (local_ip != NULL) {
        printf("IP-address of your PC: %s\n", local_ip);
        free(local_ip);
    } else {
        printf("Can not get your local IP-address.\n");
    }

    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    inet_pton(AF_INET, HOST, &(server.sin_addr));

    bind(server_socket, (struct sockaddr*)&server, sizeof(server));
    listen(server_socket, 1);

    printf("Server started. Waiting for client...\n");

    struct sockaddr_in client;
    socklen_t client_len = sizeof(client);
    int client_socket = accept(server_socket, (struct sockaddr*)&client, &client_len);
    printf("Connected to client: %s\n", inet_ntoa(client.sin_addr));

    while (1) {
        char data[BUFF_SIZE];
        ssize_t bytes_received = recv(client_socket, data, sizeof(data), 0);
        if (bytes_received <= 0) {
            break;
        }
        fprintf(stdout, "Data received: %.*s\n", (int)bytes_received, data);
        snprintf(data, BUFF_SIZE, "Hello, I'm a server!\n");
        send(client_socket, data, strlen(data) + 1, 0);
    }

    close(client_socket);
    close(server_socket);

    return 0;
}