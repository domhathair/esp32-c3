#include "../include/crc32.h"
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <mutex>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#define HOST "0.0.0.0"
#define PORT 80
#define BUFF_SIZE 256

std::mutex mtx;
std::vector<int> clientSockets;
std::condition_variable cv;

char *getLocalIp() {
    char *localIp = new char[INET_ADDRSTRLEN];
    int tempSocket = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server {};
    server.sin_family = AF_INET;
    server.sin_port = htons(80);
    inet_pton(AF_INET, "8.8.8.8", &(server.sin_addr));

    connect(tempSocket, (struct sockaddr *)&server, sizeof(server));
    struct sockaddr_in local {};
    socklen_t len = sizeof(local);
    getsockname(tempSocket, (struct sockaddr *)&local, &len);
    close(tempSocket);

    inet_ntop(AF_INET, &(local.sin_addr), localIp, INET_ADDRSTRLEN);
    return localIp;
}

void handleClient(int clientSocket) {
    char data[BUFF_SIZE];
    ssize_t received = 0U;
    bool skip = false;
    std::srand(std::clock());

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (!(fcntl(clientSocket, F_GETFL) & O_NONBLOCK)) {
        if (fcntl(clientSocket, F_SETFL,
                  fcntl(clientSocket, F_GETFL) | O_NONBLOCK) < 0) {
            std::printf("Can not put socket into non-blocking mode...\n");
            skip = true;
        }
    }
    while (!skip) {
        const uint32_t crc = 0xFFFFFFFF;
        uint32_t randValue = static_cast<uint32_t>(rand());
        unsigned sendBufferLength = sizeof(uint32_t) * 2U + 1U;
        char *sendBuffer = new char[sendBufferLength];
        char *receiveBuffer = new char[sendBufferLength];
        std::snprintf(sendBuffer, sendBufferLength, "%0*X",
                      static_cast<int>(sizeof(uint32_t) * 2U), randValue);
        send(clientSocket, sendBuffer, sendBufferLength, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        received = recv(clientSocket, receiveBuffer, sendBufferLength,
                        MSG_PEEK | MSG_TRUNC | MSG_DONTWAIT);
        if (received < 0U) {
            std::perror("Error receiving data from client");
            break;
        }

        delete[] receiveBuffer;
        receiveBuffer = new char[received + 1U];
        received = read(clientSocket, receiveBuffer, received + 1U);

        if (crc32(crc, sendBuffer, std::strlen(sendBuffer)) !=
            std::strtoul(receiveBuffer, NULL, 16)) {
            std::perror("Did not receive the same message from the client. "
                        "Disconnecting");
            break;
        }

        delete[] receiveBuffer;
        delete[] sendBuffer;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        received = recv(clientSocket, data, sizeof(data), MSG_DONTWAIT);

        // if (received > 0) {
        // std::printf("Data received: %.*s\n", static_cast<int>(received -
        // 6UL),
        //            data); /*Without CRC32 check-sum at the end*/
        const char *strExample =
            "00:11:22:33:44:55&Name#SYS:120#DIA:080#PUL:060#TMP:36.6#0000";
        const char delimeter = '#';
        char *basicStr = strdup(strExample);
        char *pointer[2] = {basicStr, NULL};
        while ((pointer[1] = std::strchr(pointer[0], delimeter)) != NULL) {
            *pointer[1] = '\0';
            char *newStr = strdup(pointer[0]);
            std::printf("%s\n", newStr);
            std::free(newStr);
            pointer[0] = pointer[1] + 1U;
        }
        std::free(basicStr);
        //}
    }

    close(clientSocket);

    std::unique_lock<std::mutex> lock(mtx);
    clientSockets.erase(std::remove_if(
        clientSockets.begin(), clientSockets.end(),
        [clientSocket](int socket) { return socket == clientSocket; }));
    cv.notify_all();
}

int main() {
    char *localIp = getLocalIp();

    if (localIp != nullptr) {
        std::printf("IP-address of your PC: %s\n", localIp);
        delete[] localIp;
    } else {
        std::printf("Can not get your local IP-address\n");
    }

    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serverAddr {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    inet_pton(AF_INET, HOST, &(serverAddr.sin_addr));

    bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    listen(serverSocket, 1);

    std::printf("Server started. Waiting for client...\n");

    while (true) {
        struct sockaddr_in clientAddr {};
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket =
            accept(serverSocket, (struct sockaddr *)&clientAddr, &clientLen);

        {
            std::unique_lock<std::mutex> lock(mtx);
            clientSockets.push_back(clientSocket);
        }

        std::printf("Connected to client: %s\n",
                    inet_ntoa(clientAddr.sin_addr));

        std::thread clientThread(handleClient, clientSocket);
        clientThread.detach();
    }

    close(serverSocket);

    return 0;
}