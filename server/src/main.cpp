#include "../include/crc32.h"
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <mutex>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#define HOST "0.0.0.0"
#define PORT 80
#define BUFF_SIZE 128

namespace fs = std::filesystem;
using namespace std;

mutex mtx;
vector<int> clientSockets;
condition_variable cv;

struct data_s {
    const char *delimeter;
    char *mac;
    char *name;
    time_t time;
    int SYS;
    int DIA;
    int PUL;
    float TMP;
    unsigned long crc;
    const size_t size;
};

void protectedFree(void *ptr) {
    if (ptr) {
        free(ptr);
        ptr = NULL;
    }
}

int exitAndFree(const char *message, int count, ...) {
    va_list args;
    perror(message);
    va_start(args, count);
    for (int i = 0; i < count; ++i) {
        void *ptr = va_arg(args, void *);
        protectedFree(ptr);
    }
    va_end(args);
    return count + 1;
}

char *getLocalIp() {
    char *localIp = (char *)calloc(INET_ADDRSTRLEN, sizeof(char));
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

int saveToJSON(struct data_s *data) {
    const char *mainFolderPath = "../records";
    char *deviceFolderPath;
    char *userFilePath;
    char *workMode;
    char *fileContent;
    char *updatedFileContent;
    asprintf(&deviceFolderPath, "%s/%s", mainFolderPath, data->mac);
    if (!deviceFolderPath)
        return exitAndFree("Memory allocation error", 0);
    if (!data->name)
        data->name = "Unnamed";
    asprintf(&userFilePath, "%s/%s.json", deviceFolderPath, data->name);
    if (!userFilePath)
        return exitAndFree("Memory allocation error", 1, deviceFolderPath);
    if (!fs::exists(deviceFolderPath))
        fs::create_directories(deviceFolderPath);
    if (!fs::exists(userFilePath))
        workMode = "wb+";
    else
        workMode = "rb";

    FILE *userFile = fopen(userFilePath, workMode);
    if (!userFile)
        return exitAndFree("Can not open file", 2, deviceFolderPath,
                           userFilePath);
    struct tm *timeInfo = localtime(&data->time);
    char timeString[21];
    if (!strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%SZ",
                  timeInfo)) {
        return exitAndFree("Parsing time stamp failed", 2, deviceFolderPath,
                           userFilePath);
    }

    fseek(userFile, 0, SEEK_END);
    long fileSize = ftell(userFile);
    fseek(userFile, 0, SEEK_SET);

    if (fileSize < 2)
        asprintf(&fileContent, "[");
    else
        fileContent = (char *)calloc(fileSize + 1U, sizeof(char));
    if (!fileContent)
        return exitAndFree("Memory allocation error", 2, deviceFolderPath,
                           userFilePath);

    if (fileSize > 2) {
        fread(fileContent, 1U, fileSize, userFile);
        fileContent[strlen(fileContent) - 2U] = ',';
        fileContent[strlen(fileContent) - 1U] = '\0';
    }

    if (fclose(userFile))
        return exitAndFree("Can not close file", 3, deviceFolderPath,
                           userFilePath, fileContent);

    asprintf(&updatedFileContent,
             "%s\n"
             "\t{\n"
             "\t\t\"Time\": \"%s\",\n"
             "\t\t\"Systole\": %d,\n"
             "\t\t\"Diastole\": %d,\n"
             "\t\t\"Pulse\": %d,\n"
             "\t\t\"Temperature\": %3.1f\n"
             "\t}\n"
             "]",
             fileContent, timeString, data->SYS, data->DIA, data->PUL,
             data->TMP);
    if (!updatedFileContent)
        return exitAndFree("Memory allocation error", 3, deviceFolderPath,
                           userFilePath, fileContent);

    userFile = fopen(userFilePath, "w");
    if (!userFile)
        return exitAndFree("Error occured while opening file "
                           "for write",
                           3, userFilePath, fileContent, updatedFileContent);
    fputs(updatedFileContent, userFile);
    if (fclose(userFile))
        return exitAndFree("Can not close file", 4, deviceFolderPath,
                           userFilePath, fileContent, updatedFileContent);

    protectedFree(updatedFileContent);
    protectedFree(fileContent);
    protectedFree(userFilePath);
    protectedFree(deviceFolderPath);

    return false;
}

void handleClient(int clientSocket) {
    char package[BUFF_SIZE];
    long received = 0;
    bool error = false;

    srand(clock());

    this_thread::sleep_for(chrono::milliseconds(1000));

    if (!(fcntl(clientSocket, F_GETFL) & O_NONBLOCK)) {
        if (fcntl(clientSocket, F_SETFL,
                  fcntl(clientSocket, F_GETFL) | O_NONBLOCK) < 0) {
            perror("Can not put socket into non-blocking mode...");
            error = true;
        }
    }
    while (!error) {
        const uint32_t CRC = 0xFFFFFFFF;
        uint32_t randValue = static_cast<uint32_t>(rand());
        unsigned sendBufferLength = sizeof(uint32_t) * 2U + 1U;
        char *sendBuffer = (char *)calloc(sendBufferLength, sizeof(char));
        char *receiveBuffer = (char *)calloc(sendBufferLength, sizeof(char));
        snprintf(sendBuffer, sendBufferLength, "%0*X",
                 static_cast<int>(sizeof(uint32_t) * 2U), randValue);
        send(clientSocket, sendBuffer, strlen(sendBuffer) + 1U, 0);

        this_thread::sleep_for(chrono::milliseconds(1500));

        received = recv(clientSocket, receiveBuffer, sendBufferLength,
                        MSG_PEEK | MSG_TRUNC | MSG_DONTWAIT);
        if (received < 0) {
            perror("Error receiving package from client");
            error = true;
        }
        protectedFree(receiveBuffer);

        if (!error) {
            receiveBuffer = (char *)calloc(received + 1U, sizeof(char));
            received = read(clientSocket, receiveBuffer, received + 1U);

            if (crc32(CRC, sendBuffer, strlen(sendBuffer)) !=
                strtoul(receiveBuffer, NULL, 16)) {
                perror("Did not receive the same message from the client. "
                       "Disconnecting");
                error = true;
            }

            protectedFree(receiveBuffer);
        }
        protectedFree(sendBuffer);

        if (!error) {
            this_thread::sleep_for(chrono::milliseconds(1000));

            received =
                recv(clientSocket, package, sizeof(package), MSG_DONTWAIT);

            if (received > 0) {
                struct data_s data = {
                    .delimeter = "#", .time = time(NULL), .size = 5U};
                char *pointer = package;
                vector<char *> substrings;

                puts("Received package");

                if (pointer = strrchr(pointer, *data.delimeter)) {
                    *pointer = '\0';
                    pointer++;
                    if (crc32(CRC, package, strlen(package)) !=
                        (data.crc = strtoul(pointer, NULL, 16))) {
                        perror("CRC don't match");
                        error = true;
                    }
                } else {
                    perror("Can't find delimeter");
                    error = true;
                }
                if (!error) {
                    pointer = strtok(package, data.delimeter);
                    while (pointer) {
                        substrings.push_back(pointer);
                        pointer = strtok(NULL, data.delimeter);
                    }
                    if (substrings.size() != data.size) {
                        perror("Package size doesn't equal to protocol "
                               "defined value");
                        error = true;
                    }
                }
                if (!error) {
                    data.mac = strdup(substrings[0]);
                    if (data.mac) {
                        if (char *devider = strchr(data.mac, '&')) {
                            *devider = '\0';
                            data.name = devider + 1U;
                        }
                        data.SYS = atoi(substrings[1]);
                        data.DIA = atoi(substrings[2]);
                        data.PUL = atoi(substrings[3]);
                        data.TMP = atof(substrings[4]);

                        error = saveToJSON(&data);

                        protectedFree(data.mac);
                    } else {
                        perror("Memory allocation error");
                        error = true;
                    }
                }
            }
        }
        if (error)
            break;
    }

    close(clientSocket);

    unique_lock<mutex> lock(mtx);
    clientSockets.erase(remove_if(
        clientSockets.begin(), clientSockets.end(),
        [clientSocket](int socket) { return socket == clientSocket; }));
    cv.notify_all();
}

int main() {
    char *localIp = getLocalIp();

    if (localIp != nullptr) {
        printf("IP-address of your PC: %s\n", localIp);
        protectedFree(localIp);
    } else {
        printf("Can not get your local IP-address\n");
    }

    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serverAddr {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    inet_pton(AF_INET, HOST, &(serverAddr.sin_addr));

    bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    listen(serverSocket, 1);

    printf("Server started. Waiting for client...\n");

    while (true) {
        struct sockaddr_in clientAddr {};
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket =
            accept(serverSocket, (struct sockaddr *)&clientAddr, &clientLen);

        {
            unique_lock<mutex> lock(mtx);
            clientSockets.push_back(clientSocket);
        }

        printf("Connected to client: %s\n", inet_ntoa(clientAddr.sin_addr));

        thread clientThread(handleClient, clientSocket);
        clientThread.detach();
    }

    close(serverSocket);

    return 0;
}