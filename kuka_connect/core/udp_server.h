#ifndef H_UDP_SERVER
#define H_UDP_SERVER

#include <iostream>
#include <string>
#include <stdexcept>

#include <sys/time.h>

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <assert.h>

#define BUFSIZE 1024

class UDPServer {
public:
  UDPServer(std::string host, unsigned short port)
      : host(host), timeoutEnabled(false) {

    // Create a UDP socket
    sockFd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockFd < 0) {
      throw std::runtime_error("Error opening socket: " + std::string(strerror(errno)));
    }

    // Reuse address if necessary
    int optVal = 1;
    setsockopt(sockFd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optVal , sizeof(int));

    // Set socket connection address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(host.c_str());
    serverAddr.sin_port = htons(port);

    // Bind address
    if (bind(sockFd, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
      throw std::runtime_error("Error binding socket: " + std::string(strerror(errno)));
    }
  }

  ~UDPServer() {
    close(sockFd);
  }

  void set_timeout(int millisecs) {
    assert(millisecs >= 0);

    timeout.tv_sec  = millisecs / 1000;
    timeout.tv_usec = (millisecs % 1000) * 1000;
    timeoutEnabled = true;
  }

  ssize_t send(std::string& buffer) {
    ssize_t bytes = 0;
    socklen_t clientLen = sizeof(clientAddr);
    bytes = sendto(sockFd, buffer.c_str(), buffer.size(), 0, (struct sockaddr *) &clientAddr, clientLen);

    if (bytes < 0) {
      std::cout << "ERROR in sendto: " << strerror(errno) << std::endl;
      char *ip = inet_ntoa(clientAddr.sin_addr);
      std::cout << ip;
    }

    return bytes;
  }

  ssize_t recv(std::string& message) {
    ssize_t numBytesRead = 0;

    if (timeoutEnabled) {
      fd_set readFds;
      FD_ZERO(&readFds);
      FD_SET(sockFd, &readFds);

      struct timeval timeoutCopy;
      timeoutCopy.tv_sec = timeout.tv_sec;
      timeoutCopy.tv_usec = timeout.tv_usec;

      // Wait for data to be ready to be read from the socket or time out
      if (select(sockFd + 1, &readFds, NULL, NULL, &timeoutCopy) < 0) {
        return 0;
      }

      if (FD_ISSET(sockFd, &readFds)) {
        memset(buffer, 0, BUFSIZE);
        socklen_t clientLen = sizeof(clientAddr);

        numBytesRead = recvfrom(sockFd, buffer, BUFSIZE, 0, (struct sockaddr *) &clientAddr, &clientLen);

        if (numBytesRead < 0) {
          std::cout << "ERROR in recvfrom" << std::endl;
        }
      } else {
        return 0;
      }
    } else {
      // Read from the socket without a timeout
      memset(buffer, 0, BUFSIZE);
      socklen_t clientLen = sizeof(clientAddr);
      numBytesRead = recvfrom(sockFd, buffer, BUFSIZE, 0, (struct sockaddr *) &clientAddr, &clientLen);

      if (numBytesRead < 0) {
        std::cout << "ERROR in recvfrom" << std::endl;
      }
    }

    message = std::string(buffer);

    return numBytesRead;
  }

private:
  std::string host;
  bool timeoutEnabled;
  struct timeval timeout;

  int sockFd;
  struct sockaddr_in serverAddr;
  struct sockaddr_in clientAddr;

  char buffer[BUFSIZE];
};

#endif