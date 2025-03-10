#include "../include/declarations.h"
namespace BKND {
namespace IRoC {
int Connect(bool p_ishost, std::string p_targetip) {
  {
    const int PORT = 12345;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      std::cerr << "Socket Creation Failed" << std::endl;
      return -1;
    }
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(PORT);

    int comm_sock = -1;
    if (p_ishost) {
      address.sin_addr.s_addr = INADDR_ANY;
      if (bind(sock, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Binding failed: " << strerror(errno) << std::endl;
        close(sock);
        return -1;
      }
      if (listen(sock, 1) < 0) {
        std::cerr << "Listen failed: " << strerror(errno) << std::endl;
        close(sock);
        return -2;
      }
      std::cout << "Awaiting Connection..." << std::endl;
      comm_sock = accept(sock, nullptr, nullptr);
      close(sock);
    } else if (!p_ishost) {
      address.sin_addr.s_addr = inet_addr(p_targetip.c_str());
      std::cout << "Attempting connection..." << std::endl;
      if (connect(sock, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Connection failed: " << strerror(errno) << std::endl;
        return -3;
      }
      comm_sock = sock;
    }
    if (comm_sock < 0) {
      std::cerr << "Failed to connect" << std::endl;
      return -4;
    }
    setsockopt(comm_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(comm_sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    setsockopt(comm_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(comm_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    std::cout << "Connected successfully" << std::endl;
    return comm_sock;
  }
}
void Send(int p_socket, std::string p_message) {
  if (!p_message.empty()) {
    send(p_socket, p_message.c_str(), p_message.length(), 0);
  }
}

} // namespace IRoC
} // namespace BKND
