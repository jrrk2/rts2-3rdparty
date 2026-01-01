/*
 * Simple WebSocket Client for RTS2
 * Qt-free implementation using standard POSIX sockets
 */

#ifndef __RTS2_ORIGIN_WEBSOCKET_H__
#define __RTS2_ORIGIN_WEBSOCKET_H__

#include <string>
#include <functional>

class OriginWebSocket
{
public:
    OriginWebSocket();
    ~OriginWebSocket();
    
    bool connect(const std::string& host, int port, const std::string& path = "/cgi-bin/AlpacaWebSocket");
    void disconnect();
    bool isConnected() const { return m_connected; }
    
    bool sendText(const std::string& message);
    std::string receiveText();
    bool hasData();
    
    // Callback for received messages
    std::function<void(const std::string&)> onTextMessage;
    
private:
    int m_socket;
    bool m_connected;
    
    bool doHandshake(const std::string& host, const std::string& path);
};

#endif // __RTS2_ORIGIN_WEBSOCKET_H__
