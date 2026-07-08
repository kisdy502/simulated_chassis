#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <libwebsockets.h>
#include <functional>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>

// 前向声明，避免包含libwebsockets头文件
struct lws;
struct lws_context;

class WebSocketServer
{
public:
    using MessageCallback = std::function<void(const std::string &)>;

    WebSocketServer(int port, MessageCallback callback);
    ~WebSocketServer();

    // 启动服务器
    void start();

    // 停止服务器
    void stop();

    // 向所有连接的客户端发送消息
    void sendToAll(const std::string &message);

    // 获取客户端连接数
    size_t getClientCount() const;

    // 检查服务器是否正在运行
    bool isRunning() const;

private:
    // libwebsockets 回调函数（静态）
    static int websocket_callback(struct lws *wsi,
                                  enum lws_callback_reasons  reason,
                                  void *user, void *in, size_t len);

    int port_;                          // 端口号
    MessageCallback messageCallback_;   // 消息回调函数
    struct lws_context *context_;       // libwebsockets 上下文
    std::thread serverThread_;          // 服务器线程
    bool running_;                      // 运行标志
    mutable std::mutex mutex_;          // 互斥锁
    std::vector<struct lws *> clients_; // 连接的客户端列表

    // 全局实例指针（用于静态回调函数）
    static WebSocketServer *instance_;
};

#endif // WEBSOCKET_SERVER_H