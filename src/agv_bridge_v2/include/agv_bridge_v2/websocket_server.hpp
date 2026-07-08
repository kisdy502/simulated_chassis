#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <libwebsockets.h>
#include <functional>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>  // 用于消息队列
#include <atomic> // 用于原子操作
#include <map>    // 用于管理正在发送的消息

// 前向声明，避免包含libwebsockets头文件
struct lws;
struct lws_context;

// 待发送的消息结构
struct PendingMessage
{
    std::string content;
    struct lws *target; // nullptr 表示广播给所有客户端
};

// 正在发送的消息（支持分片）
struct ActiveWrite
{
    std::string content;
    size_t offset;
    struct lws *target;
};

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

    // 向指定客户端发送消息
    void sendToClient(struct lws *client, const std::string &message);

private:
    // libwebsockets 回调函数（静态）
    static int websocket_callback(struct lws *wsi,
                                  enum lws_callback_reasons reason,
                                  void *user, void *in, size_t len);

    // 处理可写事件（在事件循环线程中安全发送消息）
    void handleWritable(struct lws *wsi);

    // 清理客户端连接
    void cleanupClient(struct lws *wsi);

    int port_;                          // 端口号
    MessageCallback messageCallback_;   // 消息回调函数
    struct lws_context *context_;       // libwebsockets 上下文
    std::thread serverThread_;          // 服务器线程
    std::atomic<bool> running_;         // 运行标志（原子操作）
    mutable std::mutex mutex_;          // 互斥锁
    std::vector<struct lws *> clients_; // 连接的客户端列表

    // 客户端列表及其互斥锁
    mutable std::mutex clients_mutex_;

    // 消息队列及其互斥锁
    std::queue<PendingMessage> message_queue_;
    std::mutex queue_mutex_;

    // 正在发送的消息（支持分片）
    std::map<struct lws *, ActiveWrite> active_writes_;

    // 全局实例指针（用于静态回调函数）
    static WebSocketServer *instance_;
};

#endif // WEBSOCKET_SERVER_H