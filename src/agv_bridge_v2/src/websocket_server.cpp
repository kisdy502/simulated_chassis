#include "agv_bridge_v2/websocket_server.hpp"
#include <libwebsockets.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <signal.h>

// // 初始化静态成员
// WebSocketServer *WebSocketServer::instance_ = nullptr;

// // 静态回调函数 - 修正函数签名
// int WebSocketServer::websocket_callback(struct lws *wsi,
//                                         enum lws_callback_reasons  reason,
//                                         void *user, void *in, size_t len)
// {
//     (void)user;
//     if (!instance_)
//     {
//         return 0;
//     }

//     switch (reason)
//     {
//     case LWS_CALLBACK_ESTABLISHED:
//     {
//         std::lock_guard<std::mutex> lock(instance_->mutex_);
//         instance_->clients_.push_back(wsi);
//         std::cout << "WebSocket: 客户端已连接，当前连接数: "
//                   << instance_->clients_.size() << std::endl;
//         break;
//     }

//     case LWS_CALLBACK_RECEIVE:
//     {
//         // 接收客户端消息
//         std::string message((char *)in, len);

//         // 调用回调函数处理消息
//         if (instance_->messageCallback_)
//         {
//             instance_->messageCallback_(message);
//         }
//         break;
//     }

//     case LWS_CALLBACK_CLOSED:
//     case LWS_CALLBACK_HTTP_DROP_PROTOCOL:
//     {
//         std::lock_guard<std::mutex> lock(instance_->mutex_);

//         // 从客户端列表中移除
//         auto &clients = instance_->clients_;
//         clients.erase(std::remove(clients.begin(), clients.end(), wsi), clients.end());

//         std::cout << "WebSocket: 客户端已断开，当前连接数: "
//                   << clients.size() << std::endl;
//         break;
//     }

//     case LWS_CALLBACK_SERVER_WRITEABLE:
//     {
//         // 当套接字可写时调用
//         break;
//     }

//     default:
//         break;
//     }

//     return 0;
// }

// // 构造函数
// WebSocketServer::WebSocketServer(int port, MessageCallback callback)
//     : port_(port),
//       messageCallback_(callback),
//       context_(nullptr),
//       running_(false)
// {
//     instance_ = this;
// }

// // 析构函数
// WebSocketServer::~WebSocketServer()
// {
//     stop();
//     instance_ = nullptr;
// }

// // 启动服务器
// void WebSocketServer::start()
// {
//     if (running_)
//     {
//         return;
//     }

//     serverThread_ = std::thread([this]()
//                                 {
//         struct lws_context_creation_info info;
//         struct lws_protocols protocols[2];

//         // 清空结构体
//         memset(&info, 0, sizeof(info));
//         memset(protocols, 0, sizeof(protocols));

//         // 设置协议
//         protocols[0].name = "agv-protocol";
//         protocols[0].callback = websocket_callback;
//         protocols[0].per_session_data_size = 0;
//         protocols[0].rx_buffer_size = 1024 * 1024; // 1MB缓冲区

//         // 空协议结尾
//         protocols[1].name = nullptr;
//         protocols[1].callback = nullptr;
//         protocols[1].per_session_data_size = 0;

//         // 配置上下文信息
//         info.port = port_;
//         info.iface = nullptr;        // 监听所有接口
//         info.protocols = protocols;
//         info.gid = -1;
//         info.uid = -1;
//         info.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;

//         // 创建上下文
//         context_ = lws_create_context(&info);
//         if (!context_)
//         {
//             std::cerr << "WebSocket: 创建上下文失败" << std::endl;
//             return;
//         }

//         std::cout << "WebSocket服务器已启动，监听端口 " << port_ << std::endl;

//         running_ = true;

//         // 事件循环
//         while (running_ && context_)
//         {
//             lws_service(context_, 50); // 50ms超时
//         }

//         std::cout << "WebSocket服务器已停止" << std::endl; });
// }

// // 停止服务器
// void WebSocketServer::stop()
// {
//     if (!running_)
//     {
//         return;
//     }

//     running_ = false;

//     // 等待线程结束
//     if (serverThread_.joinable())
//     {
//         serverThread_.join();
//     }

//     // 销毁上下文
//     if (context_)
//     {
//         lws_context_destroy(context_);
//         context_ = nullptr;
//     }

//     // 清空客户端列表
//     {
//         std::lock_guard<std::mutex> lock(mutex_);
//         clients_.clear();
//     }
// }

// // 向所有客户端发送消息
// void WebSocketServer::sendToAll(const std::string &message)
// {
//     std::lock_guard<std::mutex> lock(mutex_);

//     if (clients_.empty())
//     {
//         return;
//     }

//     // 为每个客户端发送消息
//     for (struct lws *client : clients_)
//     {
//         if (client)
//         {
//             // 分配缓冲区（消息内容 + LWS_PRE）
//             size_t buffer_size = message.length() + LWS_PRE;
//             unsigned char *buffer = new unsigned char[buffer_size];

//             // 复制消息内容（跳过LWS_PRE字节）
//             memcpy(buffer + LWS_PRE, message.c_str(), message.length());

//             // 发送消息
//             lws_write(client, buffer + LWS_PRE, message.length(), LWS_WRITE_TEXT);

//             // 释放缓冲区
//             delete[] buffer;
//         }
//     }
// }

// // 获取客户端连接数
// size_t WebSocketServer::getClientCount() const
// {
//     std::lock_guard<std::mutex> lock(mutex_);
//     return clients_.size();
// }

// // 检查服务器是否正在运行
// bool WebSocketServer::isRunning() const
// {
//     return running_;
// }

/******* */

// 初始化静态成员
WebSocketServer *WebSocketServer::instance_ = nullptr;

int WebSocketServer::websocket_callback(struct lws *wsi,
                                        enum lws_callback_reasons reason,
                                        void *user, void *in, size_t len)
{
    (void)user;
    if (!instance_)
        return 0;

    switch (reason)
    {
    case LWS_CALLBACK_ESTABLISHED:
    {
        std::lock_guard<std::mutex> lock(instance_->clients_mutex_);
        instance_->clients_.push_back(wsi);
        std::cout << "WebSocket: 客户端已连接，当前连接数: "
                  << instance_->clients_.size() << std::endl;
        break;
    }

    case LWS_CALLBACK_RECEIVE:
    {
        std::string message((char *)in, len);
        if (instance_->messageCallback_)
        {
            instance_->messageCallback_(message);
        }
        break;
    }

    case LWS_CALLBACK_CLOSED:
    case LWS_CALLBACK_HTTP_DROP_PROTOCOL:
    {
        instance_->cleanupClient(wsi);
        break;
    }

    case LWS_CALLBACK_SERVER_WRITEABLE:
    {
        instance_->handleWritable(wsi);
        break;
    }

    default:
        break;
    }

    return 0;
}

void WebSocketServer::handleWritable(struct lws *wsi)
{
    // 获取待发送的消息
    ActiveWrite *active = nullptr;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        // 检查是否有正在发送的消息
        auto it = active_writes_.find(wsi);
        if (it != active_writes_.end())
        {
            active = &it->second;
        }
        else if (!message_queue_.empty())
        {
            // 从队列中取出新消息
            PendingMessage msg = message_queue_.front();

            // 如果是广播，为所有客户端创建消息
            if (msg.target == nullptr)
            {
                std::lock_guard<std::mutex> clients_lock(clients_mutex_);
                for (auto *client : clients_)
                {
                    active_writes_[client] = {msg.content, 0, client};
                }
                message_queue_.pop();
            }
            // 如果是单播，只为目标客户端创建
            else if (msg.target == wsi)
            {
                active_writes_[wsi] = {msg.content, 0, wsi};
                message_queue_.pop();
            }

            active = &active_writes_[wsi];
        }
    }

    if (!active)
    {
        return;
    }

    // 计算本次发送的大小（最大4KB）
    size_t remaining = active->content.length() - active->offset;
    size_t chunk_size = std::min(remaining, size_t(4096));

    // 分配缓冲区
    unsigned char *buffer = new unsigned char[LWS_PRE + chunk_size];
    memcpy(buffer + LWS_PRE, active->content.c_str() + active->offset, chunk_size);

    // 设置标志
    lws_write_protocol flags = LWS_WRITE_TEXT;

    if (active->offset > 0)
    {
        flags = static_cast<lws_write_protocol>(flags | LWS_WRITE_NO_FIN);
    }

    // 发送数据
    int written = lws_write(wsi, buffer + LWS_PRE, chunk_size, flags);
    delete[] buffer;

    if (written < 0)
    {
        // 发送失败，移除消息
        std::lock_guard<std::mutex> lock(queue_mutex_);
        active_writes_.erase(wsi);
        return;
    }

    active->offset += written;

    if (active->offset >= active->content.length())
    {
        // 发送完成
        std::lock_guard<std::mutex> lock(queue_mutex_);
        active_writes_.erase(wsi);
    }
    else
    {
        // 继续发送剩余部分
        lws_callback_on_writable(wsi);
    }
}

void WebSocketServer::cleanupClient(struct lws *wsi)
{
    std::lock_guard<std::mutex> clients_lock(clients_mutex_);
    auto &clients = clients_;
    clients.erase(std::remove(clients.begin(), clients.end(), wsi), clients.end());

    std::cout << "WebSocket: 客户端已断开，当前连接数: "
              << clients.size() << std::endl;

    // 移除该客户端的待发送消息
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    active_writes_.erase(wsi);
}

WebSocketServer::WebSocketServer(int port, MessageCallback callback)
    : port_(port),
      messageCallback_(callback),
      context_(nullptr),
      running_(false)
{
    instance_ = this;
}

WebSocketServer::~WebSocketServer()
{
    stop();
    instance_ = nullptr;
}

void WebSocketServer::start()
{
    if (running_)
        return;

    serverThread_ = std::thread([this]()
                                {
        struct lws_context_creation_info info;
        struct lws_protocols protocols[2];
        
        memset(&info, 0, sizeof(info));
        memset(protocols, 0, sizeof(protocols));
        
        protocols[0].name = "agv-protocol";
        protocols[0].callback = websocket_callback;
        protocols[0].per_session_data_size = 0;
        protocols[0].rx_buffer_size = 1024 * 1024;
        protocols[0].tx_packet_size = 4096;
        
        protocols[1].name = nullptr;
        protocols[1].callback = nullptr;
        
        info.port = port_;
        info.iface = nullptr;
        info.protocols = protocols;
        info.gid = -1;
        info.uid = -1;
        info.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
        
        context_ = lws_create_context(&info);
        if (!context_)
        {
            std::cerr << "WebSocket: 创建上下文失败" << std::endl;
            return;
        }
        
        std::cout << "WebSocket服务器已启动，监听端口 " << port_ << std::endl;
        
        running_ = true;
        
        while (running_ && context_)
        {
            lws_service(context_, 50);
            
            // 定期检查是否有待发送的消息
            std::lock_guard<std::mutex> queue_lock(queue_mutex_);
            if (!message_queue_.empty())
            {
                std::lock_guard<std::mutex> clients_lock(clients_mutex_);
                for (auto *client : clients_)
                {
                    lws_callback_on_writable(client);
                }
            }
        }
        
        std::cout << "WebSocket服务器已停止" << std::endl; });
}

void WebSocketServer::stop()
{
    if (!running_)
        return;

    running_ = false;

    if (serverThread_.joinable())
    {
        serverThread_.join();
    }

    if (context_)
    {
        lws_context_destroy(context_);
        context_ = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.clear();
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (!message_queue_.empty())
        {
            message_queue_.pop();
        }
        active_writes_.clear();
    }
}

void WebSocketServer::sendToAll(const std::string &message)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    PendingMessage msg;
    msg.content = message;
    msg.target = nullptr; // nullptr 表示广播
    message_queue_.push(msg);
}

void WebSocketServer::sendToClient(struct lws *client, const std::string &message)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    PendingMessage msg;
    msg.content = message;
    msg.target = client;
    message_queue_.push(msg);

    lws_callback_on_writable(client);
}

size_t WebSocketServer::getClientCount() const
{
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return clients_.size();
}

bool WebSocketServer::isRunning() const
{
    return running_;
}