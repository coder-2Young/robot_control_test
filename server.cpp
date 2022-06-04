#include "./include/rb_tcp.h"

#include <stdio.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <iostream>

using namespace rb_server;

class RbServer
{
private:
    HookFunction action;            // 接受到字符串后的回调函数
    int create_tcp_server_socket(); // 建立tcp连接并初始化
    int check_start(char[]);        // 检查是否有开始字符
    int check_end(char[]);          // 检查是否有结束字符
    int port;                       // 端口
    char split;                     //开始字符
    map<int, string> received_data; // 收到的字符串

public:
    RbServer(int port, char split, HookFunction action);
    ~RbServer();
    bool hook(HookFunction); // 注册回调函数
    void start();            // 开始监听并接受数据
};

RbServer::RbServer(int port, char split, HookFunction action)
{
    this->port = port;
    // this->create_tcp_server_socket();
    this->split = split;
    // this->action = action;
    this->hook(action);
    this->start();
}

RbServer::~RbServer()
{
}

void RbServer::start()
{
    fd_set read_fd_set;
    struct sockaddr_in new_addr;
    int server_fd, new_fd, ret_val, i;
    socklen_t addrlen;
    char buf[DATA_BUFFER];
    int all_connections[MAX_CONNECTIONS];

    /* Get the socket server fd */
    server_fd = create_tcp_server_socket();
    if (server_fd == -1)
    {
        fprintf(stderr, "Failed to create a server\n");
        return;
    }

    /* Initialize all_connections and set the first entry to server fd */
    for (i = 0; i < MAX_CONNECTIONS; i++)
    {
        all_connections[i] = -1;
    }
    all_connections[0] = server_fd;

    while (1)
    {
        FD_ZERO(&read_fd_set);
        /* Set the fd_set before passing it to the select call */
        for (i = 0; i < MAX_CONNECTIONS; i++)
        {
            if (all_connections[i] >= 0)
            {
                FD_SET(all_connections[i], &read_fd_set);
            }
        }

        /* Invoke select() and then wait! */
        // printf("\nUsing select() to listen for incoming events\n");
        ret_val = select(FD_SETSIZE, &read_fd_set, NULL, NULL, NULL);

        /* select() woke up. Identify the fd that has events */
        if (ret_val >= 0)
        {
            // printf("Select returned with %d\n", ret_val);
            /* Check if the fd with event is the server fd */
            if (FD_ISSET(server_fd, &read_fd_set))
            {
                /* accept the new connection */
                // printf("Returned fd is %d (server's fd)\n", server_fd);
                new_fd = accept(server_fd, (struct sockaddr *)&new_addr, &addrlen);
                if (new_fd >= 0)
                {
                    printf("Accepted a new connection with fd: %d\n", new_fd);
                    for (i = 0; i < MAX_CONNECTIONS; i++)
                    {
                        if (all_connections[i] < 0)
                        {
                            all_connections[i] = new_fd;
                            break;
                        }
                    }
                }
                else
                {
                    fprintf(stderr, "accept failed [%s]\n", strerror(errno));
                }
                ret_val--;
                if (!ret_val)
                    continue;
            }

            /* Check if the fd with event is a non-server fd */
            for (i = 1; i < MAX_CONNECTIONS; i++)
            {
                if ((all_connections[i] > 0) &&
                    (FD_ISSET(all_connections[i], &read_fd_set)))
                {
                    /* read incoming data */
                    // printf("Returned fd is %d [index, i: %d]\n", all_connections[i], i);
                    // buf[0] = '\0';
                    memset(buf, 0, DATA_BUFFER);
                    ret_val = recv(all_connections[i], buf, DATA_BUFFER, 0);
                    if (ret_val == 0)
                    {
                        printf("Closing connection for fd:%d\n", all_connections[i]);
                        close(all_connections[i]);
                        this->received_data[all_connections[i]] = "";
                        all_connections[i] = -1; /* Connection is now closed */
                    }
                    if (ret_val > 0)
                    {
                        bool flag = false;
                        // 收到信息
                        string buf_str = string(buf);
                        int split_position = buf_str.find(this->split);

                        // 找到所有分割符号分割出来的子串
                        while ((split_position = buf_str.find(this->split)) != string::npos)
                        {
                            flag = true;
                            // printf("Received data (len %d bytes, fd: %d): %s\n", ret_val, all_connections[i], buf);
                            this->received_data[all_connections[i]] += buf_str.substr(0, split_position);
                            // cout << this->received_data[all_connections[i]] << endl;
                            // cout << this->action(this->received_data[all_connections[i]]) << "+++" << endl;
                            string msg_str = this->action(this->received_data[all_connections[i]]) + "|";
                            // this->action(this->received_data[all_connections[i]]);
                            send(all_connections[i], msg_str.c_str(), strlen(msg_str.c_str()), 0);
                            buf_str = buf_str.substr(split_position + 1);
                            this->received_data[all_connections[i]] = "";
                        }
                        if (not flag)
                        {
                            // 存储剩余的字符串
                            this->received_data[all_connections[i]] += buf_str;
                        }

                        // 如果收到分割符
                    }
                    if (ret_val == -1)
                    {
                        printf("recv() failed for fd: %d [%s]\n",
                               all_connections[i], strerror(errno));
                        break;
                    }
                }
                ret_val--;
                if (!ret_val)
                    continue;
            } /* for-loop */
        }     /* (ret_val >= 0) */
    }         /* while(1) */

    /* Last step: Close all the sockets */
    for (i = 0; i < MAX_CONNECTIONS; i++)
    {
        if (all_connections[i] > 0)
        {
            close(all_connections[i]);
        }
    }
    return;
}

bool RbServer::hook(HookFunction func)
{
    this->action = func;
    return true;
}

int RbServer::create_tcp_server_socket()
{
    struct sockaddr_in saddr;
    int fd, ret_val;

    /* Step1: create a TCP socket */
    fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd == -1)
    {
        fprintf(stderr, "socket failed [%s]\n", strerror(errno));
        return -1;
    }
    printf("Created a socket with fd: %d\n", fd);

    /* Initialize the socket address structure */
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(this->port);
    saddr.sin_addr.s_addr = INADDR_ANY;

    /* Step2: bind the socket to port 7000 on the local host */
    ret_val = bind(fd, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    if (ret_val != 0)
    {
        fprintf(stderr, "bind failed [%s]\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Step3: listen for incoming connections */
    ret_val = listen(fd, 5);
    if (ret_val != 0)
    {
        fprintf(stderr, "listen failed [%s]\n", strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}