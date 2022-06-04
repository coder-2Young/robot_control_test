#ifndef RB_TCP_H
#define RB_TCP_H

#include <vector>
#include <map>

namespace rb_common
{
    using namespace std;

    namespace __common
    {
#define DATA_BUFFER 5000
#define MAX_CONNECTIONS 10
        // 定义tcp连接的回调函数类型
        typedef string (*HookFunction)(string);
        // 定义回调函数组
        typedef map<string, HookFunction> HookFunctionGroup;

    }
    // 用于存储tcp连接池
    typedef vector<int> ConnectionPool;
    // 定义用于连接池中连接的对应函数的回调函数组
    typedef map<int, __common::HookFunctionGroup> ConnectionGroupHookFunctionGroup;
    // 直接使用的回调函数
    typedef __common::HookFunction HookFunction;
}

namespace rb_server
{
    using namespace rb_common;
} // namespace rb_server

namespace rb_client
{
    using namespace rb_common;
} // namespace rb_client

#endif
