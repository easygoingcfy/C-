#include <iostream> 
#include <boost/asio.hpp> 

using namespace boost::asio;
#define BLOCK_SIZE 64*1024

int main(int argc, char* argv[])
{
    // 所有asio类都需要io_service对象 
    io_service iosev; 
    ip::tcp::acceptor acceptor(iosev,  
        ip::tcp::endpoint(ip::tcp::v4(), 1000)); 
    for(;;) 
    { 
        // socket对象 
        ip::tcp::socket socket(iosev); 
        // 等待直到客户端连接进来 
        acceptor.accept(socket); 
        // 显示连接进来的客户端 
        std::cout << "client from: " 
            << socket.remote_endpoint().address() << std::endl; 
        
        boost::system::error_code ec;

        // 从客户端读取数据
        char buf[BLOCK_SIZE];
        int len = socket.read_some(buffer(buf), ec);
        // 或者可以使用read_until读到某个字符为止
        // 或者可以使用某种判断方式循环读取

        if (ec)
        {
            std::cout <<  
                boost::system::system_error(ec).what() << std::endl; 
            break; 
        }
        std::cout.write(buf, len);
        std::cout << len << std::endl;

        //Sleep(1000);

        // 向客户端发送 
        len = socket.write_some(buffer(buf, len), ec); 
        if(ec) 
        { 
            std::cout <<  
                boost::system::system_error(ec).what() << std::endl; 
            break; 
        } 
        std::cout << "writed " << len << std::endl;
        // 与当前客户交互完成后循环继续等待下一客户连接 
    } 


    return 0; 

}