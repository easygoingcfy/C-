#include "pinling_protocol.cc"

#include <iostream>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
typedef boost::function<void(const boost::system::error_code&, const int& handle)> Callback;

Callback funcRead_;
Callback funcWrite_;
boost::shared_ptr<boost::asio::io_service> io_service_;
boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;
 
 
void OnSendData(const boost::system::error_code& ErrorCode, const int& handle)
{
	std::cout << "OnSendData" << std::endl;
 
	if (ErrorCode.value() != 0)
	{
		std::cout <<  " Error::Sending Data::" << ErrorCode.message() << std::endl;;
	}
}
 
boost::array<char, 5> bufferRead_;
 
void OnReceiveData(const boost::system::error_code& ErrorCode, const int& handle)
{
	std::cout << "OnReceiveData" << std::endl;
	
	if (ErrorCode.value() == 0)
	{
		for (int i = 0; i < bufferRead_.size(); i++)
		{
			std::cout << bufferRead_[i] << " ";
		}
		std::cout << std::endl;
	}
 
	boost::asio::async_read(*socket_, boost::asio::buffer(bufferRead_, bufferRead_.size()), funcRead_);
}
 
void StartReading(int n)
{
	io_service_->run();
}
 
int main()
{
	io_service_ = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service);
	socket_ = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(*io_service_));
    boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string("192.168.2.119"), 2000);
 
	socket_->connect(ep);
	
	funcWrite_ = boost::bind(OnSendData, boost::asio::placeholders::error, 0);
	funcRead_ = boost::bind(OnReceiveData, boost::asio::placeholders::error, 0);
    io_service_->run();
 
    std::cout << "tag" << std::endl;
	boost::asio::async_read(*socket_, boost::asio::buffer(bufferRead_, bufferRead_.size()), funcRead_);
	std::thread t(StartReading, NULL);
 
	std::vector<char> sendBuffer_(100);
    CombinationControlA cmd = get_cmd();
	while (1)
	{
		//std::cin >> &sendBuffer_[0];
		socket_->async_send(boost::asio::buffer((void*)&cmd, sizeof(cmd)), funcWrite_);
	}
}