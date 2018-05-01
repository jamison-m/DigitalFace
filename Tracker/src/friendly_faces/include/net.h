#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#define basio boost::asio
#define budp boost::asio::ip::udp

class UDPBroadcast {
private:
	basio::io_service io_service;
	budp::socket socket;
	budp::endpoint endp;
public:
	UDPBroadcast();
	~UDPBroadcast();
	void send(const std::string& msg);
};
