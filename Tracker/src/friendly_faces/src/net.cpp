#include "net.h"

UDPBroadcast::UDPBroadcast() : socket(io_service, budp::endpoint(budp::v4(), 0)) {
	budp::resolver reslv(io_service);
	budp::resolver::query q(budp::v4(),
		boost::asio::ip::host_name(), "8081");
	budp::resolver::iterator iter = reslv.resolve(q);
	endp = *iter;
}

UDPBroadcast::~UDPBroadcast() {
	socket.close();
}

void UDPBroadcast::send(const std::string& msg) {
	socket.send_to(basio::buffer(msg, msg.size()), endp);
}
