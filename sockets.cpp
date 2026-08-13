#include "sockets.h"
#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)

bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{
	WSADATA wsaData;
	SOCKADDR_IN addrSrv;

	if(!WSAStartup(MAKEWORD(1, 1), &wsaData))
	{
		if( (sock = socket(AF_INET, SOCK_STREAM ,0)) != INVALID_SOCKET )
		{
			addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(Port);
			if (connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR)) != SOCKET_ERROR)
			{
				return true;
			}
			closesocket(sock);
			sock = INVALID_SOCKET;
		}
		WSACleanup();
	}
	return false;
}

void CloseSocket(SOCKET& sock)
{
	closesocket(sock);
	sock = INVALID_SOCKET;
	WSACleanup();
}
