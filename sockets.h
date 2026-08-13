#pragma once
#include <WinSock2.h>
#include <Windows.h>

bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);
void CloseSocket(SOCKET& sock);
