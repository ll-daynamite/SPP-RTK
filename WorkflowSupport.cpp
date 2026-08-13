#include "WorkflowSupport.h"

#include <ctime>

FileHandle OpenFile(const std::string& path, const char* mode)
{
    FILE* file = nullptr;
    if (fopen_s(&file, path.c_str(), mode) != 0)
        file = nullptr;
    return FileHandle(file, &std::fclose);
}

std::string MakeTimestampedFilename(const char* pattern)
{
    char filename[128] = {};
    const std::time_t now = std::time(nullptr);
    std::tm localTime = {};
    localtime_s(&localTime, &now);
    std::strftime(filename, sizeof(filename), pattern, &localTime);
    return filename;
}

SocketHandle::~SocketHandle()
{
    if (socket_ != INVALID_SOCKET)
        CloseSocket(socket_);
}

bool SocketHandle::Open(const NetworkEndpoint& endpoint)
{
    return OpenSocket(socket_, endpoint.address.c_str(), endpoint.port);
}

SOCKET SocketHandle::Get() const
{
    return socket_;
}
