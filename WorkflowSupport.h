#pragma once

#include "ApplicationConfig.h"
#include "sockets.h"

#include <cstdio>
#include <memory>
#include <string>

using FileHandle = std::unique_ptr<FILE, decltype(&std::fclose)>;

FileHandle OpenFile(const std::string& path, const char* mode);
std::string MakeTimestampedFilename(const char* pattern);

class SocketHandle
{
public:
    SocketHandle() = default;
    ~SocketHandle();

    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;

    bool Open(const NetworkEndpoint& endpoint);
    SOCKET Get() const;

private:
    SOCKET socket_ = INVALID_SOCKET;
};
