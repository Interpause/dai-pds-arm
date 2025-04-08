#pragma once

#include <SimpleSerialProtocol.h>

#ifndef N_SERVO
#define N_SERVO 16
#endif

#define USB_CMD_HANDSHAKE 's'
#define USB_CMD_HEARTBEAT 'h'
#define USB_CMD_RECV_SERVO 'w'
#define USB_CMD_SEND_SERVO 'r'
#define USB_CMD_SEND_DEBUG 'd'

#define USB_HANDSHAKE_PERIOD 200  // ms
#define USB_HEARTBEAT_PERIOD 200  // ms
#define USB_HEARTBEAT_TIMEOUT 400 // ms
#define USB_HEARTBEAT_MISS 2      // times

/** Due to limitations with SSP's callbacks, have to implement as static namespace. */
namespace USB
{
    struct handlers_t
    {
        void (*callbackConnState)(bool is_connected) = nullptr;
        void (*setServos)(uint16_t (&pulsewidths)[N_SERVO]) = nullptr;
    };

    extern bool is_connected;
    extern bool has_fatal;
    extern uint8_t err_num;
    extern handlers_t h;
    extern SimpleSerialProtocol *sp;

    void setup(SimpleSerialProtocol *ssp);
    void loop();
    void onError(uint8_t err_num);
    void sendHandshake();
    void recvHandshake();
    void sendHeartbeat();
    void recvHeartbeat();
    void recvServos();
    void sendServos(uint16_t (&pulsewidths)[N_SERVO]);
    void sendDebug(const char *msg);
}

/** Append string with optional format arg to msg. */
void appendf(char *msg, size_t size, const char *fmt, ...);
