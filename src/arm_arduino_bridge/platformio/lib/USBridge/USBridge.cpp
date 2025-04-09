#include "USBridge.h"

// DEBUG: Checksum to verify serial integrity.
#define USE_CHECKSUM 1
// DEBUG: Print servo values.
#define PRINT_SERVO 0

// Private variables.
uint32_t _last_hb_recv = 0;
uint32_t _last_hb_sent = 0;
uint8_t _hb_misses = 0;
uint32_t _last_handshake = 0;

#define GUARD_SETUP()       \
    if (USB::sp == nullptr) \
        return;
// TODO: GUARD_ERROR is bad cause it will accumulate invalid bytes in the serial buffer.
#define GUARD_ERROR()               \
    if (false && USB::err_num != 0) \
        return;

// Extern variables must be defined exactly once.
bool USB::is_connected = false;
bool USB::has_fatal = false;
uint8_t USB::err_num = 0;
USB::handlers_t USB::h{};
SimpleSerialProtocol *USB::sp = nullptr;

// I don't think we will run into it, but millis() can roll over.
// NOTE: Must read eot for even argument-less commands.

void _setConnState(bool is_connected)
{
    if (USB::h.callbackConnState != nullptr)
        USB::h.callbackConnState(is_connected);
    USB::is_connected = is_connected;
}

void _heartbeat()
{
    if (!USB::is_connected)
        return;

    auto now = millis();

    // Disconnect if no heartbeat received for too long.
    if ((now > _last_hb_recv) && (now - _last_hb_recv > USB_HEARTBEAT_TIMEOUT))
    {
        _hb_misses++;
        _last_hb_recv = now;
    }

    if (_hb_misses >= USB_HEARTBEAT_MISS)
        _setConnState(false);

    // Send heartbeat.
    if ((now > _last_hb_sent) && (now - _last_hb_sent > USB_HEARTBEAT_PERIOD))
        USB::sendHeartbeat();
}

void USB::setup(SimpleSerialProtocol *ssp)
{
    sp = ssp;
    sp->init();
    sp->setDieInstantlyOnNotRegisteredCommand(false);
    sp->registerCommand(USB_CMD_HANDSHAKE, recvHandshake);
    sp->registerCommand(USB_CMD_HEARTBEAT, recvHeartbeat);
    sp->registerCommand(USB_CMD_RECV_SERVO, recvServos);
    has_fatal = false;
}

void USB::loop()
{
    GUARD_SETUP();
    auto now = millis();

    _heartbeat();

    // If not connected, send handshake (repeatedly).
    if (!is_connected && (now > _last_handshake) && (now - _last_handshake > USB_HANDSHAKE_PERIOD))
    {
        sendHandshake();
        _last_handshake = now;
    }

    sp->loop();
}

/** See full list: https://github.com/yesbotics/simple-serial-protocol-arduino/blob/9c604fb26f1fc0987ebfb73abc116412a14cdeaa/src/SimpleSerialProtocol.h#L19-L30 */
void USB::onError(uint8_t err_num)
{
    GUARD_SETUP();
    // _g->err_num = err_num;

    // TODO: blinker.
    // Handle errors and send debug message.
    char msg[512];
    memset(msg, 0, sizeof(msg));
    appendf(msg, sizeof(msg), "[USB] err %d: ", err_num);
    switch (err_num)
    {
    case ERROR_IS_DEAD:
        appendf(msg, sizeof(msg), "Earlier error was fatal, will attempt restart.");
        has_fatal = true; // Let main.cpp figure this out.
        break;
    case ERROR_EOT_WAS_NOT_READ:
        appendf(msg, sizeof(msg), "Forgot sp->readEot() somewhere?");
        break;
    case ERROR_WAIT_FOR_BYTE_TIMEOUT:
        appendf(msg, sizeof(msg), "Timeout waiting for byte.");
        break;
    case ERROR_COMMAND_IS_NOT_IN_RESERVED_RANGE:
        appendf(msg, sizeof(msg), "Invalid cmd byte; Likely a msg got interrupted.");
        break;
    case ERROR_IS_NOT_EOT:
        appendf(msg, sizeof(msg), "Not an EOT byte; Is the msg structure correct?");
        break;
    // Errors we haven't bothered to handle yet.
    default:
        appendf(msg, sizeof(msg), "Unknown error %d, see: https://github.com/yesbotics/simple-serial-protocol-arduino/blob/29559284260a2c4380ef10c7ab40c75b806787ff/src/ssp_common.h#L35-L46", err_num);
        break;
    }
    sendDebug(msg);
}

void USB::sendHandshake()
{
    GUARD_SETUP();
    sp->writeCommand(USB_CMD_HANDSHAKE);
    sp->writeEot();
}

void USB::recvHandshake()
{
    GUARD_SETUP();
    sp->readEot();
    _setConnState(true);
    err_num = 0;
    _last_hb_recv = millis();
    _last_hb_sent = 0;
    _hb_misses = 0;
}

void USB::sendHeartbeat()
{
    GUARD_SETUP();
    sp->writeCommand(USB_CMD_HEARTBEAT);
    sp->writeEot();
    _last_hb_sent = millis();
}

void USB::recvHeartbeat()
{
    GUARD_SETUP();
    sp->readEot();
    if (!is_connected)
        return;
    _last_hb_recv = millis();
    _hb_misses = 0;
}

void USB::recvServos()
{
    GUARD_SETUP();
    // Servo handler not set yet, exit early.
    auto setServos = h.setServos;
    if (setServos == nullptr)
        return;

    // Right after every read, we check if there was an error.
    err_num = 0;
    uint16_t pulsewidths[N_SERVO];
    for (auto i = 0; i < N_SERVO; i++)
    {
        pulsewidths[i] = sp->readUnsignedInt16();
        GUARD_ERROR();
    }

// DEBUG: Check checksum to verify serial integrity.
#if USE_CHECKSUM
    uint16_t cksum = sp->readUnsignedInt16();
    GUARD_ERROR();
    uint32_t local_cksum = 0;
    for (auto i = 0; i < N_SERVO; i++)
        local_cksum += pulsewidths[i];
    local_cksum = local_cksum % 65536;
    if (local_cksum != cksum)
    {
        sp->readEot();
        return;
    }
#endif

    sp->readEot();
    GUARD_ERROR();

    // We only make it to the handler if there were no errors in receiving.
    setServos(pulsewidths);

    // DEBUG: echo back the servo values.
#if PRINT_SERVO
    char msg[64];
    memset(msg, 0, sizeof(msg));
    appendf(msg, sizeof(msg), "[USB] servo:");
    for (auto i = 0; i < N_SERVO; i++)
        appendf(msg, sizeof(msg), " %d=%d", i + 1, pulsewidths[i]);
    sendDebug(msg);
    delete[] msg;
#endif
}

void USB::sendServos(uint16_t (&pulsewidths)[N_SERVO])
{
    GUARD_SETUP();
    if (!is_connected)
        return;
    sp->writeCommand(USB_CMD_SEND_SERVO);
    for (auto i = 0; i < N_SERVO; i++)
        sp->writeUnsignedInt16(pulsewidths[i]);
#if USE_CHECKSUM
    uint32_t cksum = 0;
    for (auto i = 0; i < N_SERVO; i++)
        cksum += pulsewidths[i];
    cksum = cksum % 65536;
    sp->writeUnsignedInt16(cksum);
#endif
    sp->writeEot();
}

void USB::sendDebug(const char *msg)
{
    GUARD_SETUP();
    if (!is_connected)
        return;
    sp->writeCommand(USB_CMD_SEND_DEBUG);
    sp->writeCString(msg);
    sp->writeEot();
}

void appendf(char *msg, size_t size, const char *fmt, ...)
{
    size_t len = strlen(msg);
    if (len >= size - 1)
        return;

    va_list args;
    va_start(args, fmt);
    int ret = vsnprintf(msg + len, size - len, fmt, args);
    va_end(args);

    size_t new_len = strlen(msg);
    if (new_len < size)
        memset(msg + new_len, 0, size - new_len);
}
