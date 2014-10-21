int openPort(const char* port);
int closePort(int fd);
int setExclusiveAccess(int fd, int enable);
speed_t getBaudRateByNum(int baudRate);
int getDataBitsByNum(int byteSize);
int setParams(int fd, int baudRate, int byteSize, int stopBits, int parity, int setRTS, int setDTR, int flags);
int purgePort(int fd, int flags);
int setRTS(int fd, int enabled);
int setDTR(int fd, int enabled);
int writeBytes(int fd, const void* buf, size_t n);
int readBytes(int fd, void* buf, size_t nbytes);
int getBufferBytesCountIn(int fd);
int getBufferBytesCountOut(int fd);
int setFlowControlMode(int fd, int mask);
int getFlowControlMode(int fd, int* mask);
int sendBreak(int fd, int duration);

typedef struct line_status_t {
    int l_cts;
    int l_dsr;
    int l_ring;
    int l_rlsd;
} line_status_t;

typedef struct interrupt_t {
    int i_break;
    int i_tx;
    int i_frame;
    int i_overrun;
    int i_parity;
} interrupt_t;

int getLinesStatus(int fd, line_status_t* l);
int getInterruptsCount(int fd, interrupt_t* i);