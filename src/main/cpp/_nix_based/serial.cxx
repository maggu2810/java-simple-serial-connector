/* jSSC (Java Simple Serial Connector) - serial port communication library.
 * © Alexey Sokolov (scream3r), 2010-2014.
 *
 * This file is part of jSSC.
 *
 * jSSC is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jSSC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with jSSC.  If not, see <http://www.gnu.org/licenses/>.
 *
 * If you use jSSC in public project you can inform me about this by e-mail,
 * of course if you want it.
 *
 * e-mail: scream3r.org@gmail.com
 * web-site: http://scream3r.org | http://code.google.com/p/java-simple-serial-connector/
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>//-D_TS_ERRNO use for Solaris C++ compiler
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>

#include <sys/select.h>//since 2.5.0

#ifdef __linux__
#include <linux/serial.h>
#endif
#ifdef __SunOS
#include <sys/filio.h>//Needed for FIONREAD in Solaris
#include <string.h>//Needed for select() function
#endif
#ifdef __APPLE__
#include <serial/ioss.h>//Needed for IOSSIOSPEED in Mac OS X (Non standard baudrate)
#endif

#include "serial.h"

/*!
 * open serial port
 * 
 * @param port The serial port (device path) that should be opened (e.g. /dev/ttyS0).
 * @return For documentation of return value and also error number, see open.
 */
int openPort(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    return fd;
}

int closePort(int fd) {
    return close(fd);
}

/*!
 * set / unset exclusive access.
 *
 * @param fd The file descriptor of the port.
 * @return For documentation of return value and also error number, see ioctl.
 */
int setExclusiveAccess(int fd, int enable) {
    if (enable) {
#if defined TIOCEXCL //&& !defined __SunOS
        return ioctl(fd, TIOCEXCL);
#else
        errno = ENOTSUP;
        return -1;
#endif
    } else {
#if defined TIOCNXCL
        return ioctl(fd, TIOCNXCL);
#else
        errno = ENOTSUP;
        return -1;
#endif
    }
}

/*
 * Choose baudrate
 */
speed_t getBaudRateByNum(int baudRate) {
    switch (baudRate) {
        case 0:
            return B0;
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
#ifdef B57600
        case 57600:
            return B57600;
#endif
#ifdef B115200
        case 115200:
            return B115200;
#endif
#ifdef B230400
        case 230400:
            return B230400;
#endif
#ifdef B460800
        case 460800:
            return B460800;
#endif

#ifdef B500000
        case 500000:
            return B500000;
#endif
#ifdef B576000
        case 576000:
            return B576000;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
#ifdef B1000000
        case 1000000:
            return B1000000;
#endif

#ifdef B1152000
        case 1152000:
            return B1152000;
#endif
#ifdef B1500000
        case 1500000:
            return B1500000;
#endif
#ifdef B2000000
        case 2000000:
            return B2000000;
#endif
#ifdef B2500000
        case 2500000:
            return B2500000;
#endif

#ifdef B3000000
        case 3000000:
            return B3000000;
#endif
#ifdef B3500000
        case 3500000:
            return B3500000;
#endif
#ifdef B4000000
        case 4000000:
            return B4000000;
#endif
        default:
            return -1;
    }
}

/*
 * Choose data bits
 */
int getDataBitsByNum(int byteSize) {
    switch (byteSize) {
        case 5:
            return CS5;
        case 6:
            return CS6;
        case 7:
            return CS7;
        case 8:
            return CS8;
        default:
            return -1;
    }
}

//since 2.6.0 ->
const int PARAMS_FLAG_IGNPAR = 1;
const int PARAMS_FLAG_PARMRK = 2;
//<- since 2.6.0

/*!
 * Set device parameter
 * 
 * @param fd The file descriptor of the serial device.
 * @param baudRate
 * @param byteSize
 * @param stopBits
 * @param parity
 * @param setRTS
 * @param setDTR
 * @return Return 0 on success, -1 on error.
 */
int setParams(int fd,
        int baudRate, int byteSize, int stopBits, int parity,
        int setRTS, int setDTR,
        int flags) {
    int rv = -1;

    const speed_t baudRateValue = getBaudRateByNum(baudRate);
    const int dataBits = getDataBitsByNum(byteSize);

    termios settings;
    if (tcgetattr(fd, &settings) == 0) {
        if (baudRateValue != -1) {
            //Set standard baudrate from "termios.h"
            if (cfsetispeed(&settings, baudRateValue) < 0 || cfsetospeed(&settings, baudRateValue) < 0) {
                goto methodEnd;
            }
        } else {
#ifdef __SunOS
            goto methodEnd; //Solaris don't support non standard baudrates
#elif defined __linux__
            //Try to calculate a divisor for setting non standard baudrate
            serial_struct serial_info;
            if (ioctl(fd, TIOCGSERIAL, &serial_info) < 0) { //Getting serial_info structure
                goto methodEnd;
            } else {
                serial_info.flags |= ASYNC_SPD_CUST;
                serial_info.custom_divisor = (serial_info.baud_base / baudRate); //Calculate divisor
                if (serial_info.custom_divisor == 0) { //If divisor == 0 go to method end to prevent "division by zero" error
                    goto methodEnd;
                }
                settings.c_cflag |= B38400;
                if (cfsetispeed(&settings, B38400) < 0 || cfsetospeed(&settings, B38400) < 0) {
                    goto methodEnd;
                }
                if (ioctl(fd, TIOCSSERIAL, &serial_info) < 0) {//Try to set new settings with non standard baudrate
                    goto methodEnd;
                }
            }
#endif
        }
    }

    /*
     * Setting data bits
     */
    if (dataBits != -1) {
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= dataBits;
    } else {
        goto methodEnd;
    }

    /*
     * Setting stop bits
     */
    if (stopBits == 0) { //1 stop bit (for info see ->> MSDN)
        settings.c_cflag &= ~CSTOPB;
    } else if ((stopBits == 1) || (stopBits == 2)) { //1 == 1.5 stop bits; 2 == 2 stop bits (for info see ->> MSDN)
        settings.c_cflag |= CSTOPB;
    } else {
        goto methodEnd;
    }

    settings.c_cflag |= (CREAD | CLOCAL);
    settings.c_cflag &= ~CRTSCTS;
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL | ECHOPRT | ECHOKE | ISIG | IEXTEN);

    settings.c_iflag &= ~(IXON | IXOFF | IXANY | INPCK | IGNPAR | PARMRK | ISTRIP | IGNBRK | BRKINT | INLCR | IGNCR | ICRNL);
#ifdef IUCLC
    settings.c_iflag &= ~IUCLC;
#endif
    settings.c_oflag &= ~OPOST;

    //since 2.6.0 ->
    if ((flags & PARAMS_FLAG_IGNPAR) == PARAMS_FLAG_IGNPAR) {
        settings.c_iflag |= IGNPAR;
    }
    if ((flags & PARAMS_FLAG_PARMRK) == PARAMS_FLAG_PARMRK) {
        settings.c_iflag |= PARMRK;
    }
    //<- since 2.6.0

    //since 0.9 ->
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    //<- since 0.9

    /*
     * Parity bits
     */
#ifdef PAREXT
    settings.c_cflag &= ~(PARENB | PARODD | PAREXT); //Clear parity settings
#elif defined CMSPAR
    settings.c_cflag &= ~(PARENB | PARODD | CMSPAR); //Clear parity settings
#else
    settings.c_cflag &= ~(PARENB | PARODD); //Clear parity settings
#endif
    if (parity == 1) {//Parity ODD
        settings.c_cflag |= (PARENB | PARODD);
        settings.c_iflag |= INPCK;
    } else if (parity == 2) {//Parity EVEN
        settings.c_cflag |= PARENB;
        settings.c_iflag |= INPCK;
    } else if (parity == 3) {//Parity MARK
#ifdef PAREXT
        settings.c_cflag |= (PARENB | PARODD | PAREXT);
        settings.c_iflag |= INPCK;
#elif defined CMSPAR
        settings.c_cflag |= (PARENB | PARODD | CMSPAR);
        settings.c_iflag |= INPCK;
#endif
    } else if (parity == 4) {//Parity SPACE
#ifdef PAREXT
        settings.c_cflag |= (PARENB | PAREXT);
        settings.c_iflag |= INPCK;
#elif defined CMSPAR
        settings.c_cflag |= (PARENB | CMSPAR);
        settings.c_iflag |= INPCK;
#endif
    } else if (parity == 0) {
        //Do nothing (Parity NONE)
    } else {
        goto methodEnd;
    }

    if (tcsetattr(fd, TCSANOW, &settings) == 0) {//Try to set all settings
#ifdef __APPLE__
        //Try to set non-standard baud rate in Mac OS X
        if (baudRateValue == -1) {
            speed_t speed = (speed_t) baudRate;
            if (ioctl(fd, IOSSIOSPEED, &speed) < 0) {//IOSSIOSPEED must be used only after tcsetattr
                goto methodEnd;
            }
        }
#endif
        int lineStatus;
        if (ioctl(fd, TIOCMGET, &lineStatus) >= 0) {
            if (setRTS) {
                lineStatus |= TIOCM_RTS;
            } else {
                lineStatus &= ~TIOCM_RTS;
            }
            if (setDTR) {
                lineStatus |= TIOCM_DTR;
            } else {
                lineStatus &= ~TIOCM_DTR;
            }
            if (ioctl(fd, TIOCMSET, &lineStatus) >= 0) {
                rv = 0;
            }
        }
    }

methodEnd:
    return rv;
}

const int PURGE_RXABORT = 0x0002; //ignored
const int PURGE_RXCLEAR = 0x0008;
const int PURGE_TXABORT = 0x0001; //ignored
const int PURGE_TXCLEAR = 0x0004;

/*
 * @return Return 0 on success. -1 on error.
 */
int purgePort(int fd, int flags) {
    int clearValue = -1;

    if ((flags & PURGE_RXCLEAR) && (flags & PURGE_TXCLEAR)) {
        clearValue = TCIOFLUSH;
    } else if (flags & PURGE_RXCLEAR) {
        clearValue = TCIFLUSH;
    } else if (flags & PURGE_TXCLEAR) {
        clearValue = TCOFLUSH;
    } else if ((flags & PURGE_RXABORT) || (flags & PURGE_TXABORT)) {
        return 0;
    } else {
        return -1;
    }
    return tcflush(fd, clearValue) == 0 ? 0 : -1;
}

int setRTS(int fd, int enabled) {
    int returnValue;
    int lineStatus;

    ioctl(fd, TIOCMGET, &lineStatus);

    if (enabled) {
        lineStatus |= TIOCM_RTS;
    } else {
        lineStatus &= ~TIOCM_RTS;
    }

    returnValue = ioctl(fd, TIOCMSET, &lineStatus);

    return returnValue >= 0 ? 0 : -1;
}

int setDTR(int fd, int enabled) {
    int returnValue;
    int lineStatus;

    ioctl(fd, TIOCMGET, &lineStatus);

    if (enabled) {
        lineStatus |= TIOCM_DTR;
    } else {
        lineStatus &= ~TIOCM_DTR;
    }

    returnValue = ioctl(fd, TIOCMSET, &lineStatus);

    return returnValue >= 0 ? 0 : -1;
}

/*
 * @return Return the number of bytes written.
 */
int writeBytes(int fd, const void* buf, size_t n) {
    int rv;

    rv = write(fd, buf, n);

    return rv;
}

/*
 * TODO: Fix implementation
 * @return Return the number of read bytes.
 */
int readBytes(int fd, void* buf, size_t nbytes) {
    fd_set read_fd_set;
    int byteRemains = nbytes;
    while (byteRemains > 0) {
        FD_ZERO(&read_fd_set);
        FD_SET(fd, &read_fd_set);
        select(fd + 1, &read_fd_set, NULL, NULL, NULL);
        
        int result = read(fd, &((char*)buf)[nbytes - byteRemains], byteRemains);
        if (result > 0) {
            byteRemains -= result;
        }
    }
    FD_CLR(fd, &read_fd_set);
    return nbytes;
}

/*
 * @return Return the number of bytes in input buffer on success, -1 on failure.
 */
int getBufferBytesCountIn(int fd) {
    int cnt;

    if (ioctl(fd, FIONREAD, &cnt) == -1) {
        return -1;
    } else {
        return cnt;
    }
}

/*
 * @return Return the number of bytes in output buffer on success, -1 on failure.
 */
int getBufferBytesCountOut(int fd) {
    int cnt;

    if (ioctl(fd, TIOCOUTQ, &cnt) == -1) {
        return -1;
    } else {
        return cnt;
    }
}

const int FLOWCONTROL_NONE = 0;
const int FLOWCONTROL_RTSCTS_IN = 1;
const int FLOWCONTROL_RTSCTS_OUT = 2;
const int FLOWCONTROL_XONXOFF_IN = 4;
const int FLOWCONTROL_XONXOFF_OUT = 8;

/*
 * @return Return 0 on success, -1 on error.
 */
int setFlowControlMode(int fd, int mask) {
    termios settings;
    if (tcgetattr(fd, &settings) == 0) {
        settings.c_cflag &= ~CRTSCTS;
        settings.c_iflag &= ~(IXON | IXOFF);
        if (mask != FLOWCONTROL_NONE) {
            if (((mask & FLOWCONTROL_RTSCTS_IN) == FLOWCONTROL_RTSCTS_IN) || ((mask & FLOWCONTROL_RTSCTS_OUT) == FLOWCONTROL_RTSCTS_OUT)) {
                settings.c_cflag |= CRTSCTS;
            }
            if ((mask & FLOWCONTROL_XONXOFF_IN) == FLOWCONTROL_XONXOFF_IN) {
                settings.c_iflag |= IXOFF;
            }
            if ((mask & FLOWCONTROL_XONXOFF_OUT) == FLOWCONTROL_XONXOFF_OUT) {
                settings.c_iflag |= IXON;
            }
        }
        if (tcsetattr(fd, TCSANOW, &settings) == 0) {
            return 0;
        }
    }

    return -1;
}

/*
 * @return Return 0 on success (value pointing to mask is set), -1 on error.
 */
int getFlowControlMode(int fd, int* mask) {
    termios settings;

    *mask = 0;

    if (tcgetattr(fd, &settings) == 0) {
        if (settings.c_cflag & CRTSCTS) {
            *mask |= (FLOWCONTROL_RTSCTS_IN | FLOWCONTROL_RTSCTS_OUT);
        }
        if (settings.c_iflag & IXOFF) {
            *mask |= FLOWCONTROL_XONXOFF_IN;
        }
        if (settings.c_iflag & IXON) {
            *mask |= FLOWCONTROL_XONXOFF_OUT;
        }

        return 0;
    } else {
        return -1;
    }
}

/*
 * @return Return 0 on success, -1 on error.
 */
int sendBreak(int fd, int duration) {
    if(duration > 0) {
        if (ioctl(fd, TIOCSBRK, 0) >= 0) {
            int sec = (duration >= 1000 ? duration / 1000 : 0);
            int nanoSec = (sec > 0 ? duration - sec * 1000 : duration) * 1000000;
            struct timespec timeStruct;
            timeStruct.tv_sec = sec;
            timeStruct.tv_nsec = nanoSec;
            nanosleep(&timeStruct, NULL);
            if(ioctl(fd, TIOCCBRK, 0) >= 0){
                return 0;
            }
        }
    }
    return -1;
}

/*
 * @return Return 0 on success (value pointing to l is set), -1 on error.
 */
int getLinesStatus(int fd, line_status_t* l) {
    int status;
    if (ioctl(fd, TIOCMGET, status) >= 0) {
        l->l_cts = status & TIOCM_CTS ? 1 : 0;
        l->l_dsr = status & TIOCM_DSR ? 1 : 0;
        l->l_ring = status & TIOCM_RNG ? 1 : 0;
        l->l_rlsd = status & TIOCM_CAR ? 1 : 0;
        return 0;
    } else {
        return -1;
    }
}

/*
 * Get the interrupt counters
 * 
 * @param fd The file descriptor of the serial device.
 * @param i
 * @return Return 0 on success, -1 on error.
 * 
 * Not supported in Solaris and Mac OS X
 * 
 * Get interrupts count for:
 * 0 - Break(for BREAK event)
 * 1 - TX(for TXEMPTY event)
 * --ERRORS(for ERR event)--
 * 2 - Frame
 * 3 - Overrun
 * 4 - Parity
 */
int getInterruptsCount(int fd, interrupt_t* i) {
#ifdef TIOCGICOUNT
    struct serial_icounter_struct icount;
    if(ioctl(fd, TIOCGICOUNT, &icount) >= 0){
        i->i_break   = icount.brk;
        i->i_tx      = icount.tx;
        i->i_frame   = icount.frame;
        i->i_overrun = icount.overrun;
        i->i_parity  = icount.parity;
        return 0;
    } else {
        return -1;
    }
#else
    return -1;
#endif
}
