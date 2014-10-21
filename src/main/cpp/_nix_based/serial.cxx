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

#include "serial.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

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
