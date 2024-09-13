#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serio.h>
#ifdef WITH_TCP
#include <pty.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <poll.h>
#include <time.h>
#endif

void *attach(void *parm) {
    int tty = *(int *)parm, ldisc = N_MOUSE;
    unsigned long devt = 0xaa5500 | SERIO_SUNKBD;

    if (ioctl(tty, TIOCSETD, &ldisc) < 0) {
	perror("ioctl(TIOCSETD)");
	goto err_close;
    }
    if (ioctl(tty, SPIOCSTYPE, &devt) < 0) {
	perror("ioctl(SPIOCSTYPE)");
	goto err_close_disc;
    }
    do {
	errno = 0;
	read(tty, NULL, 0);
    } while (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK);
err_close_disc:
    ldisc = 0;
    ioctl(tty, TIOCSETD, &ldisc);
err_close:
    close(tty);
    return NULL;
}

void attach_tty(char *path) {
    struct termios ts;
    int tty = open(path, O_RDWR | O_NOCTTY);
    if (tty < 0) {
	perror(path);
	return;
    }
    if (tcgetattr(tty, &ts) < 0) {
	perror("tcgetattr(tty)");
	goto err_close;
    }
    ts.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;
    ts.c_iflag = IGNBRK | IGNPAR;
    ts.c_oflag = 0;
    ts.c_lflag = 0;
    ts.c_cc[VMIN ] = 1;
    ts.c_cc[VTIME] = 0;
    cfsetispeed(&ts, B4800);
    cfsetospeed(&ts, B4800);
    if (tcsetattr(tty, TCSANOW, &ts) < 0) {
	perror("tcsetattr(tty)");
	goto err_close;
    }
    if (write(tty, "\x00", 1) != 1) {
	perror("write(tty)");
	goto err_close;
    }
    usleep(100 * 1000);
    cfsetispeed(&ts, B9600);
    cfsetospeed(&ts, B9600);
    if (tcsetattr(tty, TCSANOW, &ts) < 0) {
	perror("tcsetattr(tty)");
	goto err_close;
    }
    if (write(tty, "\xc1", 1) != 1) {
	perror("write(tty)");
	goto err_close;
    }
    usleep(50 * 1000);
    attach(&tty);
err_close:
    close(tty);
}

#ifdef WITH_TCP
#define KEEPALIVE_IDLE     5
#define KEEPALIVE_INTERVAL 3
#define KEEPALIVE_COUNT    3
void attach_tcp(char *ipport) {
    struct sockaddr_in addr;
    unsigned char d, *ip = (unsigned char *)&addr.sin_addr;
    unsigned short port = 4232;
    pthread_t tid;
    pthread_attr_t attr;
    time_t ts_act = (time_t)0;
    int sock = -1, pty = -1, tty = -1, n, rn, sn;
    char rb[1024], sb[1024];
    struct pollfd p[2];

    memset(p, 0, sizeof(p));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    if (4 != sscanf(ipport, "%hhu.%hhu.%hhu.%hhu%c",
		ip+0, ip+1, ip+2, ip+3, &d) &&
	5 != sscanf(ipport, "%hhu.%hhu.%hhu.%hhu:%hu%c",
		ip+0, ip+1, ip+2, ip+3, &port, &d)) {
	fprintf(stderr, "%s: bad address\n", ipport);
	return;
    }
    addr.sin_port = htons(port);
    if (pthread_attr_init(&attr)) {
	perror("pthread_attr_init()");
	return;
    }
    for (;;) {
	if (sock < 0) {
	    if ((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket()");
		goto err;
	    }
	    n = 1;
	    if (setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &n, sizeof(n)) < 0)
		perror("setsockopt(SO_KEEPALIVE)");
	    if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &n, sizeof(n)) < 0)
		perror("setsockopt(TCP_NODELAY)");
	    n = KEEPALIVE_IDLE;
	    if (setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &n, sizeof(n)) < 0)
		perror("setsockopt(TCP_KEEPIDLE)");
	    n = KEEPALIVE_INTERVAL;
	    if (setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &n, sizeof(n)) < 0)
		perror("setsockopt(TCP_KEEPINTVL)");
	    n = KEEPALIVE_COUNT;
	    if (setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &n, sizeof(n)) < 0)
		perror("setsockopt(TCP_KEEPCNT)");
	    n = KEEPALIVE_IDLE + KEEPALIVE_INTERVAL * KEEPALIVE_COUNT - 1;
	    if (setsockopt(sock, IPPROTO_TCP, TCP_USER_TIMEOUT, &n, sizeof(n)) < 0)
		perror("setsockopt(TCP_USER_TIMEOUT)");
	    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 &&
				errno != EINPROGRESS) {
		perror("connect()");
		goto err_close_sock;
	    }
	    if ((n = fcntl(sock, F_GETFL)) < 0 ||
		    fcntl(sock, F_SETFL, n | O_NONBLOCK) < 0) {
		perror("fcntl(sock)");
		goto err_close_sock;
	    }
	    rn = sn = 0;
	    p[0].fd = sock;
	    ts_act = time(NULL);
	}
	if (pty < 0) {
	    if (openpty(&pty, &tty, NULL, NULL, NULL) < 0) {
		perror("openpty()");
		pty = tty = -1;
		goto err;
	    }
	    if ((n = fcntl(pty, F_GETFL)) < 0 ||
		    fcntl(pty, F_SETFL, n | O_NONBLOCK) < 0) {
		perror("fcntl(pty)");
		goto err_close_pty;
	    }
	    if (pthread_create(&tid, &attr, &attach, &tty)) {
		perror("pthread_create()");
		goto err_close_pty;
	    }
	    p[1].fd = pty;
	}
	for (;;) {
	    p[0].events = p[1].events = 0;
	    if (rn < sizeof(rb))
		p[0].events |= POLLIN;
	    if (sn < sizeof(sb))
		p[1].events |= POLLIN;
	    if (sn)
		p[0].events |= POLLOUT;
	    if (rn)
		p[1].events |= POLLOUT;
	    n = poll(p, 2, -1);
	    ts_act = time(NULL);
	    if (n < 0) {
		if (errno == EINTR || errno == EAGAIN)
		    continue;
		perror("poll()");
		goto err;
	    }
	    if (p[0].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)) {
		if ((n = recv(sock, rb + rn, sizeof(rb) - rn, 0)) <= 0) {
		    if (n == 0)
			fprintf(stderr, "recv(): EOF\n");
		    else
			perror("recv()");
		    goto err_close_sock;
		}
		rn += n;
	    }
	    if (p[1].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)) {
		if ((n = read(pty, sb + sn, sizeof(sb) - sn)) < 0) {
		    perror("read(pty)");
		    goto err_close_sock_pty;
		}
		sn += n;
	    }
	    if (sn) {
		if ((n = send(sock, sb, sn, 0)) < 0 && errno != EINTR &&
				errno != EAGAIN && errno != EWOULDBLOCK) {
		    perror("send()");
		    goto err_close_sock;
		}
		if (n > 0)
		    memmove(sb, sb + n, sn -= n);
	    }
	    if (rn) {
		if ((n = write(pty, rb, rn)) < 0 && errno != EINTR &&
				errno != EAGAIN && errno != EWOULDBLOCK) {
		    perror("write(pty)");
		    goto err_close_sock_pty;
		}
		if (n > 0)
		    memmove(rb, rb + n, rn -= n);
	    }
	}
err_close_pty:
	close(pty);
	close(tty);
	pty = tty = -1;
	goto err;
err_close_sock_pty:
	close(pty);
	close(tty);
	pty = tty = -1;
err_close_sock:
	close(sock);
	sock = -1;
err:
	if (pty >= 0 && time(NULL) - ts_act >= 120) {
	    close(pty);
	    close(tty);
	    pty = tty = -1;
	}
	if (sock >= 0 && send(sock, sb, 0, 0) < 0 && errno != EINTR &&
				errno != EAGAIN && errno != EWOULDBLOCK) {
	    perror("send()");
	    close(sock);
	    sock = -1;
	}
	sleep(1);
    }
}
#endif

int main(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "-f")) {
	argc--;
	argv++;
    } else if (daemon(0, 0) < 0) {
	return 1;
    }
    if (argc < 2)
	attach_tty("/dev/owbus0");
#ifdef WITH_TCP
    else if (*argv[1] != '/')
	attach_tcp(argv[1]);
#endif
    else
	attach_tty(argv[1]);
    return 0;
}
