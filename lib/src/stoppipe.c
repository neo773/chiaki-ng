// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#include <chiaki/stoppipe.h>
#include <chiaki/sock.h>
#include <chiaki/time.h>

#include <fcntl.h>

#ifdef _WIN32
#include <ws2tcpip.h>
#elif defined(__APPLE__)
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/event.h>
#include <sys/time.h>
#define CHIAKI_STOP_PIPE_USER_IDENT 1U
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#endif

CHIAKI_EXPORT ChiakiErrorCode chiaki_stop_pipe_init(ChiakiStopPipe *stop_pipe)
{
#ifdef _WIN32
	stop_pipe->event = WSACreateEvent();
	if(stop_pipe->event == WSA_INVALID_EVENT)
		return CHIAKI_ERR_UNKNOWN;
#elif defined(__APPLE__)
	stop_pipe->kq = kqueue();
	if(stop_pipe->kq < 0)
		return CHIAKI_ERR_UNKNOWN;
	fcntl(stop_pipe->kq, F_SETFD, FD_CLOEXEC);
	__atomic_store_n(&stop_pipe->stopped, 0, __ATOMIC_RELAXED);
	struct kevent kev;
	EV_SET(&kev, CHIAKI_STOP_PIPE_USER_IDENT, EVFILT_USER, EV_ADD | EV_CLEAR, 0, 0, NULL);
	if(kevent(stop_pipe->kq, &kev, 1, NULL, 0, NULL) < 0)
	{
		close(stop_pipe->kq);
		stop_pipe->kq = -1;
		return CHIAKI_ERR_UNKNOWN;
	}
#elif defined(__SWITCH__)
	// currently pipe or socketpare are not available on switch
	// use a custom udp socket as pipe

	// struct sockaddr_in addr;
	int addr_size = sizeof(stop_pipe->addr);

	stop_pipe->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(stop_pipe->fd < 0)
		return CHIAKI_ERR_UNKNOWN;
	stop_pipe->addr.sin_family = AF_INET;
	// bind to localhost
	stop_pipe->addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	// use a random port (dedicate one socket per object)
	stop_pipe->addr.sin_port = htons(0);
	// bind on localhost
	bind(stop_pipe->fd, (struct sockaddr *) &stop_pipe->addr, addr_size);
	// listen
	getsockname(stop_pipe->fd, (struct sockaddr *) &stop_pipe->addr, &addr_size);
	int r = fcntl(stop_pipe->fd, F_SETFL, O_NONBLOCK);
	if(r == -1)
	{
		close(stop_pipe->fd);
		return CHIAKI_ERR_UNKNOWN;
	}
#else
	int r = pipe(stop_pipe->fds);
	if(r < 0)
		return CHIAKI_ERR_UNKNOWN;
	r = fcntl(stop_pipe->fds[0], F_SETFL, O_NONBLOCK);
	if(r == -1)
	{
		close(stop_pipe->fds[0]);
		close(stop_pipe->fds[1]);
		return CHIAKI_ERR_UNKNOWN;
	}
#endif
	return CHIAKI_ERR_SUCCESS;
}

CHIAKI_EXPORT void chiaki_stop_pipe_fini(ChiakiStopPipe *stop_pipe)
{
#ifdef _WIN32
	WSACloseEvent(stop_pipe->event);
#elif defined(__APPLE__)
	if(stop_pipe->kq >= 0)
	{
		close(stop_pipe->kq);
		stop_pipe->kq = -1;
	}
#elif defined(__SWITCH__)
	close(stop_pipe->fd);
#else
	close(stop_pipe->fds[0]);
	close(stop_pipe->fds[1]);
#endif
}

CHIAKI_EXPORT void chiaki_stop_pipe_stop(ChiakiStopPipe *stop_pipe)
{
#ifdef _WIN32
	WSASetEvent(stop_pipe->event);
#elif defined(__APPLE__)
	__atomic_store_n(&stop_pipe->stopped, 1, __ATOMIC_RELEASE);
	struct kevent kev;
	EV_SET(&kev, CHIAKI_STOP_PIPE_USER_IDENT, EVFILT_USER, 0, NOTE_TRIGGER, 0, NULL);
	kevent(stop_pipe->kq, &kev, 1, NULL, 0, NULL);
#elif defined(__SWITCH__)
	// send to local socket (FIXME MSG_CONFIRM)
	sendto(stop_pipe->fd, "\x00", 1, 0,
		(struct sockaddr*)&stop_pipe->addr, sizeof(struct sockaddr_in));
#else
	write(stop_pipe->fds[1], "\x00", 1);
#endif
}

CHIAKI_EXPORT ChiakiErrorCode chiaki_stop_pipe_select_single(ChiakiStopPipe *stop_pipe, chiaki_socket_t fd, bool write, uint64_t timeout_ms)
{
	uint64_t timeout_us = timeout_ms == UINT64_MAX ? UINT64_MAX : (timeout_ms > UINT64_MAX / 1000ULL ? UINT64_MAX - 1 : timeout_ms * 1000ULL);
	return chiaki_stop_pipe_select_single_us(stop_pipe, fd, write, timeout_us);
}

CHIAKI_EXPORT ChiakiErrorCode chiaki_stop_pipe_select_single_us(ChiakiStopPipe *stop_pipe, chiaki_socket_t fd, bool write, uint64_t timeout_us)
{
#ifdef _WIN32
	WSAEVENT events[2];
	DWORD events_count = 1;
	events[0] = stop_pipe->event;

	if(!CHIAKI_SOCKET_IS_INVALID(fd))
	{
		events_count = 2;
		events[1] = WSACreateEvent();
		if(events[1] == WSA_INVALID_EVENT)
			return CHIAKI_ERR_UNKNOWN;
		WSAEventSelect(fd, events[1], write ? FD_WRITE : FD_READ);
	}

	DWORD wait_ms = WSA_INFINITE;
	if(timeout_us != UINT64_MAX)
	{
		uint64_t ms = timeout_us == 0 ? 0 : ((timeout_us + 999ULL) / 1000ULL);
		wait_ms = ms >= WSA_INFINITE ? (WSA_INFINITE - 1) : (DWORD)ms;
	}

	DWORD r = WSAWaitForMultipleEvents(events_count, events, FALSE, wait_ms, FALSE);

	if(events_count == 2)
		WSACloseEvent(events[1]);

	switch(r)
	{
		case WSA_WAIT_EVENT_0:
			return CHIAKI_ERR_CANCELED;
		case WSA_WAIT_EVENT_0+1:
			return CHIAKI_ERR_SUCCESS;
		case WSA_WAIT_TIMEOUT:
			return CHIAKI_ERR_TIMEOUT;
		default:
			return CHIAKI_ERR_UNKNOWN;
	}
#elif defined(__APPLE__)
	if(__atomic_load_n(&stop_pipe->stopped, __ATOMIC_ACQUIRE))
		return CHIAKI_ERR_CANCELED;

	struct kevent change;
	int nchanges = 0;
	int16_t filter = write ? EVFILT_WRITE : EVFILT_READ;
	if(!CHIAKI_SOCKET_IS_INVALID(fd))
	{
		EV_SET(&change, (uintptr_t)fd, filter, EV_ADD | EV_ONESHOT, 0, 0, NULL);
		nchanges = 1;
	}

	uint64_t deadline_us = UINT64_MAX;
	if(timeout_us != UINT64_MAX && timeout_us > 0)
		deadline_us = chiaki_time_now_monotonic_us() + timeout_us;

	struct kevent events[2];
	int r;
	while(true)
	{
		struct timespec ts;
		const struct timespec *timeout = NULL;
		if(timeout_us != UINT64_MAX)
		{
			uint64_t rem_us = 0;
			if(timeout_us > 0)
			{
				uint64_t now_us = chiaki_time_now_monotonic_us();
				if(now_us < deadline_us)
					rem_us = deadline_us - now_us;
			}
			ts.tv_sec = (time_t)(rem_us / 1000000ULL);
			ts.tv_nsec = (long)((rem_us % 1000000ULL) * 1000ULL);
			timeout = &ts;
		}

		r = kevent(stop_pipe->kq, nchanges ? &change : NULL, nchanges, events, 2, timeout);
		if(r < 0 && errno == EINTR)
		{
			if(__atomic_load_n(&stop_pipe->stopped, __ATOMIC_ACQUIRE))
			{
				if(nchanges)
				{
					struct kevent del;
					EV_SET(&del, (uintptr_t)fd, filter, EV_DELETE, 0, 0, NULL);
					kevent(stop_pipe->kq, &del, 1, NULL, 0, NULL);
				}
				return CHIAKI_ERR_CANCELED;
			}
			continue;
		}
		break;
	}

	if(r < 0)
		return CHIAKI_ERR_UNKNOWN;

	bool fd_ready = false;
	bool fd_error = false;
	for(int i = 0; i < r; i++)
	{
		if(events[i].flags & EV_ERROR)
		{
			fd_error = true;
			continue;
		}
		if(!CHIAKI_SOCKET_IS_INVALID(fd) && events[i].ident == (uintptr_t)fd && events[i].filter == filter)
			fd_ready = true;
	}

	if(nchanges && !fd_ready && !fd_error)
	{
		struct kevent del;
		EV_SET(&del, (uintptr_t)fd, filter, EV_DELETE, 0, 0, NULL);
		kevent(stop_pipe->kq, &del, 1, NULL, 0, NULL);
	}

	if(__atomic_load_n(&stop_pipe->stopped, __ATOMIC_ACQUIRE))
		return CHIAKI_ERR_CANCELED;

	if(fd_error)
		return CHIAKI_ERR_UNKNOWN;

	if(fd_ready)
		return CHIAKI_ERR_SUCCESS;

	return CHIAKI_ERR_TIMEOUT;
#else
	fd_set rfds;
	FD_ZERO(&rfds);
#if defined(__SWITCH__)
	// push udp local socket as fd
	int stop_fd = stop_pipe->fd;
#else
	int stop_fd = stop_pipe->fds[0];
#endif
	FD_SET(stop_fd, &rfds);
	int nfds = stop_fd;

	fd_set wfds;
	FD_ZERO(&wfds);
	if(!CHIAKI_SOCKET_IS_INVALID(fd))
	{
		FD_SET(fd, write ? &wfds : &rfds);
		if(fd > nfds)
			nfds = fd;
	}
	nfds++;

	struct timeval timeout_s;
	struct timeval *timeout = NULL;
	if(timeout_us != UINT64_MAX)
	{
		timeout_s.tv_sec = timeout_us / 1000000ULL;
		timeout_s.tv_usec = timeout_us % 1000000ULL;
		timeout = &timeout_s;
	}

	int r;
	do
	{
		r = select(nfds, &rfds, write ? &wfds : NULL, NULL, timeout);
	} while(r < 0 && errno == EINTR);

	if(r < 0)
		return CHIAKI_ERR_UNKNOWN;

	if(FD_ISSET(stop_fd, &rfds))
		return CHIAKI_ERR_CANCELED;

	if(!CHIAKI_SOCKET_IS_INVALID(fd) && FD_ISSET(fd, write ? &wfds : &rfds))
		return CHIAKI_ERR_SUCCESS;

	return CHIAKI_ERR_TIMEOUT;
#endif
}

CHIAKI_EXPORT ChiakiErrorCode chiaki_stop_pipe_connect(ChiakiStopPipe *stop_pipe, chiaki_socket_t fd, struct sockaddr *addr, size_t addrlen, uint64_t timeout_ms)
{
	int r = connect(fd, addr, (socklen_t)addrlen);
	if(r >= 0)
		return CHIAKI_ERR_SUCCESS;

	if(CHIAKI_SOCKET_EINPROGRESS)
	{
		ChiakiErrorCode err = chiaki_stop_pipe_select_single(stop_pipe, fd, true, timeout_ms);
		if(err != CHIAKI_ERR_SUCCESS)
			return err;
	}
	else
	{
#ifdef _WIN32
		int err = WSAGetLastError();
		if(err == WSAECONNREFUSED)
			return CHIAKI_ERR_CONNECTION_REFUSED;
		else
			return CHIAKI_ERR_NETWORK;
#else
		if(errno == ECONNREFUSED)
			return CHIAKI_ERR_CONNECTION_REFUSED;
		else
			return CHIAKI_ERR_NETWORK;
#endif
	}

	struct sockaddr_storage peer;
	socklen_t peerlen = sizeof(peer);
	if(getpeername(fd, (struct sockaddr *)(&peer), &peerlen) == 0)
		return CHIAKI_ERR_SUCCESS;

#ifdef _WIN32
	int err = WSAGetLastError();
	if(err != WSAENOTCONN)
		return CHIAKI_ERR_UNKNOWN;
#else
	if(errno != ENOTCONN)
		return CHIAKI_ERR_UNKNOWN;
#endif

#ifdef _WIN32
	int sockerr;
	socklen_t sockerr_sz = sizeof(sockerr);
	if(getsockopt(fd, SOL_SOCKET, SO_ERROR, (char*)(&sockerr), &sockerr_sz) < 0)
		return CHIAKI_ERR_UNKNOWN;
#else
	int sockerr;
	socklen_t sockerr_sz = sizeof(sockerr);
	if(getsockopt(fd, SOL_SOCKET, SO_ERROR, &sockerr, &sockerr_sz) < 0)
		return CHIAKI_ERR_UNKNOWN;
#endif

#ifdef _WIN32
	switch(sockerr)
	{
		case WSAETIMEDOUT:
			return CHIAKI_ERR_TIMEOUT;
		case WSAECONNREFUSED:
			return CHIAKI_ERR_CONNECTION_REFUSED;
		case WSAEHOSTDOWN:
			return CHIAKI_ERR_HOST_DOWN;
		case WSAEHOSTUNREACH:
			return CHIAKI_ERR_HOST_UNREACH;
		default:
			return CHIAKI_ERR_UNKNOWN;
	}
#else
	switch(sockerr)
	{
		case ETIMEDOUT:
			return CHIAKI_ERR_TIMEOUT;
		case ECONNREFUSED:
			return CHIAKI_ERR_CONNECTION_REFUSED;
		case EHOSTDOWN:
			return CHIAKI_ERR_HOST_DOWN;
		case EHOSTUNREACH:
			return CHIAKI_ERR_HOST_UNREACH;
		default:
			return CHIAKI_ERR_UNKNOWN;
	}
#endif
}

CHIAKI_EXPORT ChiakiErrorCode chiaki_stop_pipe_reset(ChiakiStopPipe *stop_pipe)
{
#ifdef _WIN32
	BOOL r = WSAResetEvent(stop_pipe->event);
	return r ? CHIAKI_ERR_SUCCESS : CHIAKI_ERR_UNKNOWN;
#elif defined(__APPLE__)
	if(__atomic_exchange_n(&stop_pipe->stopped, 0, __ATOMIC_ACQ_REL) != 0)
	{
		struct timespec zero = { 0, 0 };
		struct kevent ev;
		kevent(stop_pipe->kq, NULL, 0, &ev, 1, &zero);
	}
	return CHIAKI_ERR_SUCCESS;
#elif defined(__SWITCH__)
	//FIXME
	uint8_t v;
	int r;
	while((r = read(stop_pipe->fd, &v, sizeof(v))) > 0);
	return r < 0 ? CHIAKI_ERR_UNKNOWN : CHIAKI_ERR_SUCCESS;
#else
	uint8_t v;
	int r;
	while((r = read(stop_pipe->fds[0], &v, sizeof(v))) > 0);
	return r < 0 ? CHIAKI_ERR_UNKNOWN : CHIAKI_ERR_SUCCESS;
#endif
}
