// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#include <chiaki/sock.h>
#include <fcntl.h>

#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

CHIAKI_EXPORT ChiakiErrorCode chiaki_socket_set_nonblock(chiaki_socket_t sock, bool nonblock)
{
#ifdef _WIN32
	u_long nbio = nonblock ? 1 : 0;
	if(ioctlsocket(sock, FIONBIO, &nbio) != NO_ERROR)
		return CHIAKI_ERR_UNKNOWN;
#else
	int flags = fcntl(sock, F_GETFL, 0);
	if(flags == -1)
		return CHIAKI_ERR_UNKNOWN;
	flags = nonblock ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
	if(fcntl(sock, F_SETFL, flags) == -1)
		return CHIAKI_ERR_UNKNOWN;
#endif
	return CHIAKI_ERR_SUCCESS;
}

CHIAKI_EXPORT void chiaki_socket_set_low_latency(chiaki_socket_t sock, bool is_tcp, bool interactive_av)
{
	if(CHIAKI_SOCKET_IS_INVALID(sock))
		return;

	if(is_tcp)
	{
		int nodelay = 1;
		setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&nodelay, sizeof(nodelay));
	}

#ifdef __APPLE__
	int nosigpipe = 1;
	setsockopt(sock, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof(nosigpipe));

#ifdef SO_NET_SERVICE_TYPE
	/*
	 * Tag outgoing packets so XNU's interface scheduler (and Wi-Fi WMM) queues them
	 * ahead of bulk traffic: interactive video for stream/feedback UDP, signaling for
	 * session/ctrl TCP. This only marks packets; best effort, failure is harmless.
	 */
	int svc_type = interactive_av ? NET_SERVICE_TYPE_RV : NET_SERVICE_TYPE_SIG;
	setsockopt(sock, SOL_SOCKET, SO_NET_SERVICE_TYPE, &svc_type, sizeof(svc_type));
#endif

	if(!is_tcp && interactive_av)
	{
		int sndbuf = 0;
		socklen_t optlen = sizeof(sndbuf);
		const int min_sndbuf = 512 * 1024;
		if(getsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, &optlen) != 0 || sndbuf < min_sndbuf)
		{
			sndbuf = min_sndbuf;
			setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
		}
	}
#else
	(void)interactive_av;
#endif
}
