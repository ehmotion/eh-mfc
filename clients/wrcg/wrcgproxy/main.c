/*
	Simple UDP Server
*/

#include<stdio.h>
#include<winsock2.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 512	//Max length of buffer
#define PORT 20776	//The port on which to listen for incoming data

int main(int argc, char*argv[])
{
	SOCKET s, r;
	struct sockaddr_in server, si_other, proxy;
	int slen , recv_len, send_len;
	long srvaddr = argc>2?inet_addr(argv[2]):inet_addr("127.0.0.1");
	long prxaddr = argc>1?inet_addr(argv[1]):inet_addr("192.168.68.109");
	char buf[BUFLEN];
	WSADATA wsa;

	slen = sizeof(si_other) ;

	//Initialise winsock
	printf("\ngot arguments: %d", argc);
	for(int i = 0; i<argc;i++)
        printf("%s ", argv[i]);
	printf("\ninitialising Winsock...");
	if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
	{
		printf("Failed. Error Code : %d",WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("done.\n");

	//Create a socket
	if((r = socket(AF_INET , SOCK_DGRAM , 0 )) == INVALID_SOCKET)
	{
		printf("recv could not create socket : %d" , WSAGetLastError());
	}
	printf("recv socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = srvaddr;//inet_addr("127.0.0.1");//INADDR_ANY;
	server.sin_port = htons( PORT );

	//Bind
	if( bind(r ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
	{
		printf("recv bind failed with error code : %d" , WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	puts("recv bind done");

	//Create a socket
	if((s = socket(AF_INET , SOCK_DGRAM , 0 )) == INVALID_SOCKET)
	{
		printf("snd could not create socket : %d" , WSAGetLastError());
	}
	printf("snd socket created.\n");

	//Prepare the sockaddr_in structure
	proxy.sin_family = AF_INET;
	proxy.sin_addr.s_addr = prxaddr;//inet_addr("192.168.68.109");//INADDR_ANY;
	proxy.sin_port = htons( 20777 );
    /*
	//Bind
	if( bind(s ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
	{
		printf("snd bind failed with error code : %d" , WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	puts("Bind done");
	*/
    printf("listen on %s:%d, ", inet_ntoa(server.sin_addr), ntohs(server.sin_port));
    printf("forward to %s:%d, ", inet_ntoa(proxy.sin_addr), ntohs(proxy.sin_port));

	//keep listening for data
	while(1)
	{
		printf("waiting for data...");
		fflush(stdout);

		//clear the buffer by filling null, it might have previously received data
		memset(buf,'\0', BUFLEN);

		//try to receive some data, this is a blocking call
		if ((recv_len = recvfrom(r, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
		{
			printf("recvfrom() failed with error code : %d" , WSAGetLastError());
			exit(EXIT_FAILURE);
		}

		//print details of the client/peer and the data received
		printf("received %dB packet from %s:%d, ", recv_len, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));

		//now reply the client with the same data
		send_len = sendto(s, buf, recv_len, 0, (struct sockaddr*) &proxy, slen);
		if (send_len == SOCKET_ERROR)
		{
			printf("sendto() failed with error code : %d" , WSAGetLastError());
			exit(EXIT_FAILURE);
		} else {
            printf("sent %dB packet to %s:%d\n", send_len, inet_ntoa(proxy.sin_addr), ntohs(proxy.sin_port));
		}
	}

	closesocket(s);
	closesocket(r);

	WSACleanup();

	return 0;
}
