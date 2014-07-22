
/* Sample UDP server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
//#include <iostream>
#include <inttypes.h>

//using namespace std;

int main(int argc, char**argv)
{
   int sockfd_serv, sockfd_cli,n;
   struct sockaddr_in servaddr, cliaddr;
   socklen_t len;
   char mesg[1000];
   uint8_t mcp_angle;
   uint8_t pip_torque;

   sockfd_serv=socket(AF_INET,SOCK_DGRAM,0);
   if (sockfd_serv < 0)
     error("ERROR opening server socket");

   bzero(&servaddr,sizeof(servaddr));
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
   servaddr.sin_port=htons(52343);
   if ((bind(sockfd_serv,(struct sockaddr *)&servaddr,sizeof(servaddr))) < 0)
   {
       perror("server bind failed");
       return 0;
   }


   sockfd_cli=socket(AF_INET,SOCK_DGRAM,0);
   if (sockfd_cli < 0)
     error("ERROR opening client socket");
   bzero(&cliaddr,sizeof(cliaddr));
   cliaddr.sin_family = AF_INET;
   cliaddr.sin_addr.s_addr=htonl(INADDR_ANY);
   cliaddr.sin_port=htons(52344);
   if ((bind(sockfd_cli,(struct sockaddr *)&cliaddr,sizeof(cliaddr))) < 0)
   {
       perror("client bind failed");
       return 0;
   }

   getchar();
   for (;;)
   {
      len = sizeof(servaddr);
//      n = recvfrom(sockfd,mesg,1000,0,(struct sockaddr *)&cliaddr,&len);
      n = recvfrom(sockfd_serv,&mcp_angle,1,0,(struct sockaddr *)&servaddr,&len);
      pip_torque = mcp_angle*0.5;
      sendto(sockfd_cli,&pip_torque,1,0,(struct sockaddr *)&cliaddr,sizeof(cliaddr));
      printf("-------------------------------------------------------\n");
      printf("Received the following:\n");
      printf("%" PRIu8, mcp_angle);     
      printf("-------------------------------------------------------\n");
      printf("-------------------------------------------------------\n");
      printf("Sent the following:\n");
      printf("%" PRIu8, pip_torque);
      printf("-------------------------------------------------------\n");
   }
}

