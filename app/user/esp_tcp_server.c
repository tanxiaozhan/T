/*
at lwipopts.h set 1 enable
#define LWIP_SOCKET 1
#define LWIP_NETCONN 1
#define LWIP_COMPAT_SOCKETS 1
*/


//#include "user_interface.h"
#include "c_types.h"
#include "lwip/ip_addr.h"
#include "espconn.h"
#include "esp_libc.h"   //#include "mem.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_tcp_server.h"
//#include "osapi.h"

static uint16_t server_timeover = 120;
static struct espconn *pTcpServer;
#define at_linkMax 2
static at_linkConType pLink[at_linkMax];
static at_mdStateType mdState;
static at_linkNum = 0;
static BOOL at_ipMux = FALSE;

/**
  * @brief  Client received callback function.
  * @param  arg: contain the ip link information
  * @param  pdata: received data
  * @param  len: the lenght of received data
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_tcpclient_recv(void *arg, char *pdata, unsigned short len)
{
  struct espconn *pespconn = (struct espconn *)arg;
  at_linkConType *linkTemp = (at_linkConType *)pespconn->reserve;
  char temp[50];
  BOOL IPMODE=0;  //1--Í¸´«
  os_printf("recv\r\n");
  if(at_ipMux)
  {
    sprintf(temp, "\r\n+IPD,%d,%d:",
               linkTemp->linkId, len);
    printf(temp); //uart0_sendStr(temp);
    //uart0_tx_buffer(pdata, len);
  }
  else if(IPMODE == FALSE)
  {
    sprintf(temp, "\r\n+IPD,%d:", len);
    printf(temp); //uart0_sendStr(temp);
    //uart0_tx_buffer(pdata, len);
  }
  else
  {
  	//uart0_tx_buffer(pdata, len);
  }

}


/**
  * @brief  Tcp server connect repeat callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_tcpserver_recon_cb(void *arg, sint8 errType)
{

	return;

	struct espconn *pespconn = (struct espconn *)arg;
  at_linkConType *linkTemp = (at_linkConType *)pespconn->reserve;
  char temp[16];

  os_printf("S conect C: %p\r\n", arg);

  if(pespconn == NULL)
  {
    return;
  }

  linkTemp->linkEn = false;
  linkTemp->pCon = NULL;
  printf("con EN? %d\r\n", linkTemp->linkId);
  at_linkNum--;
  if (at_linkNum == 0)
  {
    mdState = m_unlink; //////////////////////
  }

  if(at_ipMux)
  {
    sprintf(temp, "%d,CONNECT\r\n", linkTemp->linkId);
    printf(temp);  //uart0_sendStr(temp);
  }
  else
  {
    printf(temp); //uart0_sendStr("CONNECT\r\n");
  }

    printf("Unlink\r\n");  //uart0_sendStr("Unlink\r\n");

  if(linkTemp->teToff == TRUE)
  {
    linkTemp->teToff = FALSE;
//    specialAtState = true;
//    at_state = at_statIdle;
  }
  ETS_UART_INTR_ENABLE();
}


/**
  * @brief  Client send over callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_tcpclient_sent_cb(void *arg)
{
//	os_free(at_dataLine);
//  os_printf("send_cb\r\n");
return;

  uart0_sendStr("\r\nSEND OK!\r\n");
}



/**
  * @brief  Tcp server listend callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
LOCAL void ICACHE_FLASH_ATTR
at_tcpserver_listen(void *arg)
{

	return;



	struct espconn *pespconn = (struct espconn *)arg;
  uint8_t i;
  char temp[16];

  os_printf("get tcpClient:\r\n");
  for(i=0;i<at_linkMax;i++)
  {
    if(pLink[i].linkEn == FALSE)
    {
      pLink[i].linkEn = TRUE;
      break;
    }
  }
  if(i>=5)
  {
    return;
  }
  pLink[i].teToff = FALSE;
  pLink[i].linkId = i;
  pLink[i].teType = teServer;
  pLink[i].repeaTime = 0;
  pLink[i].pCon = pespconn;
  mdState = m_linked;
  at_linkNum++;
  pespconn->reserve = &pLink[i];
  espconn_regist_recvcb(pespconn, at_tcpclient_recv);
  espconn_regist_reconcb(pespconn, at_tcpserver_recon_cb);
  //espconn_regist_disconcb(pespconn, at_tcpserver_discon_cb);
  espconn_regist_sentcb(pespconn, at_tcpclient_sent_cb);///////
  if(at_ipMux)
  {
    os_sprintf(temp, "%d,CONNECT\r\n", i);
    printf(temp);  //uart0_sendStr(temp);
  }
  else
  {
    printf(temp);  //uart0_sendStr("CONNECT\r\n");
  }
//  uart0_sendStr("Link\r\n");
}


void esp_tcp_server( void ){
	pTcpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
	    if (pTcpServer == NULL)
	    {
	      os_printf("TcpServer Failure\r\n");
	    	//uart0_sendStr("TcpServer Failure\r\n");
	      return;
	    }
	    pTcpServer->type = ESPCONN_TCP;
	    pTcpServer->state = ESPCONN_NONE;
	    pTcpServer->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
	    pTcpServer->proto.tcp->local_port = SERVER_PORT;
	    espconn_regist_connectcb(pTcpServer, at_tcpserver_listen);
	    espconn_accept(pTcpServer);
	    espconn_regist_time(pTcpServer, server_timeover, 0);
}
