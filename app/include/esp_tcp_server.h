#define SERVER_PORT 80
#define MAX_CONN 2

typedef enum
{
  teClient,
  teServer
}teType;

typedef struct
{
	BOOL linkEn;
  BOOL teToff;
	uint8_t linkId;
	teType teType;
	uint8_t repeaTime;
	uint8_t changType;
	uint8 remoteIp[4];
	int32_t remotePort;
	struct espconn *pCon;
}at_linkConType;


typedef enum{
  m_init,
  m_wact,
  m_gotip,
  m_linked,
  m_unlink,
  m_wdact
}at_mdStateType;


void esp_tcp_server( void );
//void create_tcp_server_task(void);
