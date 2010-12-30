#ifndef _ICC_H
#define _ICC_H

/* arch specific */
#define sm_atomic_read(v) bfin_read16(v)
#define sm_atomic_write(v, i) bfin_write16(v, i)

typedef unsigned char sm_unit_t;
typedef unsigned short sm_uint16_t;
typedef unsigned long sm_uint32_t;
typedef sm_uint32_t sm_address_t;
typedef sm_uint16_t sm_atomic_t;
#define MSGQ_START_ADDR	0xFEB18000
#define MSG_BUF_ADDR	0xFEB1F000

#define SM_DATA_ATTR __atrribute__((section .l2.atomic))
#define SM_ATOMIC(type, name) SM_DATA_ATTR \
		__typeof__(type) name

/* sm protocol */
/* compose type enumeration value from protocol & subtype */
#define SM_MSG_TYPE(protocol, subtype) (((protocol)<<24)|(subtype))

/* extract subtype from type enumeration value */
#define SM_MSG_SUBTYPE(type) ((type)&0xffffff)

/* extract protocol from type enumeration value */
#define SM_MSG_PROTOCOL(type) (((type)>>24)&0xff)

enum {
	SP_GENERAL = 0,
	SP_CORE_CONTROL,
	SP_RES_MANAGER,
	SP_PACKET,
	SP_SESSION_PACKET,
	SP_MAX,
};


#define SM_UNCONNECT 0
#define SM_CONNECT 0x1

#define SM_BAD_ENDPOINT SM_MSG_TYPE(0, 0)
#define SM_BAD_MSG SM_MSG_TYPE(0, 1)

#define SM_CORE_START		SM_MSG_TYPE(SP_CORE_CONTROL, 0)
#define SM_CORE_STARTED		SM_MSG_TYPE(SP_CORE_CONTROL, 1)
#define SM_CORE_STOP		SM_MSG_TYPE(SP_CORE_CONTROL, 2)
#define SM_CORE_STOPPED		SM_MSG_TYPE(SP_CORE_CONTROL, 3)
#define SM_CORE_RESET		SM_MSG_TYPE(SP_CORE_CONTROL, 4)
#define SM_CORE_RESETED		SM_MSG_TYPE(SP_CORE_CONTROL, 5)

#define SM_RES_MGR_REQUEST	SM_MSG_TYPE(SP_RES_MANAGER, 0)
#define SM_RES_MGR_REQUEST_OK	SM_MSG_TYPE(SP_RES_MANAGER, 1)
#define SM_RES_MGR_REQUEST_FAIL	SM_MSG_TYPE(SP_RES_MANAGER, 2)
#define SM_RES_MGR_FREE		SM_MSG_TYPE(SP_RES_MANAGER, 3)
#define SM_RES_MGR_FREE_DONE	SM_MSG_TYPE(SP_RES_MANAGER, 4)
#define SM_RES_MGR_EXPIRE	SM_MSG_TYPE(SP_RES_MANAGER, 5)
#define SM_RES_MGR_EXPIRE_DONE	SM_MSG_TYPE(SP_RES_MANAGER, 6)
#define SM_RES_MGR_LIST		SM_MSG_TYPE(SP_RES_MANAGER, 7)
#define SM_RES_MGR_LIST_OK	SM_MSG_TYPE(SP_RES_MANAGER, 8)
#define SM_RES_MGR_LIST_DONE	SM_MSG_TYPE(SP_RES_MANAGER, 9)

#define SM_PACKET_READY		SM_MSG_TYPE(SP_PACKET, 0)
#define SM_PACKET_CONSUMED	SM_MSG_TYPE(SP_PACKET, 1)
#define SM_PACKET_ERROR		SM_MSG_TYPE(SP_PACKET, 2)
#define SM_PACKET_ERROR_ACK	SM_MSG_TYPE(SP_PACKET, 3)

#define SM_SESSION_PACKET_READY		SM_MSG_TYPE(SP_SESSION_PACKET, 0)
#define SM_SESSION_PACKET_COMSUMED	SM_MSG_TYPE(SP_SESSION_PACKET, 1)
#define SM_SESSION_PACKET_ERROR		SM_MSG_TYPE(SP_SESSION_PACKET, 2)
#define SM_SESSION_PACKET_ERROR_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 3)
#define SM_SESSION_PACKET_CONNECT	SM_MSG_TYPE(SP_SESSION_PACKET, 4)
#define SM_SESSION_PACKET_CONNECT_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 5)
#define SM_SESSION_PACKET_CONNECT_DONE	SM_MSG_TYPE(SP_SESSION_PACKET, 6)
#define SM_SESSION_PACKET_ACTIVE	SM_MSG_TYPE(SP_SESSION_PACKET, 7)
#define SM_SESSION_PACKET_ACTIVE_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 8)
#define SM_SESSION_PACKET_CLOSE		SM_MSG_TYPE(SP_SESSION_PACKET, 9)
#define SM_SESSION_PACKET_CLOSE_ACK	SM_MSG_TYPE(SP_SESSION_PACKET, 10)


#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/mutex.h>

struct sm_message {
	struct list_head next;
	sm_uint32_t dst;
	sm_uint32_t src;
	sm_uint32_t dst_ep;
	sm_uint32_t src_ep;
	sm_uint32_t type;
	sm_uint32_t length;
	sm_uint32_t flags;
	sm_address_t payload;
};

/* A magic number - stress test shows this is safe for common cases */
#define SM_MSGQ_LEN 8

/* Simple FIFO buffer */
struct sm_message_queue {
	struct sm_message messages[SM_MSGQ_LEN];
	sm_atomic_t sent;
	sm_atomic_t received; /* head of the queue */
};

struct sm_session {
	struct list_head messages; /*rx queue sm message*/
	sm_uint32_t	local_ep;
	sm_uint32_t	remote_ep; /*remote ep*/
	sm_uint32_t	type;
	pid_t		pid;
	sm_uint32_t	flags;
	int (*handle)(struct sm_message *msg, struct sm_session *session);
	struct sm_proto *proto_ops;
	wait_queue_head_t rx_wait;
};

#define MAX_ENDPOINTS 32
struct sm_session_table {
	struct sm_session sessions[MAX_ENDPOINTS];
	sm_uint32_t	nfree;
	sm_uint32_t	bits[(MAX_ENDPOINTS - 1) / BITS_PER_LONG + 1];
	struct list_head next_table;
	struct mutex lock;
};

struct sm_proto {
	int (*sendmsg)(struct sm_message *msg, struct sm_session *session);
	int (*recvmsg)(struct sm_message *msg, struct sm_session *session);
	int (*shutdown)(struct sm_session *session);
	int (*error)(struct sm_message *msg, struct sm_session *session);
};

struct sm_icc_desc {
	struct sm_message_queue *icc_queue;
	struct sm_session_table *sessions_table;
	struct task_struct *iccq_thread;
};

#endif

#define CMD_COREB_START		_IO('m', 0)
#define CMD_COREB_STOP		_IO('m', 1)
#define CMD_COREB_RESET		_IO('m', 2)
#define CMD_SM_SEND		_IO('m', 3)
#define CMD_SM_CREATE		_IO('m', 4)
#define CMD_SM_CONNECT		_IO('m', 5)
#define CMD_SM_RECV		_IO('m', 6)
#define CMD_SM_SHUTDOWN		_IO('m', 7)

struct sm_user_param {
	sm_uint32_t session_idx;
	sm_uint32_t local_ep;
	sm_uint32_t remote_ep;
	sm_uint32_t type;
	sm_uint32_t dst_cpu;
	sm_uint32_t src_cpu;
	sm_uint32_t buf_len;
	void *buf;
};

#endif
