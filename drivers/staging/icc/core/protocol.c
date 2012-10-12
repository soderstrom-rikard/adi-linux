/* Multicore communication on dual-core Blackfin processor
 * Copyright 2004-2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */


#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/bitmap.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <icc.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <asm/cacheflush.h>
#include <asm/icc.h>
#include <asm/dma.h>

#define DRIVER_NAME "icc"

static struct bfin_icc {
	struct miscdevice mdev;
	int peer_count;
	wait_queue_head_t icc_rx_wait;
	struct sm_session_table *sessions_table;
	struct sm_icc_desc *icc_info;
	struct sm_proto *sm_protos[SP_MAX];
	struct proc_dir_entry *icc_dump;
	char name[20];
} *bfin_icc;


void icc_send_ipi_cpu(unsigned int cpu, int irq)
{
	platform_send_ipi_cpu(cpu, irq);
}

void icc_clear_ipi_cpu(unsigned int cpu, int irq)
{
	platform_clear_ipi(cpu, irq);
}

static void wakeup_icc_thread(struct sm_icc_desc *icc_info)
{
	if (icc_info->iccq_thread)
		wake_up_process(icc_info->iccq_thread);
	wake_up(&bfin_icc->icc_rx_wait);
}

static struct sm_message_queue *icc_get_inqueue(struct sm_msg *msg)
{
	struct sm_icc_desc *icc_info = bfin_icc->icc_info;
	struct sm_message_queue *queue;
	int i;
	if (!msg)
		return NULL;
	for (i = 0; i < bfin_icc->peer_count; i++, icc_info++) {
		queue = icc_info->icc_high_queue;
		if ((uint32_t)msg > (uint32_t)(queue + 4)) {
			sm_debug("wrong msg addr %p\n", msg);
			return NULL;
		}
		while ((uint32_t)msg > (uint32_t)queue)
			queue++;
		return queue - 1;
	}

	return NULL;
}

static struct sm_icc_desc *get_icc_peer(struct sm_msg *msg)
{
	struct sm_icc_desc *icc_info = bfin_icc->icc_info;
	int i;
	uint32_t msg_addr = (uint32_t)msg;
	BUG_ON(!msg);
	for (i = 0; i < bfin_icc->peer_count; i++) {
		if (((uint32_t)icc_info[i].icc_high_queue < msg_addr) &&
		(msg_addr < (uint32_t)icc_info[i].icc_high_queue + MSGQ_SIZE))
			break;
	}

	if (i == bfin_icc->peer_count)
		return NULL;
	return &icc_info[i];
}

static int sm_get_icc_queue_attribute(uint32_t cpu, uint32_t type, uint32_t *attribute)
{
	if (cpu > bfin_icc->peer_count)
		return -EINVAL;
	if (type >= ICC_QUEUE_ATTR_MAX)
		return -EINVAL;
	if (!attribute)
		return -EINVAL;
	*attribute = *(bfin_icc->icc_info[cpu - 1].icc_queue_attribute + type);
	return 0;
}

static int sm_set_icc_queue_attribute(uint32_t cpu, uint32_t type, uint32_t attribute)
{
	if (cpu > bfin_icc->peer_count)
		return -EINVAL;
	if (type >= ICC_QUEUE_ATTR_MAX)
		return -EINVAL;
	*(bfin_icc->icc_info[cpu - 1].icc_queue_attribute + type) = attribute;
	return 0;
}

static struct sm_message_queue *sm_find_queue(struct sm_message *message, struct sm_session *session)
{
	uint16_t dst = message->dst;
	struct sm_icc_desc *icc_info = bfin_icc->icc_info;
	int i;
	if (!message)
		return NULL;
	for (i = 0; i < bfin_icc->peer_count; i++) {
		if (icc_info[i].peer_cpu == dst)
			break;
	}
	if (i == bfin_icc->peer_count)
		return NULL;
	message->icc_info = &icc_info[i];
	if (!session)
		return icc_info[i].icc_high_queue;
	else {
		if (session->queue_priority)
			return icc_info[i].icc_queue;
		else
			return icc_info[i].icc_high_queue;
	}
}

static int init_sm_session_table(struct bfin_icc *icc)
{
	icc->sessions_table = kzalloc(sizeof(struct sm_session_table), GFP_KERNEL);

	if (!icc->sessions_table)
		return -ENOMEM;
	mutex_init(&icc->sessions_table->lock);
	init_waitqueue_head(&icc->sessions_table->query_wait);
	INIT_LIST_HEAD(&icc->sessions_table->query_message);
	icc->sessions_table->nfree = MAX_ENDPOINTS;
	return 0;
}

static int sm_message_enqueue(struct sm_message_queue *icc_queue, struct sm_msg *msg)
{
	/* icc_queue[1] is the queue to send message */
	struct sm_message_queue *outqueue = icc_queue + 1;
	uint16_t sent = sm_atomic_read(&outqueue->sent);
	uint16_t received = sm_atomic_read(&outqueue->received);
	uint16_t pending = sent - received;

	if (pending < 0)
		pending += USHRT_MAX;

	if (pending >= (SM_MSGQ_LEN - 1)) {
		sm_debug("over run\n");
		return -EAGAIN;
	}
	memcpy(&outqueue->messages[(sent%SM_MSGQ_LEN)], msg,
		sizeof(struct sm_msg));
	sent++;
	sm_atomic_write(&outqueue->sent, sent);
	return 0;
}

static int sm_message_dequeue(struct sm_msg *msg)
{
	/* icc_queue[0] is the queue to receive message */
	struct sm_message_queue *inqueue = icc_get_inqueue(msg);
	uint16_t received;

	memset(msg, 0, sizeof(struct sm_msg));
	received = sm_atomic_read(&inqueue->received);
	received++;
	sm_atomic_write(&inqueue->received, received);
	return 0;
}

static uint32_t sm_alloc_session(struct sm_session_table *table)
{
	unsigned long index;
	sm_debug("table bits1 %08x\n", table->bits[0]);
	index = find_next_zero_bit((unsigned long *)table->bits, BITS_PER_LONG, 0);
	if (index >= BITS_PER_LONG)
		return -EAGAIN;
	sm_debug("table index %ld\n", index);
	bitmap_set((unsigned long *)table->bits, index, 1);

	sm_debug("table bits2 %08x\n", table->bits[0]);
	table->nfree--;
	return index;
}

static int sm_free_session(uint32_t slot, struct sm_session_table *table)
{
	if (test_bit((int)slot, (unsigned long *)table->bits)) {
		memset(&table->sessions[slot], 0, sizeof(struct sm_session));
		__clear_bit((int)slot, (unsigned long *)table->bits);
		table->nfree++;
		return 0;
	}
	return -1;
}

static int
sm_find_session(uint32_t local_ep, uint32_t remote_ep,
			struct sm_session_table *table)
{
	unsigned long index;
	struct sm_session *session;
	sm_debug("%s bits %08x localep %d\n", __func__, table->bits[0], (int)local_ep);
	for_each_set_bit(index, (unsigned long *)table->bits, BITS_PER_LONG) {
		session = &table->sessions[index];
		sm_debug("index %ld ,local ep %d type %x\n", index, (int)session->local_ep, (uint32_t)session->type);
		if (session->local_ep == local_ep) {
			if (remote_ep && session->remote_ep != remote_ep)
				return -EINVAL;
			goto found_slot;
		}
	}
	return -EINVAL;
found_slot:
	return index;
}

static int sm_create_session(uint32_t src_ep, uint32_t type, uint32_t queue_priority)
{
	struct sm_session_table *table = bfin_icc->sessions_table;
	int index;
	struct sm_message *request;
	int ret = -EAGAIN;
	int pending = 0;

	mutex_lock(&table->lock);
	index = sm_find_session(src_ep, 0, table);
	if (index >= 0 && index < 32) {
		sm_debug("already bound index %d srcep %d\n", index, src_ep);
		ret = -EEXIST;
		goto fail_out;
	}
	if (type >= SP_MAX) {
		sm_debug("bad type %x\n", type);
		ret = -EINVAL;
		goto fail_out;
	}

	index = sm_alloc_session(table);
	if (index >= 0 && index < 32) {
		table->sessions[index].local_ep = src_ep;
		table->sessions[index].remote_ep = 0;
		table->sessions[index].pid = current->pid;
		table->sessions[index].flags = 0;
		table->sessions[index].n_uncompleted = 0;
		table->sessions[index].n_avail = 0;
		table->sessions[index].type = type;
		table->sessions[index].proto_ops = bfin_icc->sm_protos[type];
		table->sessions[index].queue_priority = queue_priority;
		INIT_LIST_HEAD(&table->sessions[index].rx_messages);
		INIT_LIST_HEAD(&table->sessions[index].tx_messages);
		init_waitqueue_head(&table->sessions[index].rx_wait);

		list_for_each_entry(request, &table->query_message, next) {
			if (request->msg.dst_ep == src_ep) {
				pending = 1;
				break;
			}
		}
		mutex_unlock(&table->lock);

		if (pending) {
			list_del(&request->next);
			sm_send_control_msg(&table->sessions[index], request->msg.src_ep,
				request->src, 0, 0, SM_QUERY_ACK_MSG);
			kfree(request);
		}

		sm_debug("create ep index %d srcep %d type %x\n", index, src_ep, type);
		sm_debug("session %p\n", &table->sessions[index]);
		return index;
	}
fail_out:
	mutex_unlock(&table->lock);
	return ret;
}

static struct sm_session *sm_index_to_session(uint32_t session_idx)
{
	struct sm_session *session;
	struct sm_session_table *table = bfin_icc->sessions_table;
	if (session_idx < 0 && session_idx >= MAX_SESSIONS)
		return NULL;
	if (!test_bit(session_idx, (unsigned long *)table->bits))
		return NULL;
	session = &table->sessions[session_idx];
	return session;
}

static uint32_t sm_session_to_index(struct sm_session *session)
{
	struct sm_session_table *table = bfin_icc->sessions_table;
	if ((session >= &table->sessions[0])
		&& (session < &table->sessions[MAX_SESSIONS])) {
		return (session - &table->sessions[0])/sizeof(struct sm_session);
	}
	return -EINVAL;
}

static int __iccqueue_getpending(struct sm_message_queue *inqueue)
{
	uint16_t sent = sm_atomic_read(&inqueue->sent);
	uint16_t received = sm_atomic_read(&inqueue->received);
	uint16_t pending;
	pending = sent - received;
	if (pending < 0)
		pending += USHRT_MAX;
	pending %= SM_MSGQ_LEN;
	return pending;
}

static int iccqueue_getpending(struct sm_icc_desc *icc_info)
{
	/* icc_queue[0] is the queue to receive message */
	uint16_t pending;

	pending = __iccqueue_getpending(icc_info->icc_high_queue);
	if (pending)
		return pending;
	return __iccqueue_getpending(icc_info->icc_queue);
}

static int sm_send_message_internal(struct sm_session *session,
					struct sm_message *message)
{
	struct sm_message_queue *icc_queue = sm_find_queue(message, session);
	struct sm_msg *msg = &message->msg;
	int ret = 0;
	struct sm_icc_desc *icc_info = message->icc_info;
	BUG_ON(!icc_info);
	if (!icc_queue)
		return -EINVAL;
	sm_debug("%s: dst %d %08x\n", __func__, icc_info->peer_cpu, (uint32_t)msg->type);
	ret = sm_message_enqueue(icc_queue, msg);
	if (!ret)
		icc_send_ipi_cpu(icc_info->peer_cpu, icc_info->notify);
	return ret;
}


int
sm_send_control_msg(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu, uint32_t payload,
			uint32_t len, uint32_t type)
{
	struct sm_message *message;
	int ret = 0;

	message = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;

	message->msg.type = type;
	if (session)
		message->msg.src_ep = session->local_ep;
	message->msg.dst_ep = remote_ep;
	message->msg.length = len;
	message->msg.payload = payload;
	message->dst = dst_cpu;
	message->src = blackfin_core_id();

retry:
	ret = sm_send_message_internal(session, message);
	if (ret) {
		interruptible_sleep_on(&message->icc_info->iccq_tx_wait);
		goto retry;
	}

	kfree(message);
	return ret;

}

int sm_send_packet_ack(struct sm_session *session, uint32_t remote_ep,
		uint32_t dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_PACKET_CONSUMED);
}

int
sm_send_session_packet_ack(struct sm_session *session, uint32_t remote_ep,
		uint32_t dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SESSION_PACKET_CONSUMED);
}

int sm_send_scalar_cmd(struct sm_session *session, uint32_t remote_ep,
		uint32_t dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SCALAR_READY_64);
}

int sm_send_scalar_ack(struct sm_session *session, uint32_t remote_ep,
		uint32_t dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SCALAR_CONSUMED);
}

int
sm_send_session_scalar_ack(struct sm_session *session, uint32_t remote_ep,
		uint32_t dst_cpu, uint32_t payload, uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SESSION_SCALAR_CONSUMED);
}

int sm_send_connect(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu, uint32_t type)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, type);
}

int sm_send_connect_ack(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CONNECT_ACK);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_SCALAR_CONNECT_ACK);
	else
		return -EINVAL;
}

int sm_send_connect_done(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CONNECT_DONE);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
				0, SM_SESSION_SCALAR_CONNECT_DONE);
	else
		return -EINVAL;
}

int sm_send_session_active(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_ACTIVE);
}

int sm_send_session_active_ack(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, SM_OPEN,
					0, SM_SESSION_PACKET_ACTIVE_ACK);
}

int sm_send_session_active_noack(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_ACTIVE_ACK);
}

int sm_send_close(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CLOSE);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_SCALAR_CLOSE);
	else
		return -EINVAL;
}

int sm_send_close_ack(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	if (session->type == SP_SESSION_PACKET)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CLOSE_ACK);
	else if (session->type == SP_SESSION_SCALAR)
		return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_SCALAR_CLOSE_ACK);
	else
		return -EINVAL;
}

int sm_send_error(struct sm_session *session, uint32_t remote_ep,
			uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_PACKET_ERROR);
}

static int
sm_send_scalar(uint32_t session_idx, uint32_t dst_ep, uint32_t dst_cpu,
		uint32_t scalar0, uint32_t scalar1, uint32_t type, int nonblock)
{
	struct sm_session *session;
	struct sm_message *m;
	int ret = -EAGAIN;
	if (session_idx < 0 || session_idx >= MAX_SESSIONS)
		return -EINVAL;
	session = sm_index_to_session(session_idx);
	m = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	if (session->type == SP_SESSION_SCALAR) {
		if (session->flags == SM_CONNECT)
			m->msg.dst_ep = session->remote_ep;
		else
			return -EINVAL;
	} else
		m->msg.dst_ep = dst_ep;

	m->msg.src_ep = session->local_ep;
	m->src = blackfin_core_id();
	m->dst = dst_cpu;
	m->msg.payload = scalar0;
	m->msg.length = scalar1;
	m->msg.type = type;

	if (session->proto_ops->sendmsg) {
		ret = session->proto_ops->sendmsg(m, session);
	} else {
		sm_debug("session type not supported\n");
		ret = 0;
	}
	if (ret)
		goto out;

	sm_debug("%s: scalar0 %x scalar1 %x type %x dst %d dstep %d src %d srcep %d\n", __func__, scalar0, scalar1, (uint32_t)m->msg.type, (int)m->dst, (int)m->msg.dst_ep, (int)m->src, (int)m->msg.src_ep);
retry:
	ret = sm_send_message_internal(session, m);
	if ((!nonblock) && (ret == -EAGAIN)) {
		interruptible_sleep_on(&m->icc_info->iccq_tx_wait);
		goto retry;
	} else {
		goto out;
	}
out:
	return ret;
}

static int
sm_send_packet(uint32_t session_idx, uint32_t dst_ep, uint32_t dst_cpu,
		void *buf, uint32_t len, int nonblock)
{
	struct sm_session *session;
	struct sm_message *m;
	void *payload_buf = NULL;
	int ret = -EAGAIN;
	uint32_t queue_status;
	if (session_idx < 0 || session_idx >= MAX_SESSIONS)
		return -EINVAL;
	session = sm_index_to_session(session_idx);
	sm_debug("%s: %u %p\n", __func__, session_idx, session);
	m = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	if (session->type == SP_SESSION_PACKET) {
		if (session->flags == SM_CONNECT)
			m->msg.dst_ep = session->remote_ep;
		else
			return -EINVAL;
	} else
		m->msg.dst_ep = dst_ep;

	m->msg.src_ep = session->local_ep;
	m->src = blackfin_core_id();
	m->dst = dst_cpu;
	m->msg.length = len;
	m->msg.type = SM_MSG_TYPE(session->type, 0);

	if (m->msg.length) {
		payload_buf = kzalloc(m->msg.length, GFP_KERNEL);
		if (!payload_buf) {
			ret = -ENOMEM;
			goto fail1;
		}
		sm_debug("alloc buffer %x\n", (uint32_t)payload_buf);

		m->msg.payload = (uint32_t)payload_buf;

		if (copy_from_user((void *)m->msg.payload, buf, m->msg.length)) {
			ret = -EFAULT;
			sm_debug("copy failed\n");
			goto fail2;
		}

	} else {
		ret = -EINVAL;
		goto fail1;
	}

	if (session->proto_ops->sendmsg) {
		ret = session->proto_ops->sendmsg(m, session);
	} else {
		sm_debug("session type not supported\n");
		ret = 0;
	}
	if (ret)
		goto fail2;

	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__, (int)m->msg.length, (int)m->msg.type, (int)m->dst, (int)m->msg.dst_ep, (int)m->src, (int)m->msg.src_ep);
retry:
	ret = sm_send_message_internal(session, m);
	if (ret == -EAGAIN) {
		if (nonblock) {
			mutex_lock(&bfin_icc->sessions_table->lock);
			list_del(&m->next);
			mutex_unlock(&bfin_icc->sessions_table->lock);
			goto fail2;
		}
		sm_debug(">>>>sleep on send queue\n");
		interruptible_sleep_on(&m->icc_info->iccq_tx_wait);
		sm_debug("<<<<wakeup send queue\n");
		goto retry;
	} else {
		if (session->type == SP_TASK_MANAGER) {
			ret = wait_event_interruptible_timeout(m->icc_info->iccq_tx_wait,
			!sm_get_icc_queue_attribute(m->dst,
				ICC_QUEUE_ATTR_STATUS, &queue_status) &&
					(queue_status == ICC_QUEUE_READY), HZ);
			if (ret == 0) {
				sm_debug("coreb ready timeout\n");
				ret = -ETIMEDOUT;
			}
			kfree(m);
		}
		goto out;
	}

fail2:
	kfree(payload_buf);
fail1:
	kfree(m);
out:
	return ret;
}

static int sm_recv_scalar(uint32_t session_idx, uint32_t *src_ep,
		uint32_t *src_cpu, uint32_t *scalar0, uint32_t *scalar1,
		uint32_t *type, int nonblock)
{
	struct sm_session *session = NULL;
	struct sm_message *message = NULL;
	struct sm_msg *msg = NULL;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;

	sm_debug("recv sleep on queue index %s index %d\n", __func__, session_idx);

	if (list_empty(&session->rx_messages)) {
		sm_debug("recv sleep on queue\n");
		if (nonblock)
			return -EAGAIN;
		interruptible_sleep_on(&session->rx_wait);
	}

	if (list_empty(&session->rx_messages)) {
		sm_debug("finish wait by signal\n");
		return -EINTR;
	}

	mutex_lock(&bfin_icc->sessions_table->lock);
	message = list_first_entry(&session->rx_messages, struct sm_message, next);
	msg = &message->msg;

	list_del(&message->next);
	mutex_unlock(&bfin_icc->sessions_table->lock);

	if (src_ep)
		*src_ep = msg->src_ep;

	if (src_cpu)
		*src_cpu = message->src;

	if (scalar0)
		*scalar0 = msg->payload;
	if (scalar1)
		*scalar1 = msg->length;

	if (type) {
		switch (msg->type) {
		case SM_SCALAR_READY_8:
		case SM_SESSION_SCALAR_READY_8:
			*type = 1;
			break;
		case SM_SCALAR_READY_16:
		case SM_SESSION_SCALAR_READY_16:
			*type = 2;
			break;
		case SM_SCALAR_READY_32:
		case SM_SESSION_SCALAR_READY_32:
			*type = 4;
			break;
		case SM_SCALAR_READY_64:
		case SM_SESSION_SCALAR_READY_64:
			*type = 8;
			break;
		}
	}

	sm_debug("scalar0 %x, scalar1 %x type %d\n", *scalar0, *scalar1, *type);
	session->n_avail--;
	kfree(message);

	sm_debug("leave recv scalar\n");
	return 0;
}

static int sm_recv_packet(uint32_t session_idx, uint32_t *src_ep,
		uint32_t *src_cpu, void *user_buf, uint32_t *buf_len, int nonblock)
{
	struct sm_session *session = NULL;
	struct sm_message *message = NULL;
	struct sm_msg *msg = NULL;
	int ret = 0;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;

	if (list_empty(&session->rx_messages)) {
		sm_debug("recv sleep on queue\n");
		if (nonblock)
			return -EAGAIN;
		interruptible_sleep_on(&session->rx_wait);
	}

	if (list_empty(&session->rx_messages)) {
		sm_debug("should not fail here\n");
		return -EINTR;
	}

	mutex_lock(&bfin_icc->sessions_table->lock);
	message = list_first_entry(&session->rx_messages, struct sm_message, next);
	msg = &message->msg;
	list_del(&message->next);
	mutex_unlock(&bfin_icc->sessions_table->lock);

	if (src_ep)
		*src_ep = msg->src_ep;

	if (src_cpu)
		*src_cpu = message->src;

	if (buf_len)
		*buf_len = message->msg.length;

	if (copy_to_user(user_buf, (void *)message->msg.payload, message->msg.length))
		ret = -EFAULT;
	invalidate_dcache_range(msg->payload, msg->payload + msg->length);

	if (msg->type == SM_PACKET_READY)
		sm_send_packet_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);
	else if (msg->type == SM_SESSION_PACKET_READY)
		sm_send_session_packet_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);

	session->n_avail--;
	kfree(message);
	return ret;
}

static int sm_query_remote_ep(uint32_t dst_ep,
		uint32_t dst_cpu)
{
	int ret;
	mutex_lock(&bfin_icc->sessions_table->lock);
	bfin_icc->sessions_table->query_status = 1;
	mutex_unlock(&bfin_icc->sessions_table->lock);
	sm_debug("%s dst_ep %d dst_cpu %d\n", __func__, dst_ep, dst_cpu);
	sm_send_control_msg(NULL, dst_ep, dst_cpu, 0,
				0, SM_QUERY_MSG);
	ret = wait_event_interruptible_timeout(bfin_icc->sessions_table->query_wait,
				!bfin_icc->sessions_table->query_status, HZ);
	if (ret == -ERESTARTSYS)
		return ret;

	sm_debug("received query ack\n");
	return 0;
}

static int
sm_wait_for_connect_ack(struct sm_session *session)
{
	interruptible_sleep_on(&session->rx_wait);
	if (signal_pending(current)) {
		sm_debug("signal\n");
		return -EINTR;
	}
	if (session->flags == SM_CONNECT)
		return 0;
	else
		return -EAGAIN;
}

static int sm_get_remote_session_active(uint32_t session_idx,
		uint32_t dst_ep, uint32_t dst_cpu)
{
	struct sm_session *session;
	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	session->type = SP_SESSION_PACKET;
	sm_send_session_active(session, dst_ep, dst_cpu);
	wait_event_interruptible_timeout(session->rx_wait,
				(session->flags & SM_ACTIVE), 5);

	if (session->flags & SM_ACTIVE) {
		sm_debug("received active ack\n");
		return 0;
	}
	return -EAGAIN;
}

static int sm_connect_session(uint32_t session_idx, uint32_t dst_ep,
		uint32_t dst_cpu, uint32_t type)
{
	struct sm_session *session;
	uint32_t msg_type;
	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	if (type == SP_SESSION_SCALAR) {
		session->type = SP_SESSION_SCALAR;
		msg_type = SM_SESSION_SCALAR_CONNECT;
	} else if (type == SP_SESSION_PACKET) {
		session->type = SP_SESSION_PACKET;
		msg_type = SM_SESSION_PACKET_CONNECT;
	} else
		return -EINVAL;
	sm_send_connect(session, dst_ep, dst_cpu, msg_type);
	if (sm_wait_for_connect_ack(session))
		return -EAGAIN;
	sm_debug("received connect ack\n");
	if (session->remote_ep == dst_ep)
		sm_debug("auto accept\n");

	sm_send_connect_done(session, session->remote_ep, dst_cpu);
	return 0;
}

static int sm_disconnect_session(uint32_t dst_ep, uint32_t src_ep,
					struct sm_session_table *table)
{
	uint32_t slot = sm_find_session(src_ep, 0, table);
	if (slot < 0)
		return -EINVAL;

	table->sessions[slot].remote_ep = 0;
	table->sessions[slot].flags = 0;
}

static int sm_open_session(uint32_t index)
{
	struct sm_session *session;
	session = sm_index_to_session(index);
	if (!session)
		return -EINVAL;
	if (session->flags == SM_CONNECT) {
		session->flags |= SM_OPEN;
		return 0;
	}
	return -EINVAL;
}

static int sm_close_session(uint32_t index)
{
	struct sm_session *session;
	session = sm_index_to_session(index);
	if (!session)
		return -EINVAL;
	if (session->flags & SM_OPEN) {
		session->flags &= ~SM_OPEN;
		return 0;
	}
	return -EINVAL;
}

static int sm_destroy_session(uint32_t session_idx)
{
	struct sm_message *message;
	struct sm_msg *msg;
	struct sm_session *session;
	struct sm_session_table *table = bfin_icc->sessions_table;

	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;

	mutex_lock(&table->lock);
	while (!list_empty(&session->rx_messages)) {
		sm_debug("drain rx list\n");
		message = list_first_entry(&session->rx_messages,
					struct sm_message, next);
		msg = &message->msg;

		if (session->flags == SM_CONNECT)
			sm_send_session_packet_ack(session, msg->src_ep,
					message->src, msg->payload, msg->length);
		else
			sm_send_packet_ack(session, msg->src_ep,
					message->src, msg->payload, msg->length);
		list_del(&message->next);
		kfree(message);
	}
	mutex_unlock(&table->lock);

	mutex_lock(&table->lock);
	while (!list_empty(&session->tx_messages)) {
		mutex_unlock(&table->lock);
		set_current_state(TASK_INTERRUPTIBLE);
		sm_debug("drain tx list\n");
		schedule_timeout(HZ * 2);
		sm_debug("drain tx list1\n");
		set_current_state(TASK_RUNNING);

		mutex_lock(&table->lock);

		if (signal_pending(current)) {
			sm_debug("signal\n");
			while (!list_empty(&session->tx_messages)) {
				message = list_first_entry(&session->tx_messages,
						struct sm_message, next);
				list_del(&message->next);
				kfree(message);

			}
			mutex_unlock(&table->lock);
			return -ERESTARTSYS;
		}

		if (list_empty(&session->tx_messages))
			break;

		else {

		message = list_first_entry(&session->tx_messages,
					struct sm_message, next);
		list_del(&message->next);
		kfree(message);
		sm_debug("drop tx message dsp %x src %x type %x\n", message->msg.dst_ep, message->msg.src_ep, message->msg.type);
		}
	}
	mutex_unlock(&table->lock);

	if (session->flags == SM_CONNECT) {
		sm_send_close(session, session->remote_ep, 1);

		sm_debug("send close\n");
		interruptible_sleep_on(&session->rx_wait);
		if (signal_pending(current)) {
			sm_debug("signal\n");
			return -ERESTARTSYS;
		}
	}

	sm_free_session(session_idx, table);
	return 0;
}

static int
icc_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct sm_session_table *table = bfin_icc->sessions_table;
	table->refcnt++;
	return ret;
}

static int
icc_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct sm_session_table *table = bfin_icc->sessions_table;
	int i;
	pid_t pid = current->pid;
	table->refcnt--;
	for (i = 0; i < MAX_ENDPOINTS; i++) {
		if (table->sessions[i].pid == pid)
			sm_free_session(i, table);
	}

	return ret;
}

int icc_get_node_status(void *user_param, uint32_t size)
{
	int ret = 0;
	struct sm_node_status *param = kzalloc(sizeof(struct sm_node_status), GFP_KERNEL);
	struct sm_session_table *table = bfin_icc->sessions_table;
	if (!param)
		return -ENOMEM;
	param->session_mask = table->session_mask;
	param->session_pending = table->session_pending;
	param->nfree = table->nfree;
	if (copy_to_user(user_param, (void *)param, size))
		ret = -EFAULT;

	kfree(param);
	return ret;
}

int icc_get_session_status(void *user_param, uint32_t size, uint32_t session_idx)
{
	int ret = 0;
	struct sm_session_status *param;
	struct sm_session *session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;

	param = kzalloc(sizeof(struct sm_session_status), GFP_KERNEL);
	if (!param)
		return -ENOMEM;

	sm_debug("session status index %d, avail %d\n", session_idx, (int)session->n_avail);

	param->n_avail = session->n_avail;
	param->n_uncompleted = session->n_uncompleted;
	param->local_ep = session->local_ep;
	param->remote_ep = session->remote_ep;
	param->type = session->type;
	param->pid = session->pid;
	param->flags = session->flags;

	if (copy_to_user(user_param, (void *)param, size))
		ret = -EFAULT;
	kfree(param);
	return ret;
}

int icc_handle_scalar_cmd(struct sm_msg *msg)
{
	uint32_t scalar0, scalar1;
	uint16_t src_cpu;
	struct sm_session *session;
	struct sm_icc_desc *icc_info;
	int index;

	if (msg->type != SM_SCALAR_READY_64)
		return 0;

	scalar0 = msg->payload;
	scalar1 = msg->length;

	icc_info = get_icc_peer(msg);
	src_cpu = icc_info->peer_cpu;

	if (SM_SCALAR_CMD(scalar0) != SM_SCALAR_CMD_HEAD)
		return 0;

	switch (SM_SCALAR_CMDARG(scalar0)) {
	case SM_SCALAR_CMD_GET_SESSION_ID:
		index = sm_find_session(scalar1, 0, bfin_icc->sessions_table);
		session = sm_index_to_session(index);
		if (session) {
			scalar0 = MK_SM_SCALAR_CMD_ACK(SM_SCALAR_CMD_GET_SESSION_ID);
			scalar1 = index;
			sm_debug("found scalar0 %x scalar1 %x\n", scalar0, scalar1);
			sm_send_scalar_cmd(NULL, msg->src_ep, src_cpu, scalar0,
					scalar1);
		}
		break;
	case SM_SCALAR_CMD_GET_SESSION_TYPE:
		break;
	default:
		return 0;
	}

	return 1;
}

static long
icc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int nonblock;
	uint32_t local_ep;
	uint32_t remote_ep;
	uint32_t dst_cpu;
	uint32_t src_cpu;
	uint32_t len;
	uint32_t type;
	uint32_t session_idx;
	void *buf;
	uint32_t paddr;
	struct sm_packet *pkt = kzalloc(sizeof(struct sm_packet),
							GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;
	if (copy_from_user(pkt, (void *)arg, sizeof(struct sm_packet)))
		return -EFAULT;

	session_idx = pkt->session_idx;
	local_ep = pkt->local_ep;
	remote_ep = pkt->remote_ep;
	type = pkt->type;
	dst_cpu = pkt->dst_cpu;
	src_cpu = blackfin_core_id();
	len = pkt->buf_len;
	buf = pkt->buf;
	nonblock = (file->f_flags & O_NONBLOCK) | (pkt->flag & O_NONBLOCK);

	switch (cmd) {
	case CMD_SM_SEND:
		if ((SM_MSG_PROTOCOL(type) == SP_SCALAR) ||
			(SM_MSG_PROTOCOL(type) == SP_SESSION_SCALAR))
			ret = sm_send_scalar(session_idx, remote_ep, dst_cpu,
					(uint32_t)pkt->buf, pkt->buf_len, type, nonblock);
		else
			ret = sm_send_packet(session_idx, remote_ep,
					dst_cpu, buf, len, nonblock);
		break;
	case CMD_SM_RECV:
		if ((SM_MSG_PROTOCOL(type) == SP_SCALAR) ||
			(SM_MSG_PROTOCOL(type) == SP_SESSION_SCALAR))
			ret = sm_recv_scalar(session_idx, &pkt->remote_ep,
			&pkt->dst_cpu, (uint32_t *)&pkt->buf, &pkt->buf_len, &pkt->type, nonblock);
		else
			ret = sm_recv_packet(session_idx, &pkt->remote_ep,
				&pkt->dst_cpu, buf, &pkt->buf_len, nonblock);
		break;
	case CMD_SM_CREATE:
		ret = sm_create_session(local_ep, type, pkt->queue_priority);
		if (ret < 0) {
			sm_debug("create session failed srcep %d\n", (int)local_ep);
			ret = -EINVAL;
		}
		pkt->session_idx = ret;
		break;
	case CMD_SM_CONNECT:
		ret = sm_connect_session(session_idx, remote_ep, dst_cpu, type);
		break;
	case CMD_SM_OPEN:
		ret = sm_open_session(session_idx);
		break;
	case CMD_SM_CLOSE:
		ret = sm_close_session(session_idx);
		break;
	case CMD_SM_ACTIVE:
		ret = sm_get_remote_session_active(session_idx, remote_ep, dst_cpu);
		break;
	case CMD_SM_SHUTDOWN:
		ret = sm_destroy_session(session_idx);
		break;
	case CMD_SM_GET_NODE_STATUS:
		ret = icc_get_node_status(pkt->param, pkt->param_len);
		break;
	case CMD_SM_GET_SESSION_STATUS:
		ret = icc_get_session_status(pkt->param, pkt->param_len, session_idx);
		break;
	case CMD_SM_REQUEST_UNCACHED_BUF:
		buf = dma_alloc_coherent(NULL, pkt->buf_len, &paddr, GFP_KERNEL);
		if (!buf)
			ret = -ENOMEM;
		pkt->buf = buf;
		pkt->paddr = paddr;
		break;
	case CMD_SM_RELEASE_UNCACHED_BUF:
		dma_free_coherent(NULL, pkt->buf_len, pkt->buf, pkt->paddr);
		break;
	case CMD_SM_QUERY_REMOTE_EP:
		sm_query_remote_ep(remote_ep, dst_cpu);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (copy_to_user((void *)arg, pkt, sizeof(struct sm_packet)))
		ret = -EFAULT;
	kfree(pkt);
	return ret;
}

unsigned int icc_poll(struct file *file, poll_table *wait)
{
	struct sm_icc_desc *icc_info;
	unsigned int mask = 0, i;
	int pending;

	poll_wait(file, &bfin_icc->icc_rx_wait, wait);

	for (i = 0, icc_info = bfin_icc->icc_info; i < bfin_icc->peer_count; i++, icc_info++) {
		pending = iccqueue_getpending(icc_info);
		if (pending) {
			mask |= POLLIN | POLLRDNORM;
			break;
		}
	}

	return mask;
}

struct platform_device *saved_pdev;
int icc_find_dev_name(char *name)
{
	struct device *d;
	struct platform_device *pdev;

	sm_debug("%s\n", name);
	d = bus_find_device_by_name(&platform_bus_type, NULL, name);
	if (d == NULL) {
		sm_debug("no device found\n");
		return -ENODEV;
	} else {

		pdev = to_platform_device(d);

		saved_pdev = pdev;
		platform_device_unregister(pdev);
	}

	return 0;
}

int res_manage_request_peri(uint16_t subid)
{
	return 0;
}

void res_manage_free_peri(uint16_t subid)
{

}

int res_manage_request_gpio(uint16_t subid)
{
	return gpio_request(subid, "COREB");
}

void res_manage_free_gpio(uint16_t subid)
{
	gpio_free(subid);
}

int res_manage_request_irq(uint16_t subid)
{
	if (irq_settings_can_request(subid))
		return 0;
	disable_irq(subid);
	return 0;
}

void res_manage_free_irq(uint16_t subid)
{
	if (irq_settings_can_request(subid))
		return;
	enable_irq(subid);
}

int res_manage_request_dma(uint16_t subid)
{
	return request_dma(subid, "COREB");
}

void res_manage_free_dma(uint16_t subid)
{
	free_dma(subid);
}

int res_manage_request(uint16_t id)
{
	int ret = 0;
	uint16_t type, subid;
	type = RESMGR_TYPE(id);
	subid = RESMGR_SUBID(id);
	switch (type) {
	case RESMGR_TYPE_PERIPHERAL:
		res_manage_request_peri(subid);
		break;
	case RESMGR_TYPE_GPIO:
		res_manage_request_gpio(subid);
		break;
	case RESMGR_TYPE_SYS_IRQ:
		res_manage_request_irq(subid);
		break;
	case RESMGR_TYPE_DMA:
		res_manage_request_dma(subid);
		break;
	default:
		ret = -ENODEV;
	}
	return ret;
}

int res_manage_free(uint16_t id)
{
	int ret = 0;
	uint16_t type, subid;
	type = RESMGR_TYPE(id);
	subid = RESMGR_SUBID(id);
	switch (type) {
	case RESMGR_TYPE_PERIPHERAL:
		res_manage_free_peri(subid);
		break;
	case RESMGR_TYPE_GPIO:
		res_manage_free_gpio(subid);
		break;
	case RESMGR_TYPE_SYS_IRQ:
		res_manage_free_irq(subid);
		break;
	case RESMGR_TYPE_DMA:
		res_manage_free_dma(subid);
		break;
	default:
		ret = -ENODEV;
	}
	return ret;
}

static const struct file_operations icc_fops = {
	.owner          = THIS_MODULE,
	.open		= icc_open,
	.release	= icc_release,
	.unlocked_ioctl = icc_ioctl,
	.poll		= icc_poll,
};

static int msg_recv_internal(struct sm_msg *msg, struct sm_session *session)
{
	struct sm_icc_desc *icc_info;
	int cpu = blackfin_core_id();
	struct sm_message *message;
	int ret = 0;

	icc_info = get_icc_peer(msg);
	BUG_ON(!icc_info);

	message = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	else
		memcpy(&message->msg, msg, sizeof(struct sm_msg));

	message->dst = cpu;
	message->src = icc_info->peer_cpu;

	if ((SM_MSG_PROTOCOL(msg->type) == SP_SCALAR))
		sm_send_scalar_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);
	else if ((SM_MSG_PROTOCOL(msg->type) == SP_SESSION_SCALAR))
		sm_send_session_scalar_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);

	if (session->handle) {
		session->handle(message, session);
		return 0;
	}
	mutex_lock(&bfin_icc->sessions_table->lock);
	list_add_tail(&message->next, &session->rx_messages);
	session->n_avail++;
	sm_debug("%s wakeup wait thread avail %d\n", __func__, (int)session->n_avail);
	wake_up(&session->rx_wait);
	mutex_unlock(&bfin_icc->sessions_table->lock);
	return ret;
}



static int sm_default_sendmsg(struct sm_message *message, struct sm_session *session)
{
	struct sm_msg *msg = &message->msg;
	struct sm_message *m = message;
	sm_debug("%s session type %x\n", __func__, (uint32_t)session->type);
	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__, (int)m->msg.length, (uint32_t)m->msg.type, (int)m->dst, (int)m->msg.dst_ep, (int)m->src, (int)m->msg.src_ep);
	switch (session->type) {
	case SP_PACKET:
	case SP_SESSION_PACKET:
		flush_dcache_range(msg->payload, msg->payload + msg->length);
	case SP_SCALAR:
	case SP_SESSION_SCALAR:
		mutex_lock(&bfin_icc->sessions_table->lock);
		list_add_tail(&message->next, &session->tx_messages);
		session->n_uncompleted++;
		mutex_unlock(&bfin_icc->sessions_table->lock);
		break;
	case SM_PACKET_ERROR:
		sm_debug("SM ERROR %08x\n", (uint32_t)msg->payload);
		break;
	default:
		break;
	};
	return 0;
}

static int
sm_default_recvmsg(struct sm_msg *msg, struct sm_session *session)
{
	struct sm_icc_desc *icc_info;
	int ret = 0;
	struct sm_message *uncompleted;
	struct sm_session_table *table = bfin_icc->sessions_table;
	icc_info = get_icc_peer(msg);
	BUG_ON(!icc_info);
	sm_debug("%s msg type %x\n", __func__, (uint32_t)msg->type);
	switch (msg->type) {
	case SM_PACKET_CONSUMED:
	case SM_SESSION_PACKET_CONSUMED:
		mutex_lock(&table->lock);
		/* icc queue is FIFO, so handle first message */
		list_for_each_entry(uncompleted, &session->tx_messages, next) {
			if (uncompleted->msg.payload == msg->payload) {
				sm_debug("ack matched free buf %x message %p %p %p\n", (uint32_t)msg->payload, uncompleted, uncompleted->next.next, uncompleted->next.prev);
				goto matched;
			}
		}
		mutex_unlock(&table->lock);
		break;
matched:
		list_del(&uncompleted->next);
		mutex_unlock(&table->lock);
		kfree(uncompleted);
		kfree((void *)msg->payload);
		session->n_uncompleted--;
		wake_up(&icc_info->iccq_tx_wait);
		break;
	case SM_SCALAR_CONSUMED:
	case SM_SESSION_SCALAR_CONSUMED:
		mutex_lock(&table->lock);
		/* icc queue is FIFO, so handle first message */
		list_for_each_entry(uncompleted, &session->tx_messages, next) {
			if (uncompleted->msg.payload == msg->payload) {
				sm_debug("ack matched free buf %x\n", (uint32_t)msg->payload);
				goto matched1;
			}
		}
		mutex_unlock(&table->lock);
		break;
matched1:
		list_del(&uncompleted->next);
		mutex_unlock(&table->lock);
		kfree(uncompleted);
		session->n_uncompleted--;
		wake_up(&icc_info->iccq_tx_wait);
		break;
	case SM_SESSION_PACKET_CONNECT_ACK:
	case SM_SESSION_SCALAR_CONNECT_ACK:
		sm_debug("%s wakeup wait thread\n", __func__);
		session->remote_ep = msg->src_ep;
		session->flags = SM_CONNECT;
		wake_up(&session->rx_wait);
		break;
	case SM_SESSION_PACKET_CONNECT:
	case SM_SESSION_SCALAR_CONNECT:
		session->remote_ep = msg->src_ep;
		session->flags = SM_CONNECTING;
		session->type = SM_MSG_PROTOCOL(msg->type);
		sm_send_connect_ack(session, msg->src_ep, icc_info->peer_cpu);
		break;
	case SM_SESSION_PACKET_CONNECT_DONE:
	case SM_SESSION_SCALAR_CONNECT_DONE:
		sm_debug("%s connect done\n", __func__);
		session->flags = SM_CONNECT;
		break;
	case SM_SESSION_PACKET_ACTIVE:
		if (session->flags & SM_OPEN)
			sm_send_session_active_ack(session, msg->src_ep, icc_info->peer_cpu);
		else
			sm_send_session_active_noack(session, msg->src_ep, icc_info->peer_cpu);
		break;
	case SM_SESSION_PACKET_ACTIVE_ACK:
		if (session->flags & SM_OPEN) {
			if (msg->payload == SM_OPEN) {
				session->flags |= SM_ACTIVE;
				wake_up(&session->rx_wait);
			}
		}
		break;
	case SM_SESSION_PACKET_CLOSE:
	case SM_SESSION_SCALAR_CLOSE:
		session->remote_ep = 0;
		session->flags = 0;
		sm_send_close_ack(session, msg->src_ep, icc_info->peer_cpu);
		break;
	case SM_SESSION_PACKET_CLOSE_ACK:
	case SM_SESSION_SCALAR_CLOSE_ACK:
		session->remote_ep = 0;
		session->flags = 0;
		wake_up(&session->rx_wait);
		break;
	case SM_PACKET_READY:
	case SM_SESSION_PACKET_READY:
		if (SM_MSG_PROTOCOL(msg->type) != session->type) {
			sm_debug("msg type %08x unmatch session type %08x\n", (uint32_t)msg->type, (uint32_t)session->type);
			break;
		}
		msg_recv_internal(msg, session);
		break;
	case SM_SCALAR_READY_8:
	case SM_SCALAR_READY_16:
	case SM_SCALAR_READY_32:
	case SM_SCALAR_READY_64:
	case SM_SESSION_SCALAR_READY_8:
	case SM_SESSION_SCALAR_READY_16:
	case SM_SESSION_SCALAR_READY_32:
	case SM_SESSION_SCALAR_READY_64:
		msg_recv_internal(msg, session);
		break;
	case SM_PACKET_ERROR:
		sm_debug("SM ERROR %08x\n", (uint32_t)msg->payload);
		break;
	default:
		ret = -EINVAL;
	};

	sm_message_dequeue(msg);
	return ret;
}

static int sm_default_shutdown(struct sm_session *session)
{
	return 0;
}

static int sm_default_error(struct sm_msg *msg, struct sm_session *session)
{
	return 0;
}

static int sm_task_sendmsg(struct sm_message *message, struct sm_session *session)
{
	struct sm_msg *msg = &message->msg;
	struct sm_task *task;

	if (msg->length >= sizeof(struct sm_task))
		msg->type = SM_TASK_RUN;
	else
		msg->type = SM_TASK_KILL;
	sm_debug("%s msg type %x\n", __func__, (uint32_t)msg->type);
	switch (msg->type) {
	case SM_TASK_RUN:
		flush_dcache_range(_ramend, physical_mem_end - _ramend);
		task = (struct sm_task *)msg->payload;
		sm_debug("%s init addr%p\n", __func__, task->task_init);
		sm_set_icc_queue_attribute(message->dst, ICC_QUEUE_ATTR_STATUS, ICC_QUEUE_STOP);
		mutex_lock(&bfin_icc->sessions_table->lock);
		list_add_tail(&message->next, &session->tx_messages);
		session->n_uncompleted++;
		mutex_unlock(&bfin_icc->sessions_table->lock);
		break;
	case SM_TASK_KILL:
		break;
	default:
		break;
	};
	return 0;
}

static int sm_task_recvmsg(struct sm_msg *msg, struct sm_session *session)
{
	struct sm_message *message;
	sm_debug("%s msg type %x\n", __func__, (uint32_t)msg->type);
	switch (msg->type) {
	case SM_TASK_RUN_ACK:
		sm_debug("%s free %x\n", __func__, (uint32_t)msg->payload);
		message = list_first_entry(&session->tx_messages,
					struct sm_message, next);
		list_del(&message->next);

		kfree((void *)msg->payload);
		break;
	case SM_TASK_KILL_ACK:
		break;
	default:
		break;
	};
	sm_message_dequeue(msg);
	return 0;
}

static int sm_resouce_manage_recvmsg(struct sm_msg *msg, struct sm_session *session)
{
	int ret;
	struct sm_icc_desc *icc_info = get_icc_peer(msg);
	BUG_ON(!icc_info);

	sm_debug("%s msg type %x\n", __func__, (uint32_t)msg->type);
	switch (msg->type) {
	case SM_RES_MGR_REQUEST:
		sm_debug("%s free %x\n", __func__, (uint32_t)msg->payload);
		ret = res_manage_request((uint16_t)msg->payload);
		if (ret)
			sm_send_control_msg(session, msg->src_ep, icc_info->peer_cpu, 0,
					0, SM_RES_MGR_REQUEST_FAIL);
		else
			sm_send_control_msg(session, msg->src_ep, icc_info->peer_cpu, 0,
					0, SM_RES_MGR_REQUEST_OK);

		break;
	case SM_RES_MGR_FREE:
		res_manage_free((uint16_t)msg->payload);
		sm_send_control_msg(session, msg->src_ep, icc_info->peer_cpu, 0,
					0, SM_RES_MGR_FREE_DONE);
		break;
	default:
		break;
	};
	sm_message_dequeue(msg);
	return 0;
}


struct sm_proto core_control_proto = {
	.sendmsg = NULL,
	.recvmsg = NULL,
	.shutdown = NULL,
	.error = NULL,
};

struct sm_proto task_manager_proto = {
	.sendmsg = sm_task_sendmsg,
	.recvmsg = sm_task_recvmsg,
	.shutdown = NULL,
	.error = NULL,
};

struct sm_proto res_manager_proto = {
	.sendmsg = NULL,
	.recvmsg = sm_resouce_manage_recvmsg,
	.shutdown = NULL,
	.error = NULL,
};

struct sm_proto packet_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

struct sm_proto session_packet_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

struct sm_proto scalar_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

struct sm_proto session_scalar_proto = {
	.sendmsg = sm_default_sendmsg,
	.recvmsg = sm_default_recvmsg,
	.shutdown = sm_default_shutdown,
	.error = sm_default_error,
};

int __handle_general_msg(struct sm_msg *msg)
{
	int index;
	struct sm_session *session;
	struct sm_icc_desc *icc_info;
	struct sm_message *message;
	int ret = 0;

	switch (msg->type) {
	case SM_BAD_MSG:
		printk(KERN_WARNING "%s", (char *)msg->payload);
		ret = 1;
		break;
	case SM_QUERY_MSG:
		icc_info = get_icc_peer(msg);
		index = sm_find_session(msg->dst_ep, 0, bfin_icc->sessions_table);
		session = sm_index_to_session(index);
		if (session) {
			sm_send_control_msg(session, 0,
				icc_info->peer_cpu, 0, 0, SM_QUERY_ACK_MSG);
		} else {
			message = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
			if (message) {
				memcpy(&message->msg, msg, sizeof(struct sm_msg));
				message->src = icc_info->peer_cpu;
				mutex_lock(&bfin_icc->sessions_table->lock);
				list_add_tail(&message->next,
					&bfin_icc->sessions_table->query_message);
				mutex_unlock(&bfin_icc->sessions_table->lock);
			}
		}
		ret = 1;
		break;
	case SM_QUERY_ACK_MSG:
		mutex_lock(&bfin_icc->sessions_table->lock);
		bfin_icc->sessions_table->query_status = 0;
		mutex_unlock(&bfin_icc->sessions_table->lock);
		wake_up_interruptible(&bfin_icc->sessions_table->query_wait);
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

int __msg_handle(struct sm_icc_desc *icc_info, struct sm_message_queue *inqueue)
{
	uint16_t received = sm_atomic_read(&inqueue->received);
	uint16_t sent = sm_atomic_read(&inqueue->sent);
	uint16_t pending;
	struct sm_msg *msg;
	struct sm_session *session;
	int index;

	pending = sent - received;
	if (pending < 0)
		pending += USHRT_MAX;
	pending %= SM_MSGQ_LEN;
	if (pending == 0)
		return pending;

	msg = &inqueue->messages[(received % SM_MSGQ_LEN)];

	if (__handle_general_msg(msg)) {
		sm_message_dequeue(msg);
		return 1;
	}

	index = sm_find_session(msg->dst_ep, 0, bfin_icc->sessions_table);

	session = sm_index_to_session(index);

	if (!session) {
		sm_debug("discard msg type %x dst %x src %x\n", (uint32_t)msg->type, msg->dst_ep, msg->src_ep);
		sm_message_dequeue(msg);
		wake_up(&icc_info->iccq_tx_wait);
		return 1;
	}

	sm_debug("session %p index %d msg type%x\n", session, index, (uint32_t)msg->type);

	if (session->proto_ops->recvmsg)
		session->proto_ops->recvmsg(msg, session);
	else
		sm_debug("session type not supported\n");

	return 1;
}

void msg_handle(struct sm_icc_desc *icc_info)
{
	int ret;

	/* icc_queue[0] is the queue to receive message */
	ret = __msg_handle(icc_info, icc_info->icc_high_queue);
	if (!ret)
		__msg_handle(icc_info, icc_info->icc_queue);
}
static int message_queue_thread(void *data)
{
	int pending;
	struct sm_icc_desc *icc_info = (struct sm_icc_desc *)data;

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		pending = iccqueue_getpending(icc_info);
		if (!pending) {
			if (kthread_should_stop()) {
				set_current_state(TASK_RUNNING);
				break;
			}
			schedule_timeout(HZ);
			continue;
		}
		set_current_state(TASK_RUNNING);

		msg_handle(icc_info);

	} while (1);
	return 0;
}

void register_sm_proto(struct bfin_icc *icc)
{
	icc->sm_protos[SP_CORE_CONTROL] = &core_control_proto;
	icc->sm_protos[SP_TASK_MANAGER] = &task_manager_proto;
	icc->sm_protos[SP_RES_MANAGER] = &res_manager_proto;
	icc->sm_protos[SP_PACKET] = &packet_proto;
	icc->sm_protos[SP_SESSION_PACKET] = &session_packet_proto;
	icc->sm_protos[SP_SCALAR] = &scalar_proto;
	icc->sm_protos[SP_SESSION_SCALAR] = &session_scalar_proto;
}

static int
icc_write_proc(struct file *file, const char __user * buffer,
		unsigned long count, void *data)
{
	char line[256];
	unsigned long val;
	int ret;
	struct sm_session *session;
	struct sm_session_table *table = bfin_icc->sessions_table;

	ret = copy_from_user(line, buffer, count);
	if (ret)
		return -EFAULT;

	line[count - 1] = '\0';

	if (!strcmp(line, "test")) {
		icc_find_dev_name("bfin-spi.0");
		return count;
	}

	if (!strcmp(line, "test1")) {
		platform_device_add(saved_pdev);
		return count;
	}

	if (strict_strtoul(line, 10, &val))
		return -EINVAL;

	sm_debug("session index %ld\n", val);
	if (val < 0 && val >= MAX_SESSIONS)
		return -EINVAL;

	session = &table->sessions[val];
	sm_debug(" %x", session->local_ep);
	sm_debug(" %x", session->remote_ep);
	sm_debug(" %X", (uint32_t)session->type);
	sm_debug(" %X\n", (uint32_t)session->flags);
	return count;
}

static irqreturn_t ipi_handler_int0(int irq, void *dev_instance)
{
	struct sm_icc_desc *icc_info = (struct sm_icc_desc *)dev_instance;
	unsigned int cpu = blackfin_core_id();

	icc_clear_ipi_cpu(cpu, icc_info->irq);
	wakeup_icc_thread(icc_info);
	return IRQ_HANDLED;
}

static int __devinit bfin_icc_probe(struct platform_device *pdev)
{
	struct icc_platform_data *icc_data = (struct icc_platform_data *)pdev->dev.platform_data;
	struct bfin_icc *icc = NULL;
	struct sm_icc_desc *icc_info;
	int ret, i;

	if (bfin_icc) {
		dev_err(&pdev->dev, "Can't register more than one bfin_icc device.\n");
		return -ENOENT;
	}

	if (!icc_data || icc_data->peer_count <= 0) {
		dev_err(&pdev->dev, "No ICC platform data are defined.\n");
		return -ENOENT;
	}

	icc = kzalloc(sizeof(*icc), GFP_KERNEL);
	if (!icc) {
		dev_err(&pdev->dev, "fail to malloc bfin_icc\n");
		return -ENOMEM;
	}

	icc->mdev.minor	= MISC_DYNAMIC_MINOR;
	snprintf(icc->name, 20, "%s", DRIVER_NAME);
	icc->mdev.name	= icc->name;
	icc->mdev.fops	= &icc_fops;
	icc->peer_count = icc_data->peer_count;
	init_waitqueue_head(&icc->icc_rx_wait);

	icc->icc_info = kzalloc(sizeof(struct sm_icc_desc) * icc->peer_count, GFP_KERNEL);
	if (!icc->icc_info) {
		dev_err(&pdev->dev, "fail to malloc icc_info array\n");
		ret = -ENOMEM;
		goto out_error_free_mem;
	}

	for (i = 0, icc_info = icc->icc_info; i < icc->peer_count; i++, icc_info++) {
		icc_info->peer_cpu = icc_data->peer_info[i].peerid;
		icc_info->notify = icc_data->peer_info[i].notify;
		if( icc_info->notify < 0) {
			dev_err(&pdev->dev, "No ICC Notify specified\n");
			ret = -ENOENT;
			goto out_error_free_mem;
		}

		icc_info->irq = icc_data->peer_info[i].irq;
		if (icc_info->irq < 0) {
			dev_err(&pdev->dev, "No ICC receive IRQ specified\n");
			ret = -ENOENT;
			goto out_error_free_mem;
		}

		ret = request_irq(icc_info->irq, ipi_handler_int0, IRQF_PERCPU,
					"ICC receive IRQ", icc_info);
		if (ret) {
			dev_err(&pdev->dev, "Fail to request ICC receive IRQ\n");
			goto out_error_free_mem;
		}

		/* icc_queue[0] is rx queue, icc_queue[1] is tx queue. */
		icc_info->icc_high_queue = (struct sm_message_queue *)icc_data->peer_info[i].phy_peer_mem;
		icc_info->icc_queue = (struct sm_message_queue *)icc_info->icc_high_queue + 2;
		memset(icc_info->icc_high_queue, 0, MSGQ_SIZE);
		icc_info->icc_queue_attribute = (uint32_t *)((uint32_t)
			icc_info->icc_high_queue + MSGQ_SIZE);
		memset(icc_info->icc_queue_attribute, 0, 4 * ICC_QUEUE_ATTR_MAX);

		init_waitqueue_head(&icc_info->iccq_tx_wait);
		icc_info->iccq_thread = kthread_run(message_queue_thread,
					icc_info, "iccqd");
		if (IS_ERR(icc_info->iccq_thread))
			sm_debug("kthread create failed %ld\n", PTR_ERR(icc_info->iccq_thread));
	}

	ret = misc_register(&icc->mdev);
	if (ret) {
		dev_err(&pdev->dev, "cannot register ICC miscdev\n");
		goto out_error_free_irq;
	}

	init_sm_session_table(icc);

	register_sm_proto(icc);

	icc->icc_dump = create_proc_entry("icc_dump", 644, NULL);
	icc->icc_dump->write_proc = icc_write_proc;

	dev_set_drvdata(&pdev->dev, icc);
	bfin_icc = icc;

	ret = sm_create_session(EP_RESMGR_SERVICE, SP_RES_MANAGER, 0);
	BUG_ON(ret < 0);

	dev_info(&pdev->dev, "initialized\n");

	return 0;

out_error_free_irq:
	for (i = 0, icc_info = icc->icc_info; i < icc->peer_count; i++, icc_info++) {
		if (icc_info->irq)
			free_irq(icc_info->irq, icc_info);
	}
out_error_free_mem:
	if (icc->icc_info)
		kfree(icc->icc_info);
	kfree(icc);
	return ret;
}

static int __devexit bfin_icc_remove(struct platform_device *pdev)
{
	struct bfin_icc *icc = platform_get_drvdata(pdev);
	struct sm_icc_desc *icc_info;
	int i;

	bfin_icc = NULL;
	dev_set_drvdata(&pdev->dev, NULL);

	if (icc) {
		misc_deregister(&icc->mdev);
		for (i = 0, icc_info = icc->icc_info; i < icc->peer_count; i++, icc_info++) {
			if (icc_info->irq)
				free_irq(icc_info->irq, icc_info);
		}
		if (icc->icc_info)
			kfree(icc->icc_info);
		kfree(icc);
	}

	return 0;
}

static struct platform_driver bfin_icc_driver = {
	.probe     = bfin_icc_probe,
	.remove    = __devexit_p(bfin_icc_remove),
	.driver    = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(bfin_icc_driver);

MODULE_AUTHOR("Steven Miao <steven.miao@analog.com>");
MODULE_DESCRIPTION("Blackfin ICC driver");
MODULE_LICENSE("GPL");

