/* Multicore communication on a BF561
 *
 * Copyright 2004-2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */


#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/bitmap.h>
#include <linux/slab.h>
#include <icc.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>

#define DEBUG
#ifdef DEBUG
#define sm_debug(fmt, ...) \
	printk(KERN_CRIT "sm_debug:"pr_fmt(fmt), ##__VA_ARGS__)
#else
#define sm_debug(fmt, ...) \
	({ if (0) printk(KERN_CRIT "sm_debug:"pr_fmt(fmt), ##__VA_ARGS__); 0; })
#endif

struct sm_icc_desc *icc_info;
struct sm_proto *sm_protos[SP_MAX];
struct proc_dir_entry *icc_dump;

static int icc_init(void)
{
	icc_info = kzalloc(sizeof(struct sm_icc_desc), GFP_KERNEL);
	if (!icc_info)
		return -ENOMEM;
	return 0;
}

void wakeup_icc_thread(void)
{
	if (icc_info->iccq_thread)
		wake_up_process(icc_info->iccq_thread);
	wake_up(&icc_info->iccq_rx_wait);
}

static int init_sm_message_queue(void)
{
	icc_info->icc_queue = (struct sm_message_queue *)MSGQ_START_ADDR;
	memset(icc_info->icc_queue, 0, sizeof(struct sm_message_queue)*2);
	return 0;
}

static int get_msg_src(struct sm_msg *msg)
{
	unsigned int n = 0;
	unsigned int offset;
	unsigned int align = 256;
	offset = (unsigned int)msg - MSGQ_START_ADDR;
	if (align < sizeof(struct sm_message_queue))
		align = (sizeof(struct sm_message_queue) + align - 1) / align;
	n = offset / align;
	if ((n % 2) == 0)
		return n + 1;
	else
		return 0;
}

static int init_sm_session_table(void)
{
	icc_info->sessions_table = kzalloc(sizeof(struct sm_session_table),
					GFP_KERNEL);
	if (!icc_info->sessions_table)
		return -ENOMEM;
	mutex_init(&icc_info->sessions_table->lock);
	return 0;
}

static int sm_message_enqueue(int dstcpu, int srccpu, struct sm_msg *msg)
{
	struct sm_message_queue *outqueue = &icc_info->icc_queue[dstcpu];
	sm_atomic_t sent = sm_atomic_read(&outqueue->sent);
	sm_atomic_t received = sm_atomic_read(&outqueue->received);
	if ((sent - received) >= (SM_MSGQ_LEN - 1)) {
		sm_debug("over run\n");
		return -EAGAIN;
	}
	memcpy(&outqueue->messages[(sent%SM_MSGQ_LEN)], msg,
		sizeof(struct sm_message));
	sent++;
	sm_atomic_write(&outqueue->sent, sent);
	sm_debug("send message cpu %d sent %d\n", dstcpu, sent);
	return 0;
}

static int sm_message_dequeue(int srccpu, struct sm_msg *msg)
{
	struct sm_message_queue *inqueue = &icc_info->icc_queue[srccpu];
	sm_atomic_t received;
	memset(msg, 0, sizeof(struct sm_msg));
	received = sm_atomic_read(&inqueue->received);
	received++;
	sm_atomic_write(&inqueue->received, received);
	sm_debug("received %d\n", received);
	return 0;
}

static sm_uint32_t sm_alloc_session(struct sm_session_table *table)
{
	sm_uint32_t index;
	sm_debug("table bits1 %08x\n", table->bits[0]);
	index = find_next_zero_bit(table->bits, BITS_PER_LONG, 0);
	if (index >= BITS_PER_LONG)
		return -EAGAIN;
	sm_debug("table index %d\n", index);
	bitmap_set(table->bits, index, 1);

	sm_debug("table bits2 %08x\n", table->bits[0]);
	table->nfree--;
	return index;
}

static int sm_free_session(sm_uint32_t slot, struct sm_session_table *table)
{
	memset(&table->sessions[slot], 0, sizeof(struct sm_session));
	__clear_bit(slot, table->bits);
	table->nfree++;
	return 0;
}

static int
sm_find_session(sm_uint32_t local_ep, sm_uint32_t remote_ep,
			struct sm_session_table *table)
{
	sm_uint32_t index;
	struct sm_session *session;
	sm_debug("%s bits %08x localep %d\n", __func__, table->bits[0], local_ep);
	for_each_set_bit(index, table->bits, BITS_PER_LONG) {
		session = &table->sessions[index];
		sm_debug("index %d ,local ep %d type %x\n", index, session->local_ep, session->type);
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

static int sm_create_session(sm_uint32_t src_ep, sm_uint32_t type)
{
	struct sm_session_table *table = icc_info->sessions_table;
	sm_uint32_t index = sm_find_session(src_ep, 0, table);
	if (index >= 0 && index < 32) {
		sm_debug("already bound index %d srcep %d\n", index, src_ep);
		return -EEXIST;
	}
	if (type >= SP_MAX) {
		sm_debug("bad type %x\n", type);
		return -EINVAL;
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
		table->sessions[index].proto_ops = sm_protos[type];
		INIT_LIST_HEAD(&table->sessions[index].rx_messages);
		INIT_LIST_HEAD(&table->sessions[index].tx_messages);
		init_waitqueue_head(&table->sessions[index].rx_wait);
		sm_debug("create ep index %d srcep %d type %d\n", index, src_ep, type);
		sm_debug("session %p\n", &table->sessions[index]);
		sm_debug("size of session %d\n", sizeof(struct sm_session));
		return index;
	}
	return -EAGAIN;
}

static struct sm_session *sm_index_to_session(sm_uint32_t session_idx)
{
	struct sm_session *session;
	struct sm_session_table *table = icc_info->sessions_table;
	if (session_idx < 0 && session_idx >= MAX_SESSIONS)
		return NULL;
	if (!test_bit(session_idx, table->bits))
		return NULL;
	session = &table->sessions[session_idx];
	return session;
}

static sm_uint32_t sm_session_to_index(struct sm_session *session)
{
	struct sm_session_table *table = icc_info->sessions_table;
	sm_uint32_t index;
	if ((session >= &table->sessions[0])
		&& (session < &table->sessions[MAX_SESSIONS])) {
		return (session - &table->sessions[0])/sizeof(struct sm_session);
	}
	return -EINVAL;
}

static int iccqueue_getpending(sm_uint32_t srccpu)
{
	struct sm_message_queue *inqueue = &icc_info->icc_queue[srccpu];
	sm_atomic_t sent = sm_atomic_read(&inqueue->sent);
	sm_atomic_t received = sm_atomic_read(&inqueue->received);
	sm_atomic_t pending;
/*	printk("sm msgq sent=%d received=%d\n", sent, received); */
	pending = sent - received;
	if (pending < 0)
		pending += USHRT_MAX;
	pending %= SM_MSGQ_LEN;
	return pending;
}

static int sm_send_message_internal(struct sm_msg *msg, int dst_cpu,
					int src_cpu)
{
	int ret = 0;
	sm_debug("%s: dst %d src %d\n", __func__, dst_cpu, src_cpu);
	ret = sm_message_enqueue(dst_cpu, src_cpu, msg);
	if (!ret)
		icc_send_ipi_cpu(dst_cpu, IRQ_SUPPLE_0);
	return ret;
}


int
sm_send_control_msg(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu, sm_uint32_t payload,
			sm_uint32_t len, sm_uint32_t type)
{
	struct sm_msg *m;
	int ret = 0;
	int src = blackfin_core_id();

	m = kzalloc(sizeof(struct sm_msg), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->type = type;
	if (session)
		m->src_ep = session->local_ep;
	m->dst_ep = remote_ep;
	m->length = len;
	m->payload = payload;

	ret = sm_send_message_internal(m, dst_cpu, src);
	if (ret)
		return -EAGAIN;
	kfree(m);
	return ret;

}


int sm_send_packet_ack(struct sm_session *session, sm_uint32_t remote_ep,
		sm_uint32_t dst_cpu, sm_uint32_t payload, sm_uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_PACKET_CONSUMED);
}

int
sm_send_session_packet_ack(struct sm_session *session, sm_uint32_t remote_ep,
		sm_uint32_t dst_cpu, sm_uint32_t payload, sm_uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SESSION_PACKET_CONSUMED);
}

int sm_send_scalar_cmd(struct sm_session *session, sm_uint32_t remote_ep,
		sm_uint32_t dst_cpu, sm_uint32_t payload, sm_uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SCALAR_READY_64);
}

int sm_send_scalar_ack(struct sm_session *session, sm_uint32_t remote_ep,
		sm_uint32_t dst_cpu, sm_uint32_t payload, sm_uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SCALAR_CONSUMED);
}

int
sm_send_session_scalar_ack(struct sm_session *session, sm_uint32_t remote_ep,
		sm_uint32_t dst_cpu, sm_uint32_t payload, sm_uint32_t len)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, payload,
					len, SM_SESSION_SCALAR_CONSUMED);
}

int sm_send_connect(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CONNECT);
}

int sm_send_close(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CLOSE);
}

int sm_send_connect_ack(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CONNECT_ACK);
}

int sm_send_connect_done(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CONNECT_DONE);
}

int sm_send_session_active(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_ACTIVE);
}

int sm_send_session_active_ack(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, SM_OPEN,
					0, SM_SESSION_PACKET_ACTIVE_ACK);
}

int sm_send_session_active_noack(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_ACTIVE_ACK);
}

int sm_send_close_ack(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_SESSION_PACKET_CLOSE_ACK);
}

int sm_send_error(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu)
{
	return sm_send_control_msg(session, remote_ep, dst_cpu, 0,
					0, SM_PACKET_ERROR);
}

static int
sm_send_scalar(sm_uint32_t session_idx, sm_uint32_t dst_ep, sm_uint32_t dst_cpu,
		uint32_t scalar0, uint32_t scalar1, uint32_t type, int nonblock)
{
	struct sm_session *session;
	struct sm_message *m;
	int ret = -EAGAIN;
	if (session_idx < 0 || session_idx >= MAX_SESSIONS)
		return -EINVAL;
	session = sm_index_to_session(session_idx);
	sm_debug("%s: %u %p\n", __func__, session_idx, session);
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

	sm_debug("%s: scalar0 %x scalar1 %x type %x dst %d dstep %d src %d srcep %d\n", __func__, scalar0, scalar1, m->msg.type, m->dst, m->msg.dst_ep, m->src, m->msg.src_ep);
retry:
	ret = sm_send_message_internal(&m->msg, m->dst, m->src);
	if ((!nonblock) && (ret == -EAGAIN)) {
		interruptible_sleep_on(&icc_info->iccq_tx_wait);
		goto retry;
	} else {
		goto out;
	}
out:
	return ret;
}

static int
sm_send_packet(sm_uint32_t session_idx, sm_uint32_t dst_ep, sm_uint32_t dst_cpu,
		void *buf, sm_uint32_t len, int nonblock)
{
	struct sm_session *session;
	struct sm_message *m;
	void *payload_buf = NULL;
	int ret = -EAGAIN;
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

	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__, m->msg.length, m->msg.type, m->dst, m->msg.dst_ep, m->src, m->msg.src_ep);
	if (m->msg.length) {
		payload_buf = kzalloc(m->msg.length, GFP_KERNEL);
		if (!payload_buf) {
			ret = -ENOMEM;
			goto out;
		}
		sm_debug("alloc buffer %x\n", payload_buf);

		m->msg.payload = (sm_address_t)payload_buf;

		if (copy_from_user((void *)m->msg.payload, buf, m->msg.length)) {
			ret = -EFAULT;
			goto fail;
		}

	} else {
		ret = -EINVAL;
		goto out;
	}

	if (session->proto_ops->sendmsg) {
		ret = session->proto_ops->sendmsg(m, session);
	} else {
		sm_debug("session type not supported\n");
		ret = 0;
	}
	if (ret)
		goto fail;

	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__, m->msg.length, m->msg.type, m->dst, m->msg.dst_ep, m->src, m->msg.src_ep);
retry:
	ret = sm_send_message_internal(&m->msg, m->dst, m->src);
	if ((!nonblock) && (ret == -EAGAIN)) {
		interruptible_sleep_on(&icc_info->iccq_tx_wait);
		goto retry;
	} else {
		goto out;
	}

fail:
	kfree(payload_buf);
out:
	return ret;
}

static int sm_recv_scalar(sm_uint32_t session_idx, sm_uint16_t *src_ep,
		sm_uint16_t *src_cpu, uint32_t *scalar0, uint32_t *scalar1,
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
		mutex_unlock(&icc_info->sessions_table->lock);
		interruptible_sleep_on(&session->rx_wait);
		mutex_lock(&icc_info->sessions_table->lock);
	}

	if (list_empty(&session->rx_messages)) {
		sm_debug("finish wait by signal\n");
		return -EINTR;
	}

	message = list_first_entry(&session->rx_messages, struct sm_message, next);
	msg = &message->msg;

	list_del(&message->next);

	if (src_ep)
		*src_ep = msg->src_ep;

	if (src_cpu)
		*src_cpu = message->src;

	if (scalar0)
		*scalar0 = msg->payload;
	if (scalar1)
		*scalar1 = msg->length;
	if (type)
		*type = msg->type;

	sm_debug("scalar0 %x, scalar1 %x\n", *scalar0, *scalar1);
	session->n_avail--;
	kfree(message);

	sm_debug("leave recv scalar\n");
	return 0;
}

static int sm_recv_packet(sm_uint32_t session_idx, sm_uint16_t *src_ep,
		sm_uint16_t *src_cpu, void *user_buf, uint32_t *buf_len, int nonblock)
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
		mutex_unlock(&icc_info->sessions_table->lock);
		interruptible_sleep_on(&session->rx_wait);
		mutex_lock(&icc_info->sessions_table->lock);
	}

	if (list_empty(&session->rx_messages))
		return -EINTR;

	message = list_first_entry(&session->rx_messages, struct sm_message, next);
	msg = &message->msg;
	list_del(&message->next);

	if (src_ep)
		*src_ep = msg->src_ep;

	if (src_cpu)
		*src_cpu = message->src;

	if (buf_len)
		*buf_len = message->msg.length;


	sm_debug("recv %s\n", message->msg.payload);

	copy_to_user(user_buf, (void *)message->msg.payload, message->msg.length);

	blackfin_dcache_invalidate_range(msg->payload, msg->payload + msg->length);

	if (msg->type == SM_PACKET_READY)
		sm_send_packet_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);
	else if (msg->type == SM_SESSION_PACKET_READY)
		sm_send_session_packet_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);

	session->n_avail--;
	kfree(message);
	return 0;
}

static int
sm_wait_for_connect_ack(struct sm_session *session)
{
	long timeout;
	mutex_unlock(&icc_info->sessions_table->lock);
	interruptible_sleep_on(&session->rx_wait);
	mutex_lock(&icc_info->sessions_table->lock);
	if (signal_pending(current)) {
		sm_debug("signal\n");
		return -EINTR;
	}
	if (session->flags == SM_CONNECT)
		return 0;
	else
		return -EAGAIN;
}

static int sm_get_remote_session_active(sm_uint32_t session_idx, sm_uint32_t dst_ep, sm_uint32_t dst_cpu)
{
	struct sm_session *session;
	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	session->type = SP_SESSION_PACKET;
	sm_send_session_active(session, dst_ep, dst_cpu);
	mutex_unlock(&icc_info->sessions_table->lock);
	wait_event_interruptible_timeout(session->rx_wait,
				(session->flags & SM_ACTIVE), 5);
	mutex_lock(&icc_info->sessions_table->lock);

	if (session->flags & SM_ACTIVE) {
		sm_debug("received active ack\n");
		return 0;
	}
	return -EAGAIN;
}

static int sm_connect_session(sm_uint32_t session_idx, sm_uint32_t dst_ep, sm_uint32_t dst_cpu)
{
	struct sm_session *session;
	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	session->type = SP_SESSION_PACKET;
	sm_send_connect(session, dst_ep, dst_cpu);
	if (sm_wait_for_connect_ack(session))
		return -EAGAIN;
	sm_debug("received connect ack\n");
	if (session->remote_ep == dst_ep)
		sm_debug("auto accept\n");

	sm_send_connect_done(session, session->remote_ep, dst_cpu);
	return 0;
}

static int sm_disconnect_session(sm_uint32_t dst_ep, sm_uint32_t src_ep,
					struct sm_session_table *table)
{
	sm_uint32_t slot = sm_find_session(src_ep, 0, table);
	if (slot < 0)
		return -EINVAL;

	table->sessions[slot].remote_ep = 0;
	table->sessions[slot].flags = 0;
}

static int sm_open_session(sm_uint32_t index)
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

static int sm_close_session(sm_uint32_t index)
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

static int sm_destroy_session(sm_uint32_t session_idx)
{
	struct sm_message *message;
	struct sm_msg *msg;
	struct sm_session *session;
	struct sm_session_table *table = icc_info->sessions_table;
	session = sm_index_to_session(session_idx);
	if (!session)
		return -EINVAL;
	while (!list_empty(&session->rx_messages)) {
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

	if (session->flags == SM_CONNECT)
		sm_send_close(session, msg->src_ep, message->src);

	sm_free_session(session_idx, table);
	return 0;
}

static int
icc_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	return ret;
}

static int
icc_release(struct inode *inode, struct file *file)
{
	int ret = 0;

	return ret;
}

int icc_get_node_status(void *user_param, uint32_t size)
{
	int ret = 0;
	struct sm_node_status *param = kzalloc(sizeof(struct sm_node_status), GFP_KERNEL);
	if (!param)
		return -ENOMEM;
	param->session_mask = icc_info->sessions_table->session_mask;
	param->session_pending = icc_info->sessions_table->session_pending;
	param->nfree = icc_info->sessions_table->nfree;
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

	sm_debug("session status index %d, avail %d\n", session_idx, session->n_avail);
	param->avail = session->n_avail;
	param->uncomplete = session->n_uncompleted;
	param->status = session->flags;

	if (copy_to_user(user_param, (void *)param, size))
		ret = -EFAULT;
	kfree(param);
	return ret;
}

int icc_handle_scalar_cmd(struct sm_msg *msg)
{
	int ret;
	uint32_t scalar0, scalar1;
	uint16_t src_cpu;
	struct sm_session *session;
	int index;

	if (msg->type != SM_SCALAR_READY_64)
		return 0;

	scalar0 = msg->payload;
	scalar1 = msg->length;

	src_cpu = get_msg_src(msg);

	if (SM_SCALAR_CMD(scalar0) != SM_SCALAR_CMD_HEAD)
		return 0;

	switch (SM_SCALAR_CMDARG(scalar0)) {
	case SM_SCALAR_CMD_GET_SESSION_ID:
		index = sm_find_session(scalar1, 0, icc_info->sessions_table);
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
	sm_uint32_t local_ep;
	sm_uint32_t remote_ep;
	sm_uint32_t dst_cpu;
	sm_uint32_t src_cpu;
	sm_uint32_t len;
	sm_uint32_t type;
	sm_uint32_t session_idx;
	struct sm_session *session;
	void *buf;
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
	mutex_lock(&icc_info->sessions_table->lock);
	switch (cmd) {
	case CMD_SM_SEND:
		if ((SM_MSG_PROTOCOL(type) == SP_SCALAR) ||
			(SM_MSG_PROTOCOL(type) == SP_SESSION_SCALAR))
			ret = sm_send_scalar(session_idx, remote_ep, dst_cpu,
					pkt->buf, pkt->buf_len, type, nonblock);
		else
			ret = sm_send_packet(session_idx, remote_ep,
					dst_cpu, buf, len, nonblock);
		break;
	case CMD_SM_RECV:
		if ((SM_MSG_PROTOCOL(type) == SP_SCALAR) ||
			(SM_MSG_PROTOCOL(type) == SP_SESSION_SCALAR))
			ret = sm_recv_scalar(session_idx, &pkt->remote_ep,
			&pkt->dst_cpu, &pkt->buf, &pkt->buf_len, &pkt->type, nonblock);
		else
			ret = sm_recv_packet(session_idx, &pkt->remote_ep,
				&pkt->dst_cpu, buf, &pkt->buf_len, nonblock);
		break;
	case CMD_SM_CREATE:
		ret = sm_create_session(local_ep, type);
		if (ret < 0) {
			sm_debug("create session failed srcep %d\n", local_ep);
			ret = -EINVAL;
			break;
		}
		pkt->session_idx = ret;
		break;
	case CMD_SM_CONNECT:
		ret = sm_connect_session(session_idx, remote_ep, dst_cpu);
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
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&icc_info->sessions_table->lock);
	if (copy_to_user((void *)arg, pkt, sizeof(struct sm_packet)))
		ret = -EFAULT;
	kfree(pkt);
	return ret;
}

unsigned int icc_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	int cpu = blackfin_core_id();
	int pending;

	poll_wait(file, &icc_info->iccq_rx_wait, wait);

	pending = iccqueue_getpending(cpu);
	if (pending)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}


static const struct file_operations icc_fops = {
	.owner          = THIS_MODULE,
	.open		= icc_open,
	.release	= icc_release,
	.unlocked_ioctl = icc_ioctl,
	.poll		= icc_poll,
};

static struct miscdevice icc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "icc",
	.fops  = &icc_fops,
};


static int msg_recv_internal(struct sm_msg *msg, struct sm_session *session)
{
	int cpu = blackfin_core_id();
	struct sm_message *message;
	int ret = 0;
	struct sm_session_table *table = icc_info->sessions_table;
	struct sm_session *s;
	message = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!message)
		return -ENOMEM;
	else
		memcpy(&message->msg, msg, sizeof(struct sm_message));

	message->dst = cpu;
	message->src = cpu ^ 1;

	if (session->handle) {
		session->handle(message, session);
		return 0;
	}
	mutex_lock(&icc_info->sessions_table->lock);
	list_add(&message->next, &session->rx_messages);
	session->n_avail++;
	sm_debug("%s wakeup wait thread\n", __func__);
	wake_up(&session->rx_wait);
	mutex_unlock(&icc_info->sessions_table->lock);
	return ret;
}

static int sm_default_sendmsg(struct sm_message *message, struct sm_session *session)
{
	struct sm_msg *msg = &message->msg;
	struct sm_message *m = message;
	sm_debug("%s session type %x\n", __func__, session->type);
	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__, m->msg.length, m->msg.type, m->dst, m->msg.dst_ep, m->src, m->msg.src_ep);
	switch (session->type) {
	case SP_PACKET:
	case SP_SESSION_PACKET:
		blackfin_dcache_flush_range(msg->payload, msg->payload + msg->length);
	case SP_SCALAR:
	case SP_SESSION_SCALAR:
		list_add(&message->next, &session->tx_messages);
		session->n_uncompleted++;
		break;
	case SM_PACKET_ERROR:
		printk("SM ERROR %08x\n", msg->payload);
		break;
	default:
		break;
	};
	return 0;
}

static int
sm_default_recvmsg(struct sm_msg *msg, struct sm_session *session)
{
	int ret = 0;
	int cpu = blackfin_core_id();
	struct sm_message *uncompleted;
	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (msg->type) {
	case SM_PACKET_CONSUMED:
	case SM_SESSION_PACKET_CONSUMED:
		mutex_lock(&icc_info->sessions_table->lock);
		/* icc queue is FIFO, so handle first message */
		list_for_each_entry(uncompleted, &session->tx_messages, next) {
			if (uncompleted->msg.payload == msg->payload) {
				sm_debug("ack matched free buf %x\n", msg->payload);
				goto matched;
			}
			sm_debug("unmatched ack %08x %x uncomplete tx %08x\n", msg->payload, msg->length, uncompleted->msg.payload);
		}
		mutex_unlock(&icc_info->sessions_table->lock);
		break;
matched:
		list_del(&uncompleted->next);
		mutex_unlock(&icc_info->sessions_table->lock);
		kfree(uncompleted);
		kfree((void *)msg->payload);
		session->n_uncompleted--;
		wake_up(&icc_info->iccq_tx_wait);
		break;
	case SM_SCALAR_CONSUMED:
	case SM_SESSION_SCALAR_CONSUMED:
		mutex_lock(&icc_info->sessions_table->lock);
		/* icc queue is FIFO, so handle first message */
		list_for_each_entry(uncompleted, &session->tx_messages, next) {
			if (uncompleted->msg.payload == msg->payload) {
				sm_debug("ack matched free buf %x\n", msg->payload);
				goto matched1;
			}
			sm_debug("unmatched ack %08x %x uncomplete tx %08x\n", msg->payload, msg->length, uncompleted->msg.payload);
		}
		mutex_unlock(&icc_info->sessions_table->lock);
		break;
matched1:
		list_del(&uncompleted->next);
		mutex_unlock(&icc_info->sessions_table->lock);
		kfree(uncompleted);
		session->n_uncompleted--;
		wake_up(&icc_info->iccq_tx_wait);
		break;
	case SM_SESSION_PACKET_CONNECT_ACK:
		sm_debug("%s wakeup wait thread\n", __func__);
		session->remote_ep = msg->src_ep;
		session->flags = SM_CONNECT;
		wake_up(&session->rx_wait);
		break;
	case SM_SESSION_PACKET_CONNECT:
		session->remote_ep = msg->src_ep;
		session->flags = SM_CONNECTING;
		sm_send_connect_ack(session, msg->src_ep, cpu ^ 1);
		break;
	case SM_SESSION_PACKET_CONNECT_DONE:
		sm_debug("%s connect done\n", __func__);
		session->flags = SM_CONNECT;
		break;
	case SM_SESSION_PACKET_ACTIVE:
		if (session->flags & SM_OPEN)
			sm_send_session_active_ack(session, msg->src_ep, cpu ^ 1);
		else
			sm_send_session_active_noack(session, msg->src_ep, cpu ^ 1);
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
		session->remote_ep = 0;
		session->flags = 0;
		sm_send_close_ack(session, msg->src_ep, cpu ^ 1);
	case SM_SESSION_PACKET_CLOSE_ACK:
		session->remote_ep = 0;
		session->flags = 0;
	case SM_PACKET_READY:
	case SM_SESSION_PACKET_READY:
		if (SM_MSG_PROTOCOL(msg->type) != session->type) {
			coreb_msg("msg type %08x unmatch session type %08x\n", msg->type, session->type);
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
		printk("SM ERROR %08x\n", msg->payload);
		break;
	default:
		ret = -EINVAL;
	};

	sm_message_dequeue(cpu, msg);
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
	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (msg->type) {
	case SM_TASK_RUN:
		task = (struct sm_task *)msg->payload;
		sm_debug("%s init addr%p\n", __func__, task->task_init);
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
	int cpu = blackfin_core_id();
	sm_uint32_t *buf = msg->payload;
	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (msg->type) {
	case SM_TASK_RUN_ACK:
		sm_debug("%s free %x\n", __func__, msg->payload);
		kfree(msg->payload);
		break;
	case SM_TASK_KILL_ACK:
		break;
	default:
		break;
	};
	sm_message_dequeue(cpu, msg);
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
	.recvmsg = NULL,
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

void msg_handle(int cpu)
{
	struct sm_message_queue *inqueue = &icc_info->icc_queue[cpu];
	sm_atomic_t sent = sm_atomic_read(&inqueue->sent);
	sm_atomic_t received = sm_atomic_read(&inqueue->received);
	struct sm_msg *msg;
	struct sm_session *session;
	sm_uint32_t index;

	msg = &inqueue->messages[(received % SM_MSGQ_LEN)];

	if (msg->type == SM_BAD_MSG) {
		sm_debug("%p %s", msg->payload, msg->payload);
		sm_debug("\t%d : %d\n", msg->src_ep, msg->dst_ep);
		sm_message_dequeue(cpu, msg);
		return;
	}

	if (icc_handle_scalar_cmd(msg)) {
		sm_debug("handle scalar cmd\n");
		sm_message_dequeue(cpu, msg);
		return;
	}

	index = sm_find_session(msg->dst_ep, 0, icc_info->sessions_table);

	session = sm_index_to_session(index);
	sm_debug("session %p index %d msg type%x\n", session, index, msg->type);

	if (!session) {
		sm_message_dequeue(cpu, msg);
		return;
	}

	if (session) {
		if (session->proto_ops->recvmsg)
			session->proto_ops->recvmsg(msg, session);
		else
			sm_debug("session type not supported\n");
	} else {
		sm_debug("discard msg type %x sessiontype %x\n", msg->type, session->type);
		sm_message_dequeue(cpu, msg);
	}
}

static int message_queue_thread(void *d)
{
	struct sm_message_queue *queue = d;
	int pending;
	int cpu = blackfin_core_id();
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		pending = iccqueue_getpending(cpu);
		if (!pending) {
			if (kthread_should_stop()) {
				set_current_state(TASK_RUNNING);
				break;
			}
			schedule();
			continue;
		}
		set_current_state(TASK_RUNNING);

		msg_handle(cpu);

	} while (1);
	return 0;
}

void register_sm_proto()
{
	sm_protos[SP_CORE_CONTROL] = &core_control_proto;
	sm_protos[SP_TASK_MANAGER] = &task_manager_proto;
	sm_protos[SP_RES_MANAGER] = &res_manager_proto;
	sm_protos[SP_PACKET] = &packet_proto;
	sm_protos[SP_SESSION_PACKET] = &session_packet_proto;
	sm_protos[SP_SCALAR] = &scalar_proto;
	sm_protos[SP_SESSION_SCALAR] = &session_scalar_proto;
}

static int
icc_write_proc(struct file *file, const char __user * buffer,
		unsigned long count, void *data)
{
	char line[8];
	unsigned long val;
	int ret;
	struct sm_session *session;
	struct sm_session_table *table = icc_info->sessions_table;

	ret = copy_from_user(line, buffer, count);
	if (ret)
		return -EFAULT;

	if (strict_strtoul(line, 10, &val))
		return -EINVAL;

	sm_debug("session index %d\n", val);
	if (val < 0 && val >= MAX_SESSIONS)
		return -EINVAL;

	session = &table->sessions[val];
	sm_debug(" %d", session->local_ep);
	sm_debug(" %d", session->remote_ep);
	sm_debug(" %X", session->type);
	sm_debug(" %X\n", session->flags);

	return count;
}

static int __init bf561_icc_test_init(void)
{
	int ret = 0;

	ret = icc_init();
	init_sm_message_queue();

	init_sm_session_table();

	register_sm_proto();

	init_waitqueue_head(&icc_info->iccq_rx_wait);
	init_waitqueue_head(&icc_info->iccq_tx_wait);
	icc_info->iccq_thread = kthread_run(message_queue_thread,
					icc_info->icc_queue, "iccqd");
	if (IS_ERR(icc_info->iccq_thread))
		sm_debug("kthread create failed %ld\n", PTR_ERR(icc_info->iccq_thread));

	icc_dump = create_proc_entry("icc_dump", 644, NULL);
	icc_dump->write_proc = icc_write_proc;
	return misc_register(&icc_dev);
}
module_init(bf561_icc_test_init);

static void __exit bf561_icc_test_exit(void)
{
	misc_deregister(&icc_dev);
}
module_exit(bf561_icc_test_exit);

MODULE_DESCRIPTION("BF561 ICC"); MODULE_LICENSE("GPL");
