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
	unsigned int offset = msg - MSGQ_START_ADDR;
	n = offset / sizeof(struct sm_message_queue);
	if ((n % 2) == 0)
		return n + 1;
	else
		return n;
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

static int sm_message_dequeue(int srccpu, struct sm_message *msg)
{
	struct sm_message_queue *inqueue = &icc_info->icc_queue[srccpu];
	sm_atomic_t received = sm_atomic_read(&inqueue->received);
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
		return -EINVAL;
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
		table->sessions[index].type = type;
		table->sessions[index].proto_ops = sm_protos[type];
		INIT_LIST_HEAD(&table->sessions[index].bufs);
		INIT_LIST_HEAD(&table->sessions[index].messages);
		init_waitqueue_head(&table->sessions[index].rx_wait);
		sm_debug("create ep index %d srcep %d type %d\n", index, src_ep, type);
		sm_debug("session %p\n", &table->sessions[index]);
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
	session = &table->sessions[session_idx];
	return session;
}

static sm_uint32_t sm_session_to_index(struct sm_session *session)
{
	struct sm_session_table *table = icc_info->sessions_table;
	sm_uint32_t index;
	if ((session > &table->sessions[0])
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
					len, SM_SESSION_PACKET_COMSUMED);
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
sm_send_packet(sm_uint32_t session_idx, sm_uint32_t dst_ep, sm_uint32_t dst_cpu,
		void *buf, sm_uint32_t len)
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

	m->msg.src_ep = session->local_ep;
	m->src = blackfin_core_id();
	m->msg.dst_ep = dst_ep;
	m->dst = dst_cpu;
	m->msg.length = len;

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
		ret = session->proto_ops->sendmsg(&m->msg, session);
	} else {
		sm_debug("session type not supported\n");
		ret = 0;
	}
	if (ret)
		goto fail;

	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__, m->msg.length, m->msg.type, m->dst, m->msg.dst_ep, m->src, m->msg.src_ep);
	ret = sm_send_message_internal(&m->msg, m->dst, m->src);
	if (ret)
		goto fail;

fail:
	kfree(payload_buf);
out:
	kfree(m);
	return ret;
}

static int sm_recv_packet(sm_uint32_t local_ep, void *user_buf,
			int nonblock)
{
	struct sm_session *session = NULL;
	struct sm_message *message = NULL;
	struct sm_msg *msg = NULL;
	sm_uint32_t index = sm_find_session(local_ep, 0, icc_info->sessions_table);

	session = sm_index_to_session(index);

	if (list_empty(&session->messages)) {
		sm_debug("recv sleep on queue\n");
		if (nonblock)
			return -EAGAIN;
		mutex_unlock(&icc_info->sessions_table->lock);
		interruptible_sleep_on(&session->rx_wait);
		mutex_lock(&icc_info->sessions_table->lock);
	}

	if (!iccqueue_getpending(0))
		return -EINTR;

	message = list_first_entry(&session->messages, struct sm_message, next);
	msg = &message->msg;

	copy_to_user(user_buf, (void *)message->msg.payload, message->msg.length);

	if (msg->type == SM_PACKET_READY)
		sm_send_packet_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);
	else if (msg->type == SM_SESSION_PACKET_READY)
		sm_send_session_packet_ack(session, msg->src_ep, message->src,
				msg->payload, msg->length);

	list_del(&message->next);
	kfree(message);
	return 0;
}

static int
sm_wait_for_connect_ack(struct sm_session *session)
{
	mutex_unlock(&icc_info->sessions_table->lock);
	interruptible_sleep_on_timeout(&session->rx_wait, 1000);
	mutex_lock(&icc_info->sessions_table->lock);
	if (!iccqueue_getpending(0))
		return -EINTR;
	return 0;
}

static int sm_connect_session(sm_uint32_t dst_ep, sm_uint32_t dst_cpu,
			sm_uint32_t src_ep)
{
	struct sm_session *session;
	struct sm_session_table *table = icc_info->sessions_table;
	sm_uint32_t slot = sm_find_session(src_ep, 0, table);
	if (slot >= 32)
		return -EINVAL;
	session = &table->sessions[slot];
	sm_send_connect(session, dst_ep, dst_cpu);
	if (sm_wait_for_connect_ack(session))
		return -EAGAIN;
	table->sessions[slot].remote_ep = dst_ep;
	table->sessions[slot].flags = SM_CONNECT;
	sm_send_connect_done(session, dst_ep, dst_cpu);
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

static int sm_update_session(sm_uint32_t src_ep, sm_uint32_t dst_ep,
					struct sm_session_table *table)
{
	sm_uint32_t slot = sm_find_session(src_ep, dst_ep, table);
	if (slot >= 32)
		return -EINVAL;
	table->sessions[slot].remote_ep = dst_ep;
	table->sessions[slot].flags = SM_CONNECT;
}

static int sm_destroy_session(sm_uint32_t src_ep)
{
	struct sm_session_table *table = icc_info->sessions_table;
	sm_uint32_t slot = sm_find_session(src_ep, 0, table);
	struct sm_session *session;
	struct sm_message *message = NULL;
	struct sm_msg *msg = NULL;
	if (slot < 0)
		return -EINVAL;

	session = &table->sessions[slot];
	while (!list_empty(&session->messages)) {
		message = list_first_entry(&session->messages,
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

	sm_free_session(slot, table);
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

static long
icc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sm_packet *pkt = kzalloc(sizeof(struct sm_packet),
							GFP_KERNEL);
	int ret = 0;
	int nonblock = (file->f_flags & O_NONBLOCK);
	sm_uint32_t local_ep;
	sm_uint32_t remote_ep;
	sm_uint32_t dst_cpu;
	sm_uint32_t src_cpu;
	sm_uint32_t len;
	sm_uint32_t type;
	sm_uint32_t session_idx;
	void *buf;
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

	mutex_lock(&icc_info->sessions_table->lock);
	switch (cmd) {
	case CMD_SM_SEND:
		ret = sm_send_packet(session_idx, remote_ep, dst_cpu, buf, len);
		break;
	case CMD_SM_RECV:
		ret = sm_recv_packet(local_ep, buf, nonblock);
		break;
	case CMD_SM_CREATE:
		ret = sm_create_session(local_ep, type);
		if (ret < 0) {
			sm_debug("create session failed srcep %d\n", local_ep);
			ret = -EINVAL;
		}
		pkt->session_idx = ret;
		break;
	case CMD_SM_CONNECT:
		ret = sm_connect_session(remote_ep, dst_cpu, local_ep);
		break;
	case CMD_SM_SHUTDOWN:
		ret = sm_destroy_session(local_ep);
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
	message = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!message)
		ret = -ENOMEM;
	else
		memcpy(&message->msg, msg, sizeof(struct sm_message));

	message->dst = cpu;
	message->src = cpu ^ 1;
	list_add(&message->next, &session->messages);
	sm_debug("%s wakeup wait thread\n", __func__);
	wake_up(&session->rx_wait);
	return ret;
}

static int sm_default_sendmsg(struct sm_msg *msg, struct sm_session *session)
{
	sm_debug("%s session type %x\n", __func__, session->type);
	switch (session->type) {
	case SP_PACKET:
		msg->type = SM_MSG_TYPE(session->type, 0);
		break;
	case SP_SESSION_PACKET:
		msg->type = SM_MSG_TYPE(session->type, 0);
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
	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (msg->type) {
	case SM_PACKET_CONSUMED:
	case SM_SESSION_PACKET_COMSUMED:
		sm_debug("free buf %x\n", msg->payload);
		kfree((void *)msg->payload);
		break;
	case SM_SESSION_PACKET_CONNECT_ACK:
		sm_debug("%s wakeup wait thread\n", __func__);
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
	case SM_SESSION_PACKET_CLOSE:
		session->remote_ep = 0;
		session->flags = 0;
		sm_send_close_ack(session, msg->src_ep, cpu ^ 1);
	case SM_SESSION_PACKET_CLOSE_ACK:
		session->remote_ep = 0;
		session->flags = 0;
	case SM_PACKET_READY:
		msg_recv_internal(msg, session);
		break;
	case SM_SESSION_PACKET_READY:
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

static int sm_task_sendmsg(struct sm_msg *msg, struct sm_session *session)
{
	struct sm_task *task;
	if (msg->length >= sizeof(struct sm_task))
		msg->type = SM_TASK_RUN;
	else
		msg->type = SM_TASK_KILL;
	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (msg->type) {
	case SM_TASK_RUN:
		task = (struct sm_task *)msg->payload;
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
	sm_uint32_t *buf = msg->payload;
	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (msg->type) {
	case SM_TASK_RUN_ACK:
		sm_debug("task id %d\n", buf[0]);
		kfree(msg->payload);
		break;
	case SM_TASK_KILL_ACK:
		sm_debug("task exit with %d\n", buf[0]);
		break;
	default:
		break;
	};
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
		sm_debug("%s\n", msg->payload);
		sm_message_dequeue(cpu, msg);
		return;
	}

	index = sm_find_session(msg->dst_ep, 0, icc_info->sessions_table);

	session = sm_index_to_session(index);

	if (session && (SM_MSG_PROTOCOL(msg->type) == session->type)) {
		if (session->proto_ops->recvmsg)
			session->proto_ops->recvmsg(msg, session);
		else
			sm_debug("session type not supported\n");
	} else {
		printk("discard msg\n");
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
}

static int __init bf561_icc_test_init(void)
{
	int ret = 0;

	ret = icc_init();
	init_sm_message_queue();

	init_sm_session_table();

	register_sm_proto();

	init_waitqueue_head(&icc_info->iccq_rx_wait);
	icc_info->iccq_thread = kthread_run(message_queue_thread,
					icc_info->icc_queue, "iccqd");
	if (IS_ERR(icc_info->iccq_thread))
		sm_debug("kthread create failed %d\n", PTR_ERR(icc_info->iccq_thread));
	return misc_register(&icc_dev);
}
module_init(bf561_icc_test_init);

static void __exit bf561_icc_test_exit(void)
{
	misc_deregister(&icc_dev);
}
module_exit(bf561_icc_test_exit);

MODULE_DESCRIPTION("BF561 ICC"); MODULE_LICENSE("GPL");
