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

#ifdef DEBUG
#define sm_debug(fmt, ...) \
	printk(KERN_CRIT "sm_debug:"pr_fmt(fmt), ##__VA_ARGS__)
#else
#define sm_debug(fmt, ...) \
	({if (0) printk(KERN_CRIT "sm_debug:"pr_fmt(fmt), ##__VA_ARGS__); 0; })
#endif

void platform_request_ipi(int irq, void *handler, void *data);

void platform_send_ipi_cpu(unsigned int cpu, int irq);

void platform_clear_ipi(unsigned int cpu, int irq);

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

static int init_sm_session_table(void)
{
	icc_info->sessions_table = kzalloc(sizeof(struct sm_session_table),
					GFP_KERNEL);
	if (!icc_info->sessions_table)
		return -ENOMEM;
	mutex_init(&icc_info->sessions_table->lock);
	return 0;
}

static int sm_message_enqueue(int dstcpu, int srccpu, struct sm_message *msg)
{
	struct sm_message_queue *outqueue = &icc_info->icc_queue[dstcpu];
	struct sm_message *message;
	sm_atomic_t sent = sm_atomic_read(&outqueue->sent);
	sm_atomic_t received = sm_atomic_read(&outqueue->received);
	if ((sent - received) >= (SM_MSGQ_LEN - 1)) {
		sm_debug("over run\n");
		return -EAGAIN;
	}
	memcpy(&outqueue->messages[(sent%SM_MSGQ_LEN)], msg,
		sizeof(struct sm_message));
	message = &outqueue->messages[(sent%SM_MSGQ_LEN)];
	sent++;
	sm_atomic_write(&outqueue->sent, sent);
	return 0;
}

static int sm_message_dequeue(int srccpu, struct sm_message *msg)
{
	struct sm_message_queue *inqueue = &icc_info->icc_queue[srccpu];
	sm_atomic_t received = sm_atomic_read(&inqueue->received);
	received++;
	sm_atomic_write(&inqueue->received, received);
	return 0;
}

static int iccqueue_getpending(sm_uint32_t srccpu)
{
	struct sm_message_queue *inqueue = &icc_info->icc_queue[srccpu];
	sm_atomic_t sent = sm_atomic_read(&inqueue->sent);
	sm_atomic_t received = sm_atomic_read(&inqueue->received);
	sm_atomic_t pending;
	printk("sm msgq sent=%d received=%d\n", sent, received);
	pending = sent - received;
	if (pending < 0)
		pending += USHRT_MAX;
	pending %= SM_MSGQ_LEN;
	return pending;
}

static int sm_send_message_internal(struct sm_message *msg, int dst_cpu,
					int src_cpu)
{
	int ret = 0;
	ret = sm_message_enqueue(dst_cpu, src_cpu, msg);
	if (!ret)
		platform_send_ipi_cpu(dst_cpu, IRQ_SUPPLE_0);
	return ret;
}

static int
sm_send_message(sm_uint32_t session_idx, sm_uint32_t dst_ep, sm_uint32_t dst_cpu,
		void *buf, sm_uint32_t len, struct sm_session_table *table)
{
	struct sm_session *session;
	struct sm_message *m;
	void *payload_buf = NULL;
	int ret = -EAGAIN;
	if (session_idx < 0)
		return -EINVAL;
	session = &table->sessions[session_idx];
	m = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->src_ep = session->local_ep;
	m->src = blackfin_core_id();
	m->dst_ep = dst_ep;
	m->dst = dst_cpu;
	m->length = len;
	m->type = SM_MSG_TYPE(session->type, 0);

	sm_debug("%s: %u \n", __func__, session_idx);
	sm_debug("%s: len %d type %x dst %d dstep %d src %d srcep %d\n", __func__, m->length, m->type, m->dst, m->dst_ep, m->src, m->src_ep);
	if (m->length) {
		payload_buf = kzalloc(m->length, GFP_KERNEL);
		if (!payload_buf) {
			ret = -ENOMEM;
			goto out;
		}

		m->payload = (sm_address_t)payload_buf;

		if (copy_from_user((void *)m->payload, buf, m->length)) {
			ret = -EFAULT;
			goto fail;
		}

		ret = session->proto_ops->sendmsg(m, session);
		if (ret)
			goto fail;

		ret = sm_send_message_internal(m, m->dst, m->src);
		if (ret)
			goto fail;

	} else {
		ret = -EINVAL;
		goto out;
	}

fail:
	kfree(payload_buf);
out:
	kfree(m);
	return ret;
}

int
sm_send_control_msg(struct sm_session *session, sm_uint32_t remote_ep,
			sm_uint32_t dst_cpu, sm_uint32_t payload,
			sm_uint32_t len, sm_uint32_t type)
{
	struct sm_message *m;
	int ret = 0;

	m = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	sm_debug("alloc message\n");

	m->type = type;
	m->src = blackfin_core_id();
	m->dst = dst_cpu;
	m->src_ep = session->local_ep;
	m->dst_ep = remote_ep;
	m->length = len;
	m->payload = payload;

	ret = sm_send_message_internal(m, m->dst, m->src);
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

static sm_uint32_t sm_get_session(struct sm_session_table *table)
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

static int sm_put_session(sm_uint32_t slot, struct sm_session_table *table)
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
		sm_debug("index %d ,local ep %d\n", index, session->local_ep);
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

static int sm_create_session(sm_uint32_t src_ep, sm_uint32_t type,
				struct sm_session_table *table)
{
	sm_uint32_t slot = sm_find_session(src_ep, 0, table);
	if (slot >= 0 && slot < 32) {
		sm_debug("already bound slot %d srcep %d\n", slot, src_ep);
		return -EINVAL;
	}

	slot = sm_get_session(table);
	if (slot >= 0 && slot < 32) {
		table->sessions[slot].local_ep = src_ep;
		table->sessions[slot].remote_ep = 0;
		table->sessions[slot].pid = current->pid;
		table->sessions[slot].flags = 0;
		table->sessions[slot].type = type;
		table->sessions[slot].proto_ops = sm_protos[type];
		INIT_LIST_HEAD(&table->sessions[slot].messages);
		init_waitqueue_head(&table->sessions[slot].rx_wait);
		sm_debug("create ep slot %d srcep %d type %d\n", slot, src_ep, type);
		return 0;
	}
	return -EAGAIN;
}

static int
sm_wait_for_connect_ack(struct sm_session *session)
{
	interruptible_sleep_on_timeout(&session->rx_wait, 1000);

	if (!iccqueue_getpending(0))
		return -EINTR;
	return 0;
}

static int sm_destroy_session(sm_uint32_t src_ep, struct sm_session_table *table)
{
	sm_uint32_t slot = sm_find_session(src_ep, 0, table);
	struct sm_session *session;
	struct sm_message *msg = NULL;
	if (slot < 0)
		return -EINVAL;

	session = &table->sessions[slot];
	while (!list_empty(&session->messages)) {
		msg = list_first_entry(&session->messages,
					struct sm_message, next);

		if (session->flags == SM_CONNECT)
			sm_send_session_packet_ack(session, msg->src_ep,
					msg->src, msg->payload, msg->length);
		else
			sm_send_packet_ack(session, msg->src_ep,
					msg->src, msg->payload, msg->length);
		list_del(&msg->next);
		kfree(msg);
	}

	if (session->flags == SM_CONNECT)
		sm_send_close(session, msg->src_ep, msg->src);

	sm_put_session(slot, table);
	return 0;
}

static int sm_connect_session(sm_uint32_t dst_ep, sm_uint32_t dst_cpu,
			sm_uint32_t src_ep, struct sm_session_table *table)
{
	struct sm_session *session;
	sm_uint32_t slot = sm_find_session(src_ep, 0, table);
	if (slot >= 32)
		return -EINVAL;
	session = &table->sessions[slot];
	sm_send_connect(session, dst_ep, dst_cpu);
	if (sm_wait_for_connect_ack(session))
		return -EAGAIN;
	table->sessions[slot].remote_ep = dst_ep;
	table->sessions[slot].flags = SM_CONNECT;
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


static int sm_recv_message(sm_uint32_t local_ep, void *user_buf,
			struct sm_session_table *table, int nonblock)
{
	struct sm_message *message = NULL;
	sm_uint32_t slot = sm_find_session(local_ep, 0, table);

	struct sm_session *session = NULL;
	if (slot >= 0 && slot < 32)
		session = &table->sessions[slot];
	else
		return -EINVAL;


	if (list_empty(&session->messages)) {
		sm_debug("recv sleep on queue\n");
		if (nonblock)
			return -EAGAIN;
		interruptible_sleep_on(&session->rx_wait);
	}

	if (!iccqueue_getpending(0))
		return -EINTR;

	message = list_first_entry(&session->messages, struct sm_message, next);

	copy_to_user(user_buf, (void *)message->payload, message->length);
	list_del(&message->next);
	kfree(message);
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
	struct sm_user_param *param = kzalloc(sizeof(struct sm_user_param),
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
	if (!param)
		return -ENOMEM;
	if (copy_from_user(param, (void *)arg, sizeof(struct sm_user_param)))
		return -EFAULT;

	session_idx = param->session_idx;
	local_ep = param->local_ep;
	remote_ep = param->remote_ep;
	type = param->type;
	dst_cpu = param->dst_cpu;
	src_cpu = blackfin_core_id();
	len = param->buf_len;
	buf = param->buf;

	switch (cmd) {
	case CMD_SM_SEND:
		ret = sm_send_message(session_idx, remote_ep, dst_cpu, buf,
					len, icc_info->sessions_table);
		break;
	case CMD_SM_RECV:
		ret = sm_recv_message(local_ep, buf, icc_info->sessions_table,
					nonblock);
		break;
	case CMD_SM_CREATE:
		ret = sm_create_session(local_ep, type,
					icc_info->sessions_table);
		break;
	case CMD_SM_CONNECT:
		ret = sm_connect_session(remote_ep, dst_cpu, local_ep,
					icc_info->sessions_table);
		break;
	case CMD_SM_SHUTDOWN:
		ret = sm_destroy_session(local_ep, icc_info->sessions_table);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (copy_to_user((void *)arg, param, sizeof(struct sm_user_param)))
		return -EFAULT;
	kfree(param);
	return 0;
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


static int msg_recv_internal(struct sm_message *msg, struct sm_session *session)
{
	struct sm_message *message;
	int ret = 0;
	message = kzalloc(sizeof(struct sm_message), GFP_KERNEL);
	if (!message)
		ret = -ENOMEM;
	else
		memcpy(message, msg, sizeof(struct sm_message));

	list_add(&message->next, &session->messages);
	sm_debug("%s wakeup wait thread\n", __func__);
	wake_up(&session->rx_wait);
	return ret;
}

static int sm_default_sendmsg(struct sm_message *msg, struct sm_session *session)
{
	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (SM_MSG_PROTOCOL(msg->type)) {
	case SP_PACKET:
		break;
	case SP_SESSION_PACKET:
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
sm_default_recvmsg(struct sm_message *msg, struct sm_session *session)
{
	int ret = 0;

	sm_debug("%s msg type %x\n", __func__, msg->type);
	switch (msg->type) {
	case SM_PACKET_CONSUMED:
	case SM_SESSION_PACKET_COMSUMED:
		kfree((void *)msg->payload);
		break;
	case SM_SESSION_PACKET_CONNECT_ACK:
		sm_debug("%s wakeup wait thread\n", __func__);
		wake_up(&session->rx_wait);
		break;
	case SM_SESSION_PACKET_CONNECT:
		session->remote_ep = msg->src_ep;
		session->flags = SM_CONNECT;
		sm_send_connect_ack(session, msg->src, msg->src);
		break;
	case SM_SESSION_PACKET_CLOSE:
		session->remote_ep = 0;
		session->flags = 0;
		sm_send_close_ack(session, msg->src_ep, msg->src);
	case SM_SESSION_PACKET_CLOSE_ACK:
		session->remote_ep = 0;
		session->flags = 0;
	case SM_PACKET_READY:
		msg_recv_internal(msg, session);
		sm_send_packet_ack(session, msg->src_ep, msg->src,
					msg->payload, msg->length);
		break;
	case SM_SESSION_PACKET_READY:
		msg_recv_internal(msg, session);
		sm_send_session_packet_ack(session, msg->src_ep,
				msg->src, msg->payload, msg->length);
		break;
	case SM_PACKET_ERROR:
		printk("SM ERROR %08x\n", msg->payload);
		break;
	default:
		ret = -EINVAL;
	};

	sm_message_dequeue(0, msg);
	return ret;
}

static int sm_default_shutdown(struct sm_session *session)
{
	return 0;
}

static int sm_default_error(struct sm_message *msg, struct sm_session *session)
{
	return 0;
}

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
	struct sm_message *msg;
	struct sm_session *session;
	sm_uint32_t slot;

	msg = &inqueue->messages[(received % SM_MSGQ_LEN)];

	if (msg->type == SM_BAD_MSG) {
		printk("%s \n", msg->payload);
		sm_message_dequeue(cpu, msg);
		return;
	}

	slot = sm_find_session(msg->dst_ep, 0, icc_info->sessions_table);
	session = &icc_info->sessions_table[slot];

	if ((slot >= 0 && slot < 32)
		&& (SM_MSG_PROTOCOL(msg->type) == session->type)) {
		session->proto_ops->recvmsg(msg, session);
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

	icc_info->iccq_thread = kthread_run(message_queue_thread,
					icc_info->icc_queue, "iccqd");
	return misc_register(&icc_dev);
}
module_init(bf561_icc_test_init);

static void __exit bf561_icc_test_exit(void)
{
	misc_deregister(&icc_dev);
}
module_exit(bf561_icc_test_exit);

MODULE_DESCRIPTION("BF561 ICC"); MODULE_LICENSE("GPL");
