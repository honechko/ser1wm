#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/w1.h>

#ifdef DEBUG
static inline int my_serio_write(struct serio *serio, unsigned char data)
{
	dev_dbg(&serio->dev, "write 0x%02hhx\n", data);
	return serio_write(serio, data);
}
#define serio_write my_serio_write
#endif

struct ser1wm_data {
	struct w1_bus_master	bus_master;
	struct serio		*serio;
	wait_queue_head_t	waitq;
	spinlock_t		lock;
	int			state;
	int			pulse;
	int			cnt;
	u8			*ptr;
};

#define SER1WM_STATE_DESYNC	0
#define SER1WM_STATE_COMMAND	1
#define SER1WM_STATE_DATA	2

#define SER1WM_CMD_DATAMODE	0xe1
#define SER1WM_CMD_CMDMODE	0xe3
#define SER1WM_CMD_RESET	0xc5
#define SER1WM_CMD_CFGPULSE	0x31
#define SER1WM_CMD_SINGLEBIT0	0x85
#define SER1WM_CMD_SINGLEBIT1	0x95
#define SER1WM_CMD_SINGLEBIT0P	0x87
#define SER1WM_CMD_SINGLEBIT1P	0x97
#define SER1WM_CMD_SRCHACCON	0xb5
#define SER1WM_CMD_SRCHACCOFF	0xa5
#define SER1WM_CMD_PULSETERM	0xf1

#define BYTE_DELAY  3
#define RESP_DELAY  1000

static irqreturn_t ser1wm_interrupt(struct serio *serio, unsigned char data,
				    unsigned int fl)
{
	struct ser1wm_data *ser1wm = serio_get_drvdata(serio);
	unsigned long flags;

	dev_dbg(&serio->dev, "read 0x%02hhx\n", data);

	if (ser1wm->serio == NULL)
		return IRQ_NONE;

	spin_lock_irqsave(&ser1wm->lock, flags);
	if (--ser1wm->cnt >= 0 && ser1wm->ptr != NULL)
		*(ser1wm->ptr)++ = data;
	spin_unlock_irqrestore(&ser1wm->lock, flags);
	wake_up(&ser1wm->waitq);

	return IRQ_HANDLED;
}

static int read_resp(struct ser1wm_data *ser1wm, u8 *buf, int cnt, int timeout)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&ser1wm->lock, flags);
	ser1wm->ptr = buf;
	ser1wm->cnt += cnt;
	cnt -= ser1wm->cnt;
	ser1wm->cnt += cnt;
	spin_unlock_irqrestore(&ser1wm->lock, flags);

	rc = wait_event_timeout(ser1wm->waitq,
			ser1wm->cnt <= 0 ||
			ser1wm->serio == NULL ||
			ser1wm->state == SER1WM_STATE_DESYNC,
			msecs_to_jiffies(timeout));

	spin_lock_irqsave(&ser1wm->lock, flags);
	ser1wm->ptr = NULL;
	spin_unlock_irqrestore(&ser1wm->lock, flags);

	if ((rc > 0 && ser1wm->cnt > 0) ||
		ser1wm->serio == NULL ||
		ser1wm->state == SER1WM_STATE_DESYNC)
		rc = -1;
	if (rc <= 0)
		ser1wm->state = SER1WM_STATE_DESYNC;
	return rc;
}

static int get_sync(struct ser1wm_data *ser1wm)
{
	struct serio *serio = ser1wm->serio;
	unsigned long flags;
	int rc;

	if (serio == NULL)
		return 1;
	if (ser1wm->state != SER1WM_STATE_DESYNC)
		return 0;

	dev_dbg(&serio->dev, "Getting sync...");

	serio_write(serio, SER1WM_CMD_CMDMODE);
	serio_write(serio, SER1WM_CMD_SRCHACCOFF);
	serio_write(serio, SER1WM_CMD_CMDMODE);
	serio_write(serio, SER1WM_CMD_SRCHACCOFF);
	serio_write(serio, SER1WM_CMD_PULSETERM);
	ser1wm->state = SER1WM_STATE_COMMAND;

	rc = wait_event_interruptible_timeout(ser1wm->waitq,
			ser1wm->serio == NULL ||
			ser1wm->state == SER1WM_STATE_DESYNC,
			msecs_to_jiffies(4096 + RESP_DELAY));

	spin_lock_irqsave(&ser1wm->lock, flags);
	ser1wm->cnt = 0;
	spin_unlock_irqrestore(&ser1wm->lock, flags);

	if (rc < 0 || ser1wm->serio == NULL ||
		ser1wm->state != SER1WM_STATE_COMMAND) {
		ser1wm->state = SER1WM_STATE_DESYNC;
		return 1;
	}
	return 0;
}

#define command_mode() do{\
		if (ser1wm->state != SER1WM_STATE_COMMAND)\
			serio_write(serio, SER1WM_CMD_CMDMODE);\
		ser1wm->state = SER1WM_STATE_COMMAND;\
	}while(0)

#define data_mode() do{\
		if (ser1wm->state != SER1WM_STATE_DATA)\
			serio_write(serio, SER1WM_CMD_DATAMODE);\
		ser1wm->state = SER1WM_STATE_DATA;\
	}while(0)

static u8 ser1wm_set_pullup(void *data, int duration)
{
	struct ser1wm_data *ser1wm = data;

	ser1wm->pulse = duration;

	return 0;
}

static u8 ser1wm_reset_bus(void *data)
{
	struct ser1wm_data *ser1wm = data;
	struct serio *serio = ser1wm->serio;
	u8 resp;
	int rc;

	if (get_sync(ser1wm))
		return 1;
	command_mode();
	serio_write(serio, SER1WM_CMD_RESET);

	rc = read_resp(ser1wm, &resp, 1, RESP_DELAY + BYTE_DELAY + 4);
	if (rc == 0)
		dev_err(&serio->dev, "Transaction timeout (reset)\n");
	if (rc <= 0)
		return 1;
	if ((resp & 0xdc) != 0xcc) {
		dev_err(&serio->dev, "Transaction error (reset)\n");
		ser1wm->state = SER1WM_STATE_DESYNC;
		return 1;
	}
	if ((resp & 0x03) != 0x01 && (resp & 0x03) != 0x02)
		return 1;
	return 0;
}

static u8 ser1wm_touch_bit(void *data, u8 bit)
{
	struct ser1wm_data *ser1wm = data;
	struct serio *serio = ser1wm->serio;
	u8 resp;
	int rc;
	u8 cmd = bit ? SER1WM_CMD_SINGLEBIT1 : SER1WM_CMD_SINGLEBIT0;

	if (get_sync(ser1wm))
		return 1;
	command_mode();
	serio_write(serio, cmd);

	rc = read_resp(ser1wm, &resp, 1, RESP_DELAY + BYTE_DELAY);
	if (rc == 0)
		dev_err(&serio->dev, "Transaction timeout (bit)\n");
	if (rc <= 0)
		return 1;
	if ((((cmd & 0xfc) ^ ((resp >> 1) & 0x01) ^ resp) & 0xfd) != 0) {
		dev_err(&serio->dev, "Transaction error (bit)\n");
		ser1wm->state = SER1WM_STATE_DESYNC;
		return 1;
	}
	return resp & 1;
}

static void touch_block(void *data, const u8 *buf, u8 *res, int cnt)
{
	struct ser1wm_data *ser1wm = data;
	struct serio *serio = ser1wm->serio;
	int i, rc;

	if (cnt <= 0 || get_sync(ser1wm))
		return;
	data_mode();
	for (i = 0; i < cnt; i++) {
		if (buf[i] == SER1WM_CMD_CMDMODE)
			serio_write(serio, SER1WM_CMD_CMDMODE);
		serio_write(serio, buf[i]);
	}

	rc = read_resp(ser1wm, res, cnt, RESP_DELAY + cnt * BYTE_DELAY);
	if (rc == 0)
		dev_err(&serio->dev, "Transaction timeout (touch:%d)\n", cnt);
}

#ifdef HAVE_W1_TOUCH_BLOCK
static void ser1wm_touch_block(void *data, u8 *buf, int cnt)
{
	touch_block(data, buf, buf, cnt);
}
#endif

static u8 ser1wm_read_block(void *data, u8 *buf, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++)
		buf[i] = 0xff;
	touch_block(data, buf, buf, cnt);
	return cnt;
}

static u8 ser1wm_read_byte(void *data)
{
	u8 resp = 0xff;
	touch_block(data, &resp, &resp, 1);
	return resp;
}

static const int SPUD[] = {16, 65, 131, 262, 524, 1048};

static void ser1wm_write_block(void *data, const u8 *buf, int cnt)
{
	struct ser1wm_data *ser1wm = data;
	struct serio *serio = ser1wm->serio;
	int i, rc, pulse = ser1wm->pulse;
	u8 cmd, res[10];

	if (cnt <= 0)
		return;
	if (pulse != 0)
		cnt--;

	if (cnt != 0) {
		touch_block(data, buf, NULL, cnt);
		if (pulse == 0 || ser1wm->state == SER1WM_STATE_DESYNC)
			return;
	} else {
		if (get_sync(ser1wm))
			return;
	}

	for (i = 0; i < (sizeof(SPUD)/sizeof(*SPUD) - 1); i++)
		if (pulse < SPUD[i])
			break;
	pulse = SPUD[i];
	cmd = SER1WM_CMD_CFGPULSE | (i << 1);

	command_mode();
	serio_write(serio, cmd);
	for (i = 0; i < 7; i++)
		serio_write(serio, (buf[cnt] & (1 << i)) ?
					SER1WM_CMD_SINGLEBIT1 :
					SER1WM_CMD_SINGLEBIT0);
	serio_write(serio, (buf[cnt] & (1 << 7)) ?
				SER1WM_CMD_SINGLEBIT1P :
				SER1WM_CMD_SINGLEBIT0P);

	rc = read_resp(ser1wm, res, 10, RESP_DELAY + 10 * BYTE_DELAY + pulse);
	if (rc == 0)
		dev_err(&serio->dev, "Transaction timeout (pulse)\n");
	if (rc <= 0)
		return;
	for (i = 0; i < 8; i++)
		if ((((((buf[cnt] & (1 << i)) ? SER1WM_CMD_SINGLEBIT1 :
						SER1WM_CMD_SINGLEBIT0) & 0xfc) ^
			((res[i + 1] >> 1) & 0x01) ^ res[i + 1]) & 0xfd) != 0)
			break;
	if ((cmd ^ res[0]) != 0x01 || i < 8 ||
			((res[8] & 0x03) | 0xec) != res[9]) {
		dev_err(&serio->dev, "Transaction error (pulse)\n");
		ser1wm->state = SER1WM_STATE_DESYNC;
	}
}

static void ser1wm_write_byte(void *data, u8 byte)
{
	ser1wm_write_block(data, &byte, 1);
}

static inline void id2searchbuf(u64 id, u8 sbuf[16])
{
	const u8 *idbuf = (u8 *)&id;
	int i;
	for (i = 63; i >= 0; i--) {
		int bitm = 1 << (i & 7);
		sbuf[i >> 2] <<= 2;
		if (idbuf[i >> 3] & bitm)
			sbuf[i >> 2] |= 2;
	}
}
static inline u64 searchbuf2id(const u8 sbuf[16])
{
	u8 idbuf[8];
	int i;
	for (i = 63; i >= 0; i--) {
		int bitm = 2 << ((i & 3) << 1);
		idbuf[i >> 3] <<= 1;
		if (sbuf[i >> 2] & bitm)
		    idbuf[i >> 3] |= 1;
	}
	return *(u64 *)idbuf;
}

static void ser1wm_search(void *data, struct w1_master *dev,
			  u8 search_type, w1_slave_found_callback slave_cb)
{
	struct ser1wm_data *ser1wm = data;
	struct serio *serio = ser1wm->serio;
	int rc, i, slave_count = 0;
	u8 resbuf[17], *searchbuf = &resbuf[1];
	u8 romfunc = (search_type == W1_SEARCH) ? 0xf0 : 0xec;

	id2searchbuf(dev->search_id, searchbuf);
	for (;;) {

		mutex_lock(&dev->bus_mutex);

#ifdef HAVE_W1_RESET_COUNTER
		dev->reset_counter++;
#endif
		if (ser1wm_reset_bus(data))
			break;

		serio_write(serio, SER1WM_CMD_DATAMODE);
		serio_write(serio, romfunc);
		serio_write(serio, SER1WM_CMD_CMDMODE);
		serio_write(serio, SER1WM_CMD_SRCHACCON);
		serio_write(serio, SER1WM_CMD_DATAMODE);
		for (i = 0; i < 16; i++)
			serio_write(serio, searchbuf[i] & 0xaa);
		serio_write(serio, SER1WM_CMD_CMDMODE);
		serio_write(serio, SER1WM_CMD_SRCHACCOFF);

		rc = read_resp(ser1wm, resbuf, 17, RESP_DELAY + 23 * BYTE_DELAY);
		if (rc == 0)
			dev_err(&serio->dev, "Transaction timeout (search)\n");
		if (rc <= 0)
			break;
		if (romfunc != resbuf[0]) {
			dev_err(&serio->dev, "Transaction error (search)\n");
			ser1wm->state = SER1WM_STATE_DESYNC;
			break;
		}

		if (test_bit(W1_ABORT_SEARCH, &dev->flags)) {
			dev_dbg(&dev->dev, "Abort ser1wm_search\n");
			break;
		}

		mutex_unlock(&dev->bus_mutex);

		if ((searchbuf[15] & 0xc0) != 0xc0) {
			slave_cb(dev, searchbuf2id(searchbuf));
			slave_count++;
		}

		for (i = 63; i >= 0; i--) {
			int  mask = 3 << ((i & 3) << 1);
			int discr = 1 << ((i & 3) << 1);
			if ((searchbuf[i >> 2] & mask) != discr) {
				searchbuf[i >> 2] &= ~mask;
			} else {
				searchbuf[i >> 2] |=  mask;
				break;
			}
		}
		dev->search_id = searchbuf2id(searchbuf);
		if (i < 0)
			return;

		if (slave_count >= dev->max_slave_count) {
			if (!test_bit(W1_WARN_MAX_COUNT, &dev->flags)) {
				dev_info(&dev->dev, "%s: max_slave_count %d reached, "
					"will continue next search.\n", __func__,
					dev->max_slave_count);
				set_bit(W1_WARN_MAX_COUNT, &dev->flags);
			}
			return;
		}
	}

	mutex_unlock(&dev->bus_mutex);
}

static int ser1wm_connect(struct serio *serio, struct serio_driver *drv)
{
	struct ser1wm_data *ser1wm;
	int err;

	ser1wm = kzalloc(sizeof(struct ser1wm_data), GFP_KERNEL);
	if (ser1wm == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	init_waitqueue_head(&ser1wm->waitq);
	spin_lock_init(&ser1wm->lock);
	ser1wm->state = SER1WM_STATE_COMMAND;
	ser1wm->bus_master.data = ser1wm;
	ser1wm->bus_master.set_pullup  = &ser1wm_set_pullup;
	ser1wm->bus_master.reset_bus   = &ser1wm_reset_bus;
	ser1wm->bus_master.touch_bit   = &ser1wm_touch_bit;
#ifdef HAVE_W1_TOUCH_BLOCK
	ser1wm->bus_master.touch_block = &ser1wm_touch_block;
#endif
	ser1wm->bus_master.read_block  = &ser1wm_read_block;
	ser1wm->bus_master.read_byte   = &ser1wm_read_byte;
	ser1wm->bus_master.write_block = &ser1wm_write_block;
	ser1wm->bus_master.write_byte  = &ser1wm_write_byte;
	ser1wm->bus_master.search      = &ser1wm_search;
	ser1wm->serio = serio;
	serio_set_drvdata(serio, ser1wm);

	err = serio_open(serio, drv);
	if (err)
		goto exit_kfree;

	err = w1_add_master_device(&ser1wm->bus_master);
	if (err)
		goto exit_close;

	dev_info(&serio->dev, "attaching 1-wire bus master\n");
	return 0;

exit_close:
	serio_close(serio);
exit_kfree:
	kfree(ser1wm);
exit:
	return err;
}

static int ser1wm_reconnect(struct serio *serio)
{
	struct ser1wm_data *ser1wm = serio_get_drvdata(serio);

	dev_info(&serio->dev, "reattaching 1-wire bus master\n");

	ser1wm->state = SER1WM_STATE_DESYNC;
	wake_up(&ser1wm->waitq);
	return 0;
}

static void ser1wm_disconnect(struct serio *serio)
{
	struct ser1wm_data *ser1wm = serio_get_drvdata(serio);

	dev_info(&serio->dev, "detaching 1-wire bus master\n");

	ser1wm->serio = NULL;
	wake_up(&ser1wm->waitq);

	w1_remove_master_device(&ser1wm->bus_master);

	serio_close(serio);
	kfree(ser1wm);
}

static const struct serio_device_id ser1wm_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_SUNKBD,
		.id	= 0x55,
		.extra	= 0xaa,
	},
	{ 0 }
};
MODULE_DEVICE_TABLE(serio, ser1wm_serio_ids);

static struct serio_driver ser1wm_driver = {
	.driver		= {
		.name	= "ser1wm",
	},
	.description	= "serial 1-wire bus master",
	.id_table	= ser1wm_serio_ids,
	.connect	= ser1wm_connect,
	.reconnect	= ser1wm_reconnect,
	.fast_reconnect	= ser1wm_reconnect,
	.disconnect	= ser1wm_disconnect,
	.interrupt	= ser1wm_interrupt,
};

module_serio_driver(ser1wm_driver);

MODULE_AUTHOR("honechko <kernel@honey.com.ua>");
MODULE_DESCRIPTION("serial 1-wire bus master driver");
MODULE_LICENSE("GPL v2");
