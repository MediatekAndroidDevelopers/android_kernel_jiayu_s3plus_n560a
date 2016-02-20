#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
//#include <mach/gpio.h>
#include <asm/ioctls.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h> 
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
#include "abov_remote_ctrl.h"
//#include <mach/rpm-regulator.h>
//#include <mach/rpm-regulator-smd.h>
#include <linux/platform_device.h>

#define REMOTE_DEVNAME            "remoteplatformdev"
#define REMOTE_I2C_ADDR 0xA0

#define     USE_PART_WRITE              0

#define     MAX_IRBUF_SIZE              2048
#define REMOTE_CHIP_I2C_BUSNUM          3
#define GPIO_IR_EINT_PIN               (GPIO9 | 0x80000000)
#define GPIO_REMOTEIR_RESET_PIN     (GPIO3 | 0x80000000)
#define GPIO_VDDIR_EN               (GPIO4 | 0x80000000)

#define REM_TAG                  "[Remote_ctl] "
#define REM_FUN(f)               printk(KERN_ERR REM_TAG"%s\n",   __FUNCTION__)
#define REM_LOG(fmt, args...)    printk(KERN_ERR REM_TAG fmt,     ##args)
#define REM_ERR(fmt, args...)    printk(KERN_ERR REM_TAG"%s %d :  "fmt, __FUNCTION__, __LINE__, ##args)

#define REMOTE_DEVICE "remote_ctrl"


static const struct i2c_device_id bl_id[] = { {REMOTE_DEVICE, 0}, {} };
static struct i2c_board_info __initdata remote_ctrl_i2c_info = { I2C_BOARD_INFO(REMOTE_DEVICE, (REMOTE_I2C_ADDR >> 1)) };
struct platform_device *pdev= NULL;

static struct remote_ctrl_data *gDeviceData  = NULL;
// xuxiaojiang
static u8 *CTPI2CDMABuf_va = NULL;
static dma_addr_t *CTPI2CDMABuf_pa = 0;

static DEFINE_MUTEX(rx_lock);
static DEFINE_MUTEX(tx_lock);

static int _i2c_gpio_direction = RC_GPIO_MODE_OUTPUT;
static int _i2c_gpio_value      = 0;


static unsigned char _i2c_buf[MAX_IRBUF_SIZE];

static void remote_ctrl_checkdata( const unsigned char * buf, int bufsize ) __attribute__ ((unused));

static ssize_t remote_ctrl_part_write(struct file *file,
    const char __user *buf,
    size_t count,
    loff_t *ppos) __attribute__ ((unused));


static ssize_t remote_ctrl_write(struct file *file,
    const char __user *buf,
    size_t count,
    loff_t *ppos) __attribute__ ((unused));

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define MAX_PART_SIZE 250
static int CTPDMA_i2c_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len);

static int CTPDMA_i2c_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
	int i = 0;
	for(i = 0 ; i < dw_len; i++)
	{
		CTPI2CDMABuf_va[i] = pbt_buf[i];
	}

	if(dw_len <= 8)
	{
		mutex_lock(&rx_lock);
		gDeviceData->client->addr = gDeviceData->client->addr & I2C_MASK_FLAG;
		mutex_unlock(&rx_lock);
		return i2c_master_send(gDeviceData->client, pbt_buf, dw_len);
	}
	else
	{
		mutex_lock(&rx_lock);
		gDeviceData->client->addr = gDeviceData->client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		mutex_unlock(&rx_lock);
		return i2c_master_send(gDeviceData->client, CTPI2CDMABuf_pa, dw_len);
	}    
}

static int CTPDMA_i2c_read(FTS_BYTE *buf, FTS_DWRD len)
{
	int i = 0, err = 0;
	int num_of_read = 0, read_num = 0; 
	int count = len;

	if(len < 8)
	{
		mutex_lock(&rx_lock);
		gDeviceData->client->addr = gDeviceData->client->addr & I2C_MASK_FLAG;
		mutex_unlock(&rx_lock);
		
		return i2c_master_recv(gDeviceData->client, buf, len);
	}
	else
	{
		mutex_lock(&rx_lock);
		gDeviceData->client->addr = gDeviceData->client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		mutex_unlock(&rx_lock);

		while(count > 0)
		{
			num_of_read = (count > 255) ? 255 : count;
			err = i2c_master_recv(gDeviceData->client, CTPI2CDMABuf_pa + read_num, num_of_read); 
		
			if(err < 0)
			{
				REM_LOG("read error!! \n");
				return err;
			}
			
			read_num += num_of_read;
			count -= num_of_read;
		}
		
		for(i = 0; i < len; i++)
		{
			buf[i] = CTPI2CDMABuf_va[i];
			REM_LOG("remote_ctrl_read!! buf[%d]=0x%x \n", i, buf[i]);
		}
	}

	return err;
}


/* ------------------------------------------------------------------------------------------- */
/*
int remote_ctrl_enable_lvs1() 
{
    struct rpm_regulator *vreg;

	vreg = rpm_regulator_get( &pdata->dev, "8941_lvs1");
	if( vreg ) {
		REM_LOG("[remote_ctrl] enabling 8941_lvs1...\n" );
		rpm_regulator_enable(vreg);
	}
	
	return 0;
}
*/

	
static int remote_ctrl_probe(struct i2c_client *client,const struct i2c_device_id *id )
{
    struct remote_ctrl_data *deviceData = NULL;
    

	REM_LOG("[remote_ctrl] %s  addr=%d\n", __func__, client->addr);
    mt_set_gpio_pull_enable(GPIO_VDDIR_EN, GPIO_PULL_DISABLE);
    mt_set_gpio_mode(GPIO_VDDIR_EN, GPIO_MODE_GPIO);
    mt_set_gpio_dir(GPIO_VDDIR_EN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_VDDIR_EN,GPIO_OUT_ONE);

	if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)) 
	{
		REM_ERR("i2c_check_functionality failed.");
		return -ENODEV;
	}

	deviceData = kzalloc(sizeof(struct remote_ctrl_data),GFP_KERNEL);
    if (!deviceData) {
        return -ENOMEM;
    }

	deviceData->client 		= client;
	deviceData->init_gpio	= GPIO_IR_EINT_PIN;
	deviceData->reset_gpio	= GPIO_REMOTEIR_RESET_PIN;


	gDeviceData = deviceData;
//	GPIO_IR_EINT_PIN
	
	//if(!hwPowerOn(MT6325_POWER_LDO_VCN33, VOL_3300, "RemoteIr"))
	//{
	//	REM_LOG("Error:power on fails!!\n");
	//	return -2;
	//}


	if( deviceData->init_gpio ) {
		mt_set_gpio_mode(GPIO_IR_EINT_PIN, 0);
		mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_IR_EINT_PIN,GPIO_OUT_ONE);
		_i2c_gpio_direction = RC_GPIO_MODE_OUTPUT;
		_i2c_gpio_value	 	= 1;
	}

	if( deviceData->reset_gpio ) {
		mt_set_gpio_mode(GPIO_REMOTEIR_RESET_PIN,GPIO_MODE_GPIO);
		mt_set_gpio_dir(GPIO_REMOTEIR_RESET_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_REMOTEIR_RESET_PIN, GPIO_OUT_ZERO);
		msleep(10);
		mt_set_gpio_out(GPIO_REMOTEIR_RESET_PIN, GPIO_OUT_ONE);
	}
/*    1477   i2c->dma_buf.vaddr =
    1478         dma_alloc_coherent(&pdev->dev, MAX_DMA_TRANS_NUM,
    1479               &i2c->dma_buf.paddr, GFP_KERNEL|GFP_DMA32);
*/
		
	// xuxiaojiang
	CTPI2CDMABuf_va = dma_alloc_coherent(&pdev->dev, 512, &CTPI2CDMABuf_pa, GFP_KERNEL);//|GFP_DMA32);
	if(!CTPI2CDMABuf_va)
	{
    		REM_LOG("[remote_ctrl] dma_alloc_coherent error\n");
	}
		
	REM_LOG("[remote_ctrl] %s done. init_gpio=%d\n", __func__, deviceData->init_gpio );
	
	//i2c_smbus_write_i2c_block_data(gDeviceData->client, 0xb, 4,test );
	return 0;
}


	

static int  remote_ctrl_remove(struct i2c_client *client)
{
	REM_FUN();

	 if(CTPI2CDMABuf_va)
	{
		dma_free_coherent(NULL, 512, CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		CTPI2CDMABuf_va = NULL;
		CTPI2CDMABuf_pa = 0;
	}

	//xuxiaojiang
	mt_set_gpio_mode(GPIO_IR_EINT_PIN, 0);
	mt_set_gpio_out(GPIO_IR_EINT_PIN,GPIO_OUT_ZERO);
	mt_set_gpio_pull_enable(GPIO_IR_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_IR_EINT_PIN, GPIO_PULL_UP);
	mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_IN);
        
	return 0;
}

static int remote_ctrl_open(struct inode *inode, struct file *file)
{
	REM_FUN();
	return 0;
}

static int remote_ctrl_release(struct inode *inode, struct file *file)
{
	REM_FUN();
	return 0;	
}

// xuxiaojiang

static ssize_t remote_ctrl_read(struct file *file, char __user *buf,
			 size_t count, loff_t *pos)
{
	int32_t rc = 0;

	struct i2c_msg msgs[] = {
		/*
		{
			.addr  = REMOTE_CTRL_SLAVE_ADDR,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		},
		*/
		{
			.addr  = REMOTE_CTRL_SLAVE_ADDR,
			.flags = I2C_M_RD,
			.len   = count,
			.buf   = buf,
		},
	};
	//rc = i2c_transfer(gDeviceData->client->adapter, msgs, 1);
	rc = CTPDMA_i2c_read(buf,count);
	if (rc < 0) {
		REM_LOG("remote_ctrl_i2c_rxdata failed:%d\n", rc);
	}
	else
	{
		REM_LOG("read success:%d", rc);
		}
	
	return rc;
}



static void remote_ctrl_checkdata( const unsigned char * buf, int bufsize )
{
	int i, pktno, len, chksum, chksum_calc;

	if( bufsize >= 255 ) {
		REM_LOG("[remote_ir] packet_type: RAW (total size:%d :exceed 255 bytes.)\n", bufsize );
		return;
	}
	
		
	pktno = buf[0];
	len   = buf[1];		// packet length except packet no and length field.
	
	chksum  = buf[ bufsize - 2 ]; chksum <<= 8;
	chksum |= buf[ bufsize - 1 ];
	
	chksum_calc = 0;
	for(i = 0; i < bufsize-2 ; i++) {
		chksum_calc += buf[i];
	}
	
	if( (len+2) != bufsize ) {
		REM_LOG("[remote_ir] packet_type: RAW (length mismatch) (pktno:%d len:%d chksum:%d total_size:%d)\n", pktno, len, chksum, bufsize );
	}
	else if(  chksum != chksum_calc) {
		REM_LOG("[remote_ir] packet_type: RAW (checksum mismatch) (pktno:%d len:%d chksum:%d/%d total_size:%d)\n", pktno, len, chksum, chksum_calc, bufsize );
	}
	else {
		REM_LOG("[remote_ir] packet_type: SEQ_PKT (pktno:%d len:%d chksum:%d/%d total_size:%d)\n", pktno, len, chksum, chksum_calc, bufsize );
	}

}


#define MAX_PART_SIZE 250
static ssize_t remote_ctrl_part_write(struct file *file,
				const char __user *buf,
				size_t count,
				loff_t *ppos)
{
	int rc=0, i, written, num_to_write, chksum;
	int pktNo;
	unsigned char _tmpbuf[MAX_PART_SIZE + 4]; 	// add 4 for pktNo, Length, checksum16

	//REM_LOG( "[remote_ir] %s", __func__ );
	
	if( gDeviceData == NULL ) return 0;
		

	if( gDeviceData->init_gpio ) {
		
		rc = gpio_get_value(gDeviceData->init_gpio);
		REM_LOG( "[remote_ir] current gpio value=%d/%d", _i2c_gpio_value, rc );
		
		//if( (_i2c_gpio_direction != RC_GPIO_MODE_OUTPUT) || (rc != 1) ) 
		{
			mt_set_gpio_mode(GPIO_IR_EINT_PIN, 0);
			mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_IR_EINT_PIN,0);	//xuxiaojiang

			msleep(10);
			mt_set_gpio_out(GPIO_IR_EINT_PIN,1);	//xuxiaojiang
			msleep(10);
			
			_i2c_gpio_direction = RC_GPIO_MODE_OUTPUT;
			_i2c_gpio_value		= 1;
		}
	}

	
	mutex_lock(&tx_lock);

	if( count > 2048 ) count = 2048;
	rc = copy_from_user(_i2c_buf, buf, count);
	if (rc != 0) {
		REM_LOG("copy_from_user failed. (rc=%d)", rc);
		mutex_unlock(&tx_lock);
		return 0;
	}
		

	rc = 0;
	num_to_write = 0;
	written = 0;
	pktNo = 0;
	
	while( count > 0 ) 
	{
		num_to_write = (count > MAX_PART_SIZE) ? MAX_PART_SIZE : count;
			
		// build unit packet
		_tmpbuf[0] = pktNo;
		_tmpbuf[1] = (unsigned char)(num_to_write + 2);			// packet length except packet no and length field.
		memcpy( &_tmpbuf[2], &_i2c_buf[written], num_to_write );
		
		chksum = pktNo;
		for(i=1; i < (num_to_write+2); i++) {
			chksum += (int)_tmpbuf[i];
		}
		_tmpbuf[num_to_write + 2] = (unsigned char)((chksum >> 8) & 0xff);
		_tmpbuf[num_to_write + 3] = (unsigned char)(chksum & 0xff);
		
		//rc += i2c_master_send(gDeviceData->client,	&_i2c_buf[written],	num_to_write);
		//rc += i2c_master_send(gDeviceData->client, _tmpbuf, num_to_write+4);
		rc  = CTPDMA_i2c_write(_tmpbuf, num_to_write+4);
		REM_LOG("tran loops write data ret:%d\n", rc);
		if(rc<0)
		{
			rc  = CTPDMA_i2c_write(_tmpbuf, num_to_write+4);
			REM_LOG("write data ret:%d\n", rc);
			if(rc<0)
				return -2;
		}
		count -= num_to_write;
		written += num_to_write;
		pktNo ++;

		if( count > 0 ) {
			//msleep(1);
		}
		
	}
	
	// check data length of last packet
	if( num_to_write == 250 ) {
		
		//msleep(1);

		_tmpbuf[0] = pktNo;	// packet No.
		_tmpbuf[1] = 2;		// Length
		_tmpbuf[2] = 0;		// checksum
		_tmpbuf[3] = (unsigned char)(pktNo + 2);
		
		//rc += i2c_master_send(gDeviceData->client,	&_i2c_buf[written],	num_to_write);
		//i2c_master_send(gDeviceData->client, _tmpbuf, 4);
		rc = CTPDMA_i2c_write(_tmpbuf, 4);
	}
	
	mutex_unlock(&tx_lock);

	return rc;
}
// ----------



static ssize_t remote_ctrl_write(struct file *file,
				const char __user *buf,
				size_t count,
				loff_t *ppos)
{
	int rc=0;
	
	REM_FUN();
	if( gDeviceData == NULL ) {
		REM_LOG( "[remote_ir] gDeviceData is NULL!" );
		return 0;
	}
	REM_LOG("remote_ctrl_write. gDeviceData gpio=%d \n", gDeviceData->init_gpio);

	if( gDeviceData->init_gpio ) {
		//gpio_direction_output(gDeviceData->init_gpio, 0);
		mt_set_gpio_mode(GPIO_IR_EINT_PIN, 0);
		mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_IR_EINT_PIN,0);  //xuxiaojiang
		msleep(10);
		//gpio_set_value(gDeviceData->init_gpio, 1);
		mt_set_gpio_mode(GPIO_IR_EINT_PIN, 0);
		mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_IR_EINT_PIN,1);  //xuxiaojiang
		msleep(10);
		_i2c_gpio_direction = RC_GPIO_MODE_OUTPUT;
		_i2c_gpio_value		= 1;
	}

	mutex_lock(&tx_lock);

	if( count > MAX_IRBUF_SIZE ) {
		count = MAX_IRBUF_SIZE;
	}

	
	rc = copy_from_user(_i2c_buf, buf, count);
	if (rc != 0) {
		REM_LOG("copy_from_user! failed. (rc=%d)", rc);
		mutex_unlock(&tx_lock);
		return 0;
	}
		

	//rc = i2c_master_send(gDeviceData->client, _i2c_buf,	count);
	rc= CTPDMA_i2c_write(_i2c_buf, count);
	REM_LOG(" write data ret:%d\n", rc);
	mutex_unlock(&tx_lock);

	return rc;
}
// ----------



static ssize_t remote_ctrl_read_msg (void *i2cMsg)
{
	struct i2c_msg *msg = (struct i2c_msg *)i2cMsg;
	int rc;
	int count;

	struct i2c_msg msgs[] = {
		{
			.addr  = REMOTE_CTRL_SLAVE_ADDR,
			.flags = 0,
			.len   = 2,
			.buf   = _i2c_buf,
		},
		{
			.addr  = REMOTE_CTRL_SLAVE_ADDR,
			.flags = I2C_M_RD,
			.len   = 0,
			.buf   = _i2c_buf,
		},
	};


	if( gDeviceData == NULL || msg == NULL|| msg->buf == NULL ) {
		return -1;
	}

	count = msg->len;
	if( count > MAX_IRBUF_SIZE ) {
		count = MAX_IRBUF_SIZE;
	}

	msgs[1].len = count;


	//mutex_lock(&tx_lock);
	

	rc = copy_from_user(_i2c_buf, msg->buf, count);
	
	//rc = CTPDMA_i2c_write(_i2c_buf, 2);
	//rc = i2c_transfer(gDeviceData->client->adapter, msgs, 2);
	rc = CTPDMA_i2c_write(_i2c_buf, 2);
    printk("remote_dev write:rc= %d\n",rc);
	if (rc < 0) {
		REM_LOG("write_msg: i2c_transfer failed 0x%x\n", rc);
		return rc;
	}
	rc = CTPDMA_i2c_read(_i2c_buf, count);
    printk("remote_dev read:rc= %d\n",rc);
	if (rc < 0) {
		REM_LOG("read_msg: i2c_transfer failed 0x%x\n", rc);
	}
	else {
		rc = copy_to_user(msg->buf, _i2c_buf, count) ? -EFAULT : rc;
	}
	
	//mutex_unlock(&tx_lock);
	
	//rc = CTPDMA_i2c_read(buf, count);
	
	return rc;
}



static ssize_t remote_ctrl_write_msg (void *i2cMsg)
{
	struct i2c_msg *msg = (struct i2c_msg *)i2cMsg;
	int rc;
	int count;
	char *buf;
	
	if( gDeviceData == NULL || msg == NULL|| msg->buf == NULL ) {
		return -1;
	}
	
	count = msg->len;
	buf = msg->buf;
	
		
	///mutex_lock(&tx_lock);

	if( count > MAX_IRBUF_SIZE ) {
		count = MAX_IRBUF_SIZE;
	}

	
	rc = copy_from_user(_i2c_buf, buf, count);
	if (rc != 0) {
		REM_LOG("copy_from_user failed. (rc=%d)", rc);
		mutex_unlock(&tx_lock);
		return 0;
	}

	//rc = i2c_master_send(gDeviceData->client, _i2c_buf,	count);
	rc = CTPDMA_i2c_write(_i2c_buf, count);
    printk("remote_dev:ctpdam_i2c_write rc%d\n",rc);
	REM_LOG("write data ret:%d\n", rc);
	//mutex_unlock(&tx_lock);

	return rc;
}

/*
 * serport_ldisc_ioctl() allows to set the port protocol, and device ID
 */

static int remote_ctrl_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
	unsigned long type;
	int err = 0;

	REM_LOG("remote_ctrl_ioctl. cmd=0x%x, arg=%ld\n", cmd, arg);  
	
	if( gDeviceData == NULL || !gDeviceData->init_gpio ) 
	{
		return -EINVAL;
	}
	
	switch(cmd)
	{
	case REMOTE_CTRL_CTL_GET_GPIO_MODE:
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_CTL_GET_GPIO_MODE");
		err = _i2c_gpio_direction;	// 0:input 1:output mode
		break;

	case REMOTE_CTRL_CTL_SET_GPIO_MODE:

		if( arg == RC_GPIO_MODE_INPUT )
		{
			mt_set_gpio_pull_enable(GPIO_IR_EINT_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_IR_EINT_PIN, GPIO_PULL_UP);
			mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_IN);
			_i2c_gpio_direction = RC_GPIO_MODE_INPUT;
			
			err = mt_get_gpio_in(GPIO_IR_EINT_PIN);
			//REM_LOG("remote_ctrl_ioctl---REMOTE_CTRL_CTL_SET_GPIO_MODE-----GPIO_IR_EINT_PIN---RC_GPIO_MODE_INPUT err=%d \n",err);
		}
		else if( arg == RC_GPIO_MODE_OUTPUT )
		{
			mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_IR_EINT_PIN, 1);
			_i2c_gpio_direction = RC_GPIO_MODE_OUTPUT;
			
			err = 0;
		}
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_CTL_SET_GPIO_MODE");
		break;
	
	case REMOTE_CTRL_CTL_GET_GPIO_VALUE:
		err = mt_get_gpio_in(GPIO_IR_EINT_PIN);
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_CTL_GET_GPIO_VALUE,err=%d", err);
		_i2c_gpio_value = err;
		break;

	case REMOTE_CTRL_CTL_SET_GPIO_VALUE:
		mt_set_gpio_out(GPIO_IR_EINT_PIN, (int)arg);
		_i2c_gpio_value = (int)arg;
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_CTL_SET_GPIO_VALUE,_i2c_gpio_value=%d\n", _i2c_gpio_value);
		err = 0;
		break;
	case REMOTE_CTRL_CTL_SET_RESET_VALUE:
		if( gDeviceData->reset_gpio ) {
			int value = ((int)arg) ? GPIO_OUT_ONE: GPIO_OUT_ZERO;
			mt_set_gpio_mode(GPIO_REMOTEIR_RESET_PIN,GPIO_MODE_GPIO);
			mt_set_gpio_dir(GPIO_REMOTEIR_RESET_PIN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_REMOTEIR_RESET_PIN, value);
		}
		else {
			err = -1;
		}
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_CTL_SET_GPIO_VALUE,GPIO_REMOTEIR_RESET_PIN=%d\n", mt_get_gpio_in(GPIO_REMOTEIR_RESET_PIN));
		break;
	case REMOTE_CTRL_WRITE:
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_WRITE\n");
		err = remote_ctrl_write_msg( (void *)arg );
		break;

	case REMOTE_CTRL_READ:
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_READ\n");
		err = remote_ctrl_read_msg( (void *)arg );
		break;
	case REMOTE_CTRL_CTL_MODE_BOOTLOADER:
		REM_LOG("remote_ctrl_ioctl--------REMOTE_CTRL_CTL_MODE_BOOTLOADER\n");
		/* bootloader mode at P00/WAKE HIGH */
		if( gDeviceData->reset_gpio && gDeviceData->init_gpio ) {
			
			/* make P00/WAKE pin to high */
		if(_i2c_gpio_direction != RC_GPIO_MODE_OUTPUT ) {
			mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_IR_EINT_PIN, 1);
		}
		mt_set_gpio_out(GPIO_IR_EINT_PIN, 1);
		msleep(1);
		/* RESET (Active LOW) */
		mt_set_gpio_mode(GPIO_REMOTEIR_RESET_PIN,GPIO_MODE_GPIO);
		mt_set_gpio_dir(GPIO_REMOTEIR_RESET_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_REMOTEIR_RESET_PIN, GPIO_OUT_ZERO);

		msleep(10); 	/* reset input should be asserted low at least for 8us(typically) for normal reset function */

		mt_set_gpio_out(GPIO_REMOTEIR_RESET_PIN, GPIO_OUT_ONE);
		/* When the external reset input goes high, the internal reset is released after 64ms of 
		   stability time in case external clock frequency is 8MHz.
		*/
		msleep(64);
			
			
		/* Restore P00/WAKE pin status */
		if(_i2c_gpio_direction != RC_GPIO_MODE_OUTPUT ) 
		{
			mt_set_gpio_pull_enable(GPIO_IR_EINT_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_IR_EINT_PIN, GPIO_PULL_UP);
			mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_IN);
		}
		else 
		{
			mt_set_gpio_out(GPIO_IR_EINT_PIN,_i2c_gpio_value);
		}
			
		err = 0;
		}
		else {
			err = -1;
		}
		break;

	case REMOTE_CTRL_CTL_MODE_USERIR:
		REM_LOG("remote_ctrl_ioctl-------REMOTE_CTRL_CTL_MODE_USERIR\n");
		/* user IR mode at P00/WAKE LOW */
		if( gDeviceData->reset_gpio && gDeviceData->init_gpio ) {
			
		/* make P00/WAKE pin to low */
		if(_i2c_gpio_direction != RC_GPIO_MODE_OUTPUT ) {
			mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_IR_EINT_PIN, 0);
		}
		mt_set_gpio_out(GPIO_IR_EINT_PIN, 0);
		msleep(1);

		/* RESET (Active LOW) */
		mt_set_gpio_mode(GPIO_REMOTEIR_RESET_PIN,GPIO_MODE_GPIO);
		mt_set_gpio_dir(GPIO_REMOTEIR_RESET_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_REMOTEIR_RESET_PIN, GPIO_OUT_ZERO);

		msleep(10); 	/* reset input should be asserted low at least for 8us(typically) for normal reset function */

		mt_set_gpio_out(GPIO_REMOTEIR_RESET_PIN, GPIO_OUT_ONE);
		/* When the external reset input goes high, the internal reset is released after 64ms of 
		   stability time in case external clock frequency is 8MHz.
		*/
		msleep(64);
			
		if(_i2c_gpio_direction != RC_GPIO_MODE_OUTPUT ) 
		{
			mt_set_gpio_pull_enable(GPIO_IR_EINT_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_IR_EINT_PIN, GPIO_PULL_UP);
			mt_set_gpio_dir(GPIO_IR_EINT_PIN,GPIO_DIR_IN);
		}
		else 
		{
			mt_set_gpio_out(GPIO_IR_EINT_PIN,_i2c_gpio_value);
		}
			
		err = 0;
		}
		else {
			err = -1;
		}
		break;
	}
	
	REM_LOG("remote_ctrl_ioctl. err=0x%x\n", err);  

	return err;
}
static struct file_operations remote_ctrl_fops = {
	.owner		= THIS_MODULE,
	.open			= remote_ctrl_open,
	.release	= remote_ctrl_release,
	.read 		= remote_ctrl_read,
#if USE_PART_WRITE
	.write 			= remote_ctrl_part_write,
#else	
	.write 			= remote_ctrl_write,
#endif //USE_PART_WRITE	
	.unlocked_ioctl		= remote_ctrl_ioctl,
};

static struct miscdevice remote_ctrl_miscdev = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "remote_ctrl_dev",
	.fops 	= &remote_ctrl_fops
};


static int remote_ctrl_misc_device_init(void)
{
    int result=0;
	REM_LOG("[remote_ctrl] %s \n", __func__);
    result = misc_register(&remote_ctrl_miscdev);
    return result;
}

static int remote_ctrl_misc_device_deinit(void)
{
    int result=0;
	REM_FUN();
    result = misc_deregister(&remote_ctrl_miscdev);
    return result;
}

static int remote_ctrl_suspend(struct device *dev)
{
	REM_FUN();

	return 0;
}

static int remote_ctrl_resume(struct device *dev)
{
	REM_FUN();

	return 0;
}
static const struct i2c_device_id RC_ID[] ={{REMOTE_CTRL_DEV,     0}, {}};


//MODULE_DEVICE_TABLE(i2c, RC_ID);

//const static struct dev_pm_ops i2c_device_rc_pm_ops = {
//  .suspend = remote_ctrl_suspend,
//  .resume = remote_ctrl_resume,
//};

static struct of_device_id rc_match_table[] = {
    { .compatible = "mediatek,remote-ctrl",},
    { },
};


static struct i2c_driver rc_i2c_driver =
{
    .driver = {
        .name   = REMOTE_CTRL_DEV,
    },
    .probe      = remote_ctrl_probe,
    .remove     = remote_ctrl_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND) || !                        defined(USE_EARLY_SUSPEND)
    .suspend    = remote_ctrl_suspend,
    .resume     = remote_ctrl_resume,
#endif
    .id_table   = RC_ID,
};


static int remote_platform_probe(struct platform_device *dev)
{
    int rc;
    REM_FUN();
    pdev = dev;
    rc = i2c_add_driver(&rc_i2c_driver);
    REM_LOG("[remote_ctrl] %s---- i2c_add_driver return: %d\n", __func__,       rc);

    remote_ctrl_misc_device_init();

	return 0;
}

static int remote_platform_remove(struct platform_device *dev)
{

    i2c_del_driver(&rc_i2c_driver);
    remote_ctrl_misc_device_deinit();
	return 0;
}


static struct platform_driver remote_platform_driver = {
	.probe = remote_platform_probe,
	.remove = remote_platform_remove,
	.driver = {
		   .name = REMOTE_DEVNAME,
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device remote_platform_device = {
	.name = REMOTE_DEVNAME,	/* FIXME. Sync to MHAL and init.rc */
	.id = -1,
};

static int __init remote_init(void)
{
	int ret = 0;
	REM_LOG("[remote_ctrl] %s \n", __func__);

	i2c_register_board_info(REMOTE_CHIP_I2C_BUSNUM, &remote_ctrl_i2c_info, 1);
	ret = platform_device_register(&remote_platform_device);
	if (ret != 0) {
		REM_LOG("[remote_ctrl] %s----error:%d\n", __func__, ret);
		return ret;
	}
	ret = platform_driver_register(&remote_platform_driver);
	if (ret) {
		REM_LOG("[remote_ctrl] %s----error:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static void __exit remote_exit(void)
{
	REM_LOG("[remote_ctrl] %s \n", __func__);
	//platform_device_unregister(&breathlight_platform_device);
	platform_driver_unregister(&remote_platform_driver);
}

/*****************************************************************************/
module_init(remote_init);
module_exit(remote_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Arnold Lu <lubaoquan@vanzotec.com>");
MODULE_DESCRIPTION("Breath LED light");
