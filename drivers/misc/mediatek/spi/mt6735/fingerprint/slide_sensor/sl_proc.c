/*
 * =====================================================================================
 *
 *       Filename:  sl_proc.c
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  2015年03月10日 23时02分25秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *        Company:
 *
 * =====================================================================================
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/compat.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>

#include <linux/jiffies.h>
#include <linux/spi/spi.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include "sl_proc.h"
/*
 *proc sl_fp command
 * start : start cali mode ,disable capture
 * stop : stop cali mode ,start capture
 * write addr data:  write data to reg, Ex. write FF08000C 0000009F >sl_fp
 * read reg reg_val: read reg val; Ex. read reg FF08000C >sl_fp 
 * read frame:  read frame from silead device:
 *
 *
 * */
#define SL_FP_PROC_FILE "sl_fp"
#define CONFIG_PROC_FS
static struct proc_dir_entry *sl_proc = NULL;
struct spidev_data  *fp_spidev = NULL;

static void sl_proc_cmd_read(char*cmd);
static void sl_proc_cmd_reg(char*cmd);
static void sl_proc_cmd_frame(char*cmd);
typedef enum {
    SL_READ_FRAME_STAT = 1,
    SL_WRTE_REG_STAT = 2,
    SL_READ_REG_STAT = 3,
    SL_UNKNOW_STAT = 0xFFFFFFFF,
} current_seq_stat_t;
static current_seq_stat_t g_current_stat = SL_UNKNOW_STAT;
static inline char* paser_sub_cmd(char *cmd)
{
    if (cmd == NULL) return cmd;
    if (!strlen(cmd)) return cmd;
    while(cmd) {
        if(isalnum(*cmd) != 0) {
            break;
        }
        cmd++;
    }
    return cmd;
}

static inline char* trim_sub_cmd(char *cmd)
{
    if (cmd == NULL) return cmd;
    if (!strlen(cmd)) return cmd;
    while(cmd) {
        if(isalnum(*cmd) == 0) {
            break;
        }
        cmd++;
    }
    return cmd;
}

static void* sl_seq_start(struct seq_file *f, loff_t *pos)
{
    if (SL_UNKNOW_STAT == g_current_stat) {
        dev_info(&fp_spidev->spi->dev, "SL proc Unknow state");
        return NULL;
    }
    if (*pos == 0) {
        if (SL_READ_REG_STAT == g_current_stat) {
            return (void*)fp_spidev->mmap_buf;
        }
        return (void*)fp_spidev->mmap_buf + SL_HEAD_SIZE;
    }
    if (*pos < SL_ONE_FRAME_PAGES) {
        return (void*)fp_spidev->mmap_buf + SL_HEAD_SIZE + (*pos)*SL_PAGE_SIZE;
    }
    return NULL;
}

static void *sl_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
    //dev_dbg(&fp_spidev->spi->dev, "%s:f=%p, *v=%p, *pos=%d:%d\n",__func__, f, v, *pos, SL_ONE_FRAME_PAGES);
    dev_dbg(&fp_spidev->spi->dev, "%s:f=%p, *v=%p, *pos=%d:%d\n",__func__, f, v, (int)*pos, SL_ONE_FRAME_PAGES);
    if (SL_READ_REG_STAT == g_current_stat) {
        dev_dbg(&fp_spidev->spi->dev, "%s:SL proc read reg state", __func__);
        return NULL;
    }

    if ((*pos) < SL_ONE_FRAME_PAGES) {
        (*pos) = (*pos) + 1;
        return (void*)((char*)v + SL_PAGE_SIZE);
    }
    return NULL;
}
static void sl_seq_stop(struct seq_file *f, void *v)
{
    printk(KERN_DEBUG "%s:f=%p, *v=%p\n", __func__, f, v);
    return;
}

static int sl_seq_show(struct seq_file *m, void *v)
{
    int i=0;
    char *data = (char*)v;

    printk(KERN_DEBUG "%s:f=%p, *v=%p\n", __func__, m, v);
    if (SL_READ_REG_STAT == g_current_stat) {
        while(NULL != *data) {
            seq_printf(m, "%c", *data);
            data++;
        }
    }
    if (SL_READ_FRAME_STAT == g_current_stat) {
        for(i=0; i < SL_PAGE_SIZE; i++) {
            seq_printf(m, "%02x", *data);
            data++;
        }
    }
    return 0;
}
const struct seq_operations sl_seq_ops = {
    .start	= sl_seq_start,
    .next	= sl_seq_next,
    .stop	= sl_seq_stop,
    .show	= sl_seq_show,
};

static int sl_proc_open(struct inode *inode, struct file *file)
{
    return seq_open(file, &sl_seq_ops);
}

static void sl_proc_cmd_start(char*cmd)
{
    printk(KERN_DEBUG "%s\n", __func__);
    atomic_set(&fp_spidev->is_cal_mode, 1);
}
static void sl_proc_cmd_stop(char*cmd)
{
    atomic_set(&fp_spidev->is_cal_mode, 0);
    printk(KERN_DEBUG "%s\n", __func__);
}
static void sl_proc_cmd_discal(char*cmd)
{
    printk(KERN_DEBUG "%s\n", __func__);
    atomic_set(&fp_spidev->is_cal_mode, 0);
}
static void sl_proc_cmd_encal(char*cmd)
{
    printk(KERN_DEBUG "%s\n", __func__);
    atomic_set(&fp_spidev->is_cal_mode, 1);
}

static void sl_proc_cmd_write(char*cmd)
{
    unsigned int  addr;
    unsigned int val32;
    char *val_cmd = NULL;
    if (cmd == NULL) return;
    if (!strlen(cmd)) return;
    addr = simple_strtol(cmd, NULL, 16);
    val_cmd = trim_sub_cmd(cmd);
    if (!val_cmd) {
        printk(KERN_ERR "%s:trim sub cmd failed\n", __func__ );
        return;
    }
    val_cmd = paser_sub_cmd(val_cmd);
    if (!val_cmd) {
        printk(KERN_ERR "%s:paser sub cmd failed\n", __func__ );
        return;
    }
    val32 = simple_strtol(val_cmd, NULL, 16);

    if (addr &(~0xFF)){
        spidev_write_reg(fp_spidev, (addr >>7), 0xF0);
        spidev_write_reg(fp_spidev, val32, addr%0x80);
    }else{
        spidev_write_reg(fp_spidev, val32, addr);
    }
}
typedef struct sl_proc_cmd {
    char *cmd;
    void (*call)(char *);
} sl_proc_cmd_t;

struct sl_proc_cmd sl_cmd[] = {
    {"start", sl_proc_cmd_start},
    {"stop",  sl_proc_cmd_stop},
    {"discal",sl_proc_cmd_discal},
    {"encal", sl_proc_cmd_encal},
    {"write", sl_proc_cmd_write},
    {"read",  sl_proc_cmd_read},
};
struct sl_proc_cmd sl_sub_cmd[] = {
    {"reg",   sl_proc_cmd_reg},
    {"frame", sl_proc_cmd_frame},
};

static void sl_proc_cmd_read(char*cmd)
{
    int i;
    for(i=0; i < sizeof(sl_sub_cmd)/sizeof(struct sl_proc_cmd); i++) {
        if((strncmp(sl_sub_cmd[i].cmd, cmd, strlen(sl_sub_cmd[i].cmd)) == 0) &&
                (sl_sub_cmd[i].call != NULL)) {
            sl_sub_cmd[i].call(paser_sub_cmd(cmd + strlen(sl_sub_cmd[i].cmd)));
            break;
        }
    }
}
static void sl_proc_cmd_reg(char*cmd)
{
    unsigned int addr;
    if (cmd == NULL) return;
    if (!strlen(cmd)) return;
    
    addr = simple_strtol(cmd, NULL, 16);
    
    if (addr &(~0xFF)){
        spidev_write_reg(fp_spidev, (addr >>7), 0xF0);
        sprintf(fp_spidev->mmap_buf, "0x%08x=0x%08x", addr, spidev_read_reg(fp_spidev, addr%0x80));
    }else{
        sprintf(fp_spidev->mmap_buf, "0x%08x=0x%08x", addr, /*0xFF&*/spidev_read_reg(fp_spidev, addr));
    }
    g_current_stat = SL_READ_REG_STAT;
}

static void sl_proc_cmd_frame(char*cmd)
{
    struct spidev_data *spidev = fp_spidev;
    struct spi_message	m;
    //struct spi_transfer	t[SL_ONE_FRAME_PAGES];
    struct spi_transfer	*t;
    int ret = 0, i;
    t  = kzalloc(SL_ONE_FRAME_PAGES,GFP_KERNEL);
    t[0].rx_buf	= spidev->mmap_buf;
    t[0].tx_buf	= spidev->tx_mmap_buf;
    t[0].len	= SL_HEAD_SIZE +SL_PAGE_SIZE;
    t[0].bits_per_word = SPI_BITS;
    t[0].delay_usecs = SPI_DELAY;
    t[0].speed_hz = SPI_SPEED;
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    for (i=1; i < SL_ONE_FRAME_PAGES; ++i) {
        t[i].rx_buf	= spidev->mmap_buf+i*SL_PAGE_SIZE + SL_HEAD_SIZE;
        t[i].tx_buf	= spidev->tx_mmap_buf+i*SL_PAGE_SIZE + SL_HEAD_SIZE;
        t[i].len    = SL_PAGE_SIZE;
        t[i].bits_per_word = SPI_BITS;
        t[i].delay_usecs = SPI_DELAY;
        t[i].speed_hz = SPI_SPEED;
        spi_message_add_tail(&t[i], &m);
    }
    init_frame(spidev);
    spidev_sync(spidev, &m);
    g_current_stat = SL_READ_FRAME_STAT;
}

static ssize_t sl_proc_write(struct file *fp, const char __user *buffer, size_t count, loff_t *f_pos)
{
    int i;
    char *cmd =(char*)kzalloc(count+1, GFP_KERNEL);
    if (cmd == NULL) {
        return 0;
    }
    memset(cmd, 0, count);
    if(copy_from_user(cmd, buffer, count)) {
        printk("copy from user fail\n");
        return -EIO;
    }

    for(i=0; i < sizeof(sl_cmd)/sizeof(struct sl_proc_cmd); i++) {
        if((strncmp(sl_cmd[i].cmd, cmd, strlen(sl_cmd[i].cmd)) == 0) &&
                (sl_cmd[i].call != NULL)) {
            sl_cmd[i].call(paser_sub_cmd(cmd + strlen(sl_cmd[i].cmd)));
            break;
        }
    }
    return count;
}
static const struct file_operations sl_proc_fops = {
    .open	= sl_proc_open,
    .read	= seq_read,
    .write	= sl_proc_write,
    .llseek	= seq_lseek,
    .release = seq_release,
};

int sl_proc_init(struct spidev_data *spidev)
{
#ifdef CONFIG_PROC_FS
	sl_proc = proc_create(SL_FP_PROC_FILE,0666,NULL,&sl_proc_fops);
    //sl_proc = create_proc_entry(SL_FP_PROC_FILE, 0666, NULL);
    //printk(KERN_INFO "[%s] sl_proc = %x \n",__func__,sl_proc);
    if ( NULL == sl_proc) {
        printk("create_proc_entry %s failed\n", SL_FP_PROC_FILE);
        return -1;
    }
    //sl_proc->proc_fops = &sl_proc_fops;
    fp_spidev = spidev;
#endif
    return 0;
}
