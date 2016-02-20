#ifndef __GF318M_SPI_H
#define __GF318M_SPI_H

#include <linux/types.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>

/********************GF318M Mapping**********************/
#define GF318M_BASE             (0x8000)
#define GF318M_OFFSET(x)        (GF318M_BASE + x)

#define GF318M_VERSION		    GF318M_OFFSET(0)
#define GF318M_CONFIG_DATA 	    GF318M_OFFSET(0x40)
#define GF318M_CFG_ADDR  	    GF318M_OFFSET(0x47)
#define GF318M_MODE_STATUS      GF318M_OFFSET(0x043)
//#define GF318M_MIXER_DATA	    GF318M_OFFSET(0x140)
#define GF318M_BUFFER_STATUS	GF318M_OFFSET(0x140)
#define GF318M_KEY_DATA         GF318M_OFFSET(0x142)
#define GF318M_NOISE_DATA       GF318M_OFFSET(0x144)
#define GF318M_LONG_PRESS_STDP  GF318M_OFFSET(0x146)
#define GF318M_BUFFER_DATA	    GF318M_OFFSET(0x141)


#define GF318M_BUF_STA_MASK	    (0x1<<7)
#define	GF318M_BUF_STA_READY	(0x1<<7)
#define	GF318M_BUF_STA_BUSY	    (0x0<<7)

#define	GF318M_IMAGE_MASK	    (0x1<<6)
#define	GF318M_IMAGE_ENABLE	    (0x1)
#define	GF318M_IMAGE_DISABLE	(0x0)

#define	GF318M_KEY_MASK		    (GF318M_HOME_KEY_MASK | \
                                 GF318M_MENU_KEY_MASK | \
                                 GF318M_BACK_KEY_MASK )
//#define	GF318M_KEY_ENABLE	    (0x1)
//#define	GF318M_KEY_DISABLE	    (0x0)

//#define	GF318M_KEY_STA		    (0x1<<4)

//home key
#define	GF318M_HOME_KEY_MASK	(0x1<<5)
#define	GF318M_HOME_KEY_ENABL   (0x1)
#define	GF318M_HOME_KEY_DISABLE (0x0)

#define	GF318M_HOME_KEY_STA	    (0x1<<4)
//menu key
#define	GF318M_MENU_KEY_MASK    (0x1<<3)
#define	GF318M_MENU_KEY_ENABLE	(0x1)
#define	GF318M_MENU_KEY_DISABLE	(0x0)

#define	GF318M_MENU_KEY_STA		(0x1<<2)
//back key
#define	GF318M_BACK_KEY_MASK    (0x1<<1)
#define	GF318M_BACK_KEY_ENABLE  (0x1)
#define	GF318M_BACK_KEY_DISABLE (0x0)

#define	GF318M_BACK_KEY_STA	    (0x1<<0)


#define	GF318M_IMAGE_MODE	    (0x00)
#define	GF318M_KEY_MODE		    (0x01)
#define GF318M_SLEEP_MODE       (0x02)
#define GF318M_FF_MODE	    	(0x03)
#define GF318M_DEBUG_MODE	    (0x56)

/**********************GF318M ops****************************/
#define GF318M_W                0xF0
#define GF318M_R                0xF1
#define GF318M_WDATA_OFFSET	    (0x3)
#define GF318M_RDATA_OFFSET	    (0x5)
#define GF318M_CFG_LEN			(249)	/*config data length*/
/**********************************************************/

/**********************IO Magic**********************/
#define  GF318M_IOC_MAGIC    'g'  //define magic number
struct gf318m_ioc_transfer {
	u8 cmd;
       u8 reserved;
	u16 addr;
	u32 len;
	u32 buf;
};
//define commands
/*read/write GF318M registers*/
#define  GF318M_IOC_CMD	_IOWR(GF318M_IOC_MAGIC, 1, struct gf318m_ioc_transfer)
#define  GF318M_IOC_REINIT	_IO(GF318M_IOC_MAGIC, 0)
#define  GF318M_IOC_SETSPEED _IOW(GF318M_IOC_MAGIC, 2, u32)
#define  GF318M_IOC_STOPTIMER   _IO(GF318M_IOC_MAGIC, 3)
#define  GF318M_IOC_STARTTIMER  _IO(GF318M_IOC_MAGIC, 4)

#define  GF318M_IOC_MAXNR 5

/*******************Refering to platform*****************************/
/*
#define ONTIM_MTK6735
#ifdef ONTIM_MTK6735
#define 		GF318M_POWER_SOURCE_CUSTOM    PMIC_APP_CAP_TOUCH_VDD   //used as same to VGP1_PMU for TP
#else
#define          GF318M_PWR_PIN		(GPIO45 | 0x80000000)
#define 		GF318M_PWR_PIN_M_GPIO   GPIO_MODE_00
#endif
*/
          
#define		GF318M_IRQ_NUM		90
#define 	GF318M_INT_PIN 		(GPIO90 | 0x80000000)
#define 	GF318M_SPI_EINT_PIN_M_GPIO   GPIO_MODE_00
#define 	GF318M_SPI_EINT_PIN_M_EINT   GPIO_MODE_00

#define 	GF318M_RST_PIN		(GPIO89 | 0x80000000)
#define 		GF318M_SPI_RESET_PIN_M_GPIO   GPIO_MODE_00
#define 		GF318M_SPI_RESET_PIN_M_DAIPCMOUT   GPIO_MODE_01

#define		GF318M_SPI_SCK_PIN		(GPIO66 | 0x80000000)            
#define		GF318M_SPI_SCK_PIN_M_GPIO	GPIO_MODE_00
#define		GF318M_SPI_SCK_PIN_M_SCK	GPIO_MODE_01

#define		GF318M_SPI_CS_PIN		(GPIO65 | 0x80000000)
#define		GF318M_SPI_CS_PIN_M_GPIO	GPIO_MODE_00
#define		GF318M_SPI_CS_PIN_M_CS	GPIO_MODE_01
	
#define		GF318M_SPI_MOSI_PIN		(GPIO67 | 0x80000000)
#define		GF318M_SPI_MOSI_PIN_M_GPIO	GPIO_MODE_00
#define		GF318M_SPI_MOSI_PIN_M_MOSI	GPIO_MODE_01

#define		GF318M_SPI_MISO_PIN		(GPIO68 | 0x80000000)
#define		GF318M_SPI_MISO_PIN_M_GPIO	GPIO_MODE_00
#define		GF318M_SPI_MISO_PIN_M_MISO	GPIO_MODE_01

/*************************************************************/
#define GF318M_FASYNC 		1//If support fasync mechanism.
//#undef GF318M_FASYNC
//#define CHARGER_DETECT      1
struct gf318m_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev 	*input;
    struct work_struct  spi_work;
    struct timer_list   gf318m_timer;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;
    unsigned long irq_gpio;
    unsigned long reset_gpio;
    unsigned long power_gpio;
	u8 mode;
#ifdef CHARGER_DETECT
    struct usb_phy      *phy;
    struct notifier_block  nb;
#endif //CHARGER_DETECT
#ifdef GF318M_FASYNC
    struct  fasync_struct *async;
#endif
};

/*GPIO pin reference*/
int gf318m_parse_dts(struct gf318m_dev *gf318m_dev);
void gf318m_spi_pins_config(void);

/*Power Management*/
int gf318m_power_on(struct gf318m_dev *gf318m_dev);
int gf318m_power_off(struct gf318m_dev *gf318m_dev);

int gf318m_hw_reset(struct gf318m_dev *gf318m_dev, unsigned int delay_ms);
/*IRQ reference.*/
int gf318m_irq_setup(struct gf318m_dev *gf318m_dev, void (*irq_handler)(void));
int gf318m_irq_release(struct gf318m_dev *gf318m_dev);
int gf318m_irq_num(struct gf318m_dev *gf318m_dev);
int gf318m_irq_open(int irq_no);
int gf318m_irq_close(int irq_no);

int gf318m_spi_write_bytes(struct gf318m_dev *gf318m_dev,u16 addr, u32 data_len, u8 *tx_buf);
int gf318m_spi_read_bytes(struct gf318m_dev *gf318m_dev,u16 addr, u32 data_len, u8 *rx_buf);
int gf318m_fw_update(struct gf318m_dev* gf318m_dev, unsigned char *buf, unsigned short len);
#endif //__GF318M_SPI_H
