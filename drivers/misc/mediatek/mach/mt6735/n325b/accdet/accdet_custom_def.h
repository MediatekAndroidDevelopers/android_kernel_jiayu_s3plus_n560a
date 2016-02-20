// Headset mode MIC define
typedef enum
{
	ACCDET_MIC_MODE_ACC = 1,
	ACCDET_MIC_MODE_LOW_COST_WITHOUT_IN_BIAS = 2,
	ACCDET_MIC_MODE_LOW_COST_WITH_IN_BIAS = 6,
} ACCDET_MIC_MODE;
/* Vanzo:wangfei on: Fri, 06 Mar 2015 15:26:16 +0800
 * port from zhangqingzhan
#define ACCDET_MIC_MODE	(6)
 */
#define ACCDET_MIC_MODE    (1)

// End of Vanzo: wangfei

// use accdet + EINT solution
/* Vanzo:dingge on: Wed, 25 Mar 2015 17:11:25 +0800
 * TODO: for headset hotplug
 */
#define ACCDET_EINT   //ACC mode

// End of Vanzo: dingge
#ifndef ACCDET_EINT
#define ACCDET_EINT_IRQ  //DCC mode
#endif
//#define ACCDET_PIN_SWAP
//#define ACCDET_PIN_RECOGNIZATION
#define ACCDET_HIGH_VOL_MODE
#ifdef ACCDET_HIGH_VOL_MODE
#define ACCDET_MIC_VOL 7     //2.5v
#else
#define ACCDET_MIC_VOL 2     //1.9v
#endif

#define ACCDET_SHORT_PLUGOUT_DEBOUNCE
#define ACCDET_SHORT_PLUGOUT_DEBOUNCE_CN 20
//#define FOUR_KEY_HEADSET


