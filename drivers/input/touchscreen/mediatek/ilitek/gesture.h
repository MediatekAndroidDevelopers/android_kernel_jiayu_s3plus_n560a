/*static struct timeval   last_event_time;
static unsigned char finger_state=0;    //0,1,2,3,4
static int last_x=0;
static int last_y=0;
static int current_x=0;
static int current_y=0;*/
#define ABSSUB(a,b) ((a > b) ? (a - b) : (b - a))

#if GESTURE_FUN==GESTURE_FUN_2
int gesture_ili(uint16_t, uint16_t, int8_t);
#endif
#if GESTURE_FUN==GESTURE_FUN_1
char GestureMatchProcess(unsigned char ucCurrentPointNum,unsigned char ucLastPointNum,unsigned short curx,unsigned short cury);
int readgesturelist(void);
char GetGesture(void);
short gesture_model_value_x(int i);
short gesture_model_value_y(int i);
#ifndef GESTURE_H
	#define GESTURE_H
	#define E_N 100//41
	#define E_DATA_LEN 16
	extern int model_number;//=52;//48;
	//char e_str[E_N] = {'c','c','c','e','e','e','m','m','m','m','m','m','w','w','w','w','o','o','o','o','o','o','o','o','o'};
	//char e_str[E_N] = {'c','c','c','e','e','e','m','m','m','m','m','m','w','w','w','w','o','o','o','o','o','o','o','o','o'};
	extern char e_str[E_N] ;
	extern short e[E_N*2][E_DATA_LEN];//

#endif
#endif
/*#ifdef _DOUBLE_CLICK_
int event_spacing=0;

static bool g_gesture_active_flag = false;
//static int  double_click_touch(int x, int y,char finger_state);
//static int  double_click_release(int x, int y,char finger_state);
static int get_time_diff (struct timeval *past_time)
{
    struct  timeval   time_now;
    int diff_milliseconds=0;
	//取得目前時間
    do_gettimeofday(&time_now);
	//算出相差的時間 單位ms
    diff_milliseconds+=(time_now.tv_sec-past_time->tv_sec)*1000;
	
    if(time_now.tv_usec<past_time->tv_usec)
    {
        diff_milliseconds-=1000;
        diff_milliseconds+=(1000*1000+time_now.tv_usec-past_time->tv_usec)/1000;
    }
    else
    {
        diff_milliseconds+=(time_now.tv_usec-past_time->tv_usec)/1000;
    }

    if(diff_milliseconds< (-10000))
        diff_milliseconds=10000;
    return diff_milliseconds;
}
static int  double_click_touch(int x,int y,char finger_state,int i){
	if(i > 0){
		finger_state = 0;
		printk("------------------------i=%d\n",i);
		return finger_state;
	}
		
	if(finger_state==0||finger_state ==5)
	{
	
		finger_state =1;
		last_x =x;
		last_y =y;
		event_spacing =0;
		do_gettimeofday(&last_event_time);
	}
	else if(finger_state == 1)
	{
		event_spacing += get_time_diff(&last_event_time);
		if(event_spacing>800)
		{
			finger_state =4;
		}
	}
	else if(finger_state==2)
	{
		finger_state =3;
		current_x =x;
		current_y =y;
		current_x =0;
		current_y =0;
		event_spacing += get_time_diff(&last_event_time);
		if(event_spacing>1000)
		{
			finger_state =0;
		}
	}
	else if(finger_state==3)
	{
		current_x =x;
		current_y =y;
		event_spacing += get_time_diff(&last_event_time);							   	  
		if(event_spacing>2000)
		{
			finger_state =4;
		}
	}
	//printk("---double_click_touch 1---finger_state=%d-------event_spacing=%d\n",finger_state,event_spacing);
	return finger_state;
}

static int  double_click_release(int x, int y,char finger_state){							
		if(finger_state==1)
		{
			//printk("---double_click_release 1---finger_state=%d-------\n",finger_state);
			finger_state =2;
			event_spacing += get_time_diff(&last_event_time);
			if(event_spacing>875)
			{
				//printk("---double_click_release 2---finger_state=%d-------\n",finger_state);
				finger_state =0;
			}	
		}
		if(finger_state==3)
		{
			
			event_spacing += get_time_diff(&last_event_time);
			//printk("---double_click_release 3---finger_state=%d-------event_spacing=%d\n",finger_state,event_spacing);
			if((event_spacing<2500&&event_spacing>50)&&(ABSSUB(current_x,last_x)<100)&&((ABSSUB(current_y,last_y)<100)))
			{
				//printk("---double_click_release 4---finger_state=%d-------\n",finger_state);
				finger_state =0;

				if(g_gesture_active_flag != true)
				{
					//printk("---double_click_release 5---finger_state=%d-------\n",finger_state);
					g_gesture_active_flag = true;
					printk("current_x=%d,current_y=%d,last_x=%d,last_y=%d\n",current_x,current_y,last_x,last_y);
					printk("double click resume enable\n");
					finger_state = 5;
					return finger_state;
				}

				else
				{
					//printk("---double_click_release 6---finger_state=%d-------\n",finger_state);
					printk("[Bruce][gesture]double click resume disabled\n");
				}
			}
			else
			{
				//printk("---double_click_release 7---finger_state=%d-------\n",finger_state);
				finger_state =0;
			}
		}else if(finger_state==4)
		{
			//printk("---double_click_release 8---finger_state=%d-------\n",finger_state);
			finger_state =0;
		}
		//printk("---double_click_release 9--finger_state=%d-------event_spacing=%d\n",finger_state,event_spacing);
		g_gesture_active_flag = false;
		return finger_state;
}
#endif*/

