#define EVENT_AMBIGUOUS		0
#define EVENT_HOLD				1
#define EVENT_RIGHT				2
#define EVENT_LEFT				3
#define EVENT_UP				4
#define EVENT_DOWN				5
#define EVENT_ZOOM_IN			6
#define EVENT_ZOOM_OUT			7
#define EVENT_ZOOM_START		8
#define EVENT_ZOOM_END			9
#define EVENT_UNKNOWN			10

#define KEYCODE_AMBIGUOUS		KEY_F1
#define KEYCODE_HOLD			KEY_F2
#define KEYCODE_RIGHT			KEY_F3
#define KEYCODE_LEFT			KEY_F4
#define KEYCODE_UP				KEY_F5
#define KEYCODE_DOWN			KEY_F6
#define KEYCODE_ZOOM_IN		KEY_F7
#define KEYCODE_ZOOM_OUT		KEY_F8
#define KEYCODE_ZOOM_START	KEY_F9
#define KEYCODE_ZOOM_END		KEY_F10

#define BUF_SIZE 					800

#define GES_MAX_INTERNAL_TIME		300 	//300ms
#define GES_MIN_INTERNAL_TIME		20   	//20ms
#define GES_SPACING_TIME			50 	//200ms
#define GES_STABLE_TIME 			150  //100ms
#define GES_HOLD_TIME 				1200 //1000ms > zoom start + end time
#define GES_ZOOM_START_TIME		350  //350ms
#define GES_ZOOM_END_TIME			750  //750ms

#define GES_MODE_4D				0
#define GES_MODE_HORIZONTAL		1
#define GES_MODE_VERTICAL			2

#define STATUS_WAITING				0
#define STATUS_LOW 					1
#define STATUS_WORK				2
#define STATUS_FINISH 				3
#define STATUS_RESTART				4

static int KEYCODE_ARRAY[10] =
{
    KEYCODE_AMBIGUOUS,
    KEYCODE_HOLD,
    KEYCODE_RIGHT,
    KEYCODE_LEFT,
    KEYCODE_UP,
    KEYCODE_DOWN,
    KEYCODE_ZOOM_IN,
    KEYCODE_ZOOM_OUT,
    KEYCODE_ZOOM_START,
    KEYCODE_ZOOM_END
};

static int WORK_TH=5;
static int HOLD_TH=20;

static int zoom_delta=5;
static int zoom_count=0;

static int file_count=0;
static int start_idx=0;
static int ges_count=0;

static int opt = 2;						// 0: no filter, 	1: one filter, 	2: two filter
static int GESTURE_DEBUG = 1; 			// 0: no log, 	1: show log, 	2: show ambiguous, 	3: save data
static int avg_length = 0;
static int ges_mode = GES_MODE_4D;

static int interpolated_data[1500]= {0};
static int cal_data[BUF_SIZE]= {0};
static short ges_raws[BUF_SIZE]= {0};

static bool is_zoom_notify=false;
static bool is_hold_notify=false;
static bool is_hold_triger=false;
static bool is_work_triger=false;
static bool is_zoom_end=false;
static int is_event_triger=STATUS_LOW;

static struct timeval   work_start_time;
static struct timeval   event_triger_time;
static struct timeval   work_start_time;
static struct timeval   hold_start_time;
static struct timeval   zoom_start_time;

static int zoom_enabled=false;
static int crosstalk=500;
static int ctk_offset=3;
static int last_ctk=0;
static int ctk_count =0;
static int zoom_data;

static int paras[12] = { 0, 6,36, 98, 175, 230, 230, 175, 98, 36, 6, 0 };
static int filter_buffer[4*11]= {0};
static int channel_data[4];

static int zoom_ratio = 4;

#define GES_LOG(fmt, args...)    printk(KERN_INFO "epl8865 gesture:" fmt, ##args)

static int get_time_diff (struct timeval *past_time)
{
    struct  timeval   time_now;
    int diff_milliseconds=0;

    do_gettimeofday(&time_now);
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

static void average_filter(int *data, int length, int buffersize)
{
    int buffer[4][buffersize];
    int new_array[length];
    int i, j;
    int mean, whichch;

    if(buffersize==0)
        return;

    for(i=0; i<4; i++)
        memset(buffer[i],0,buffersize*sizeof(int));

    for (i = 0; i <length; i++)
    {
        whichch = i % 4;

        if(buffersize>1)
        {
            for(j=(buffersize-1); j>0; j--)
                buffer[whichch][j] = buffer[whichch][j-1];
        }
        buffer[whichch][0] = data[i];

        mean = 0;
        for(j=0; j<buffersize; j++)
            mean += buffer[whichch][j];
        new_array[i] = mean/buffersize;
    }

    for (i = 0; i <length; i++)
        data[i]=new_array[i];
}

static int sum_of_absolute_difference_filter(int *data, int array_len, int buffer_len, int idx1, int idx2)
{
    int i,j,k;
    int start = 0;
    int max=0, max_idx=0;
    int result, index;

    for (i = 0; i < buffer_len; i++)
    {
        result =0;
        index = i - buffer_len / 2;

        for (j = start; j < array_len; j+=4)
        {
            k = (j + idx2) + (i - buffer_len / 2) * 4;
            if (k >= start && k < array_len)
                result += data[j + idx1] * data[k];
        }
        if(result > max)
        {
            max = result;
            max_idx = index;
        }
    }
    if (GESTURE_DEBUG)
        GES_LOG("max: %d, max_idx: %d \n",max, max_idx);

    return max_idx;
}

static int check_4D(int *data, int array_len, int buffer_len)
{
    int max01_index,max12_index, max32_index, max03_index, difference, hori_diff, vert_diff;
    max01_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 0, 1);
    max32_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 3, 2);
    max12_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 1, 2);
    max03_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 0, 3);

    hori_diff =  (abs(max12_index) + abs(max03_index))*13;
    vert_diff = (abs(max01_index) + abs(max32_index))*10;

    if ( vert_diff < hori_diff)
    {
        difference = max12_index+max03_index;
        if (difference < 0)
            return EVENT_RIGHT;
        else if (difference>0)
            return EVENT_LEFT;
    }
    else if (vert_diff > hori_diff)
    {
        difference = max01_index+max32_index;
        if (difference < 0)
            return EVENT_UP;
        else if (difference>0)
            return EVENT_DOWN;
    }

    return  GESTURE_DEBUG>=2?EVENT_AMBIGUOUS:EVENT_UNKNOWN;
}

static int check_horizontal(int *data, int array_len, int buffer_len)
{
    int max12_index,max03_index, difference;
    max12_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 1, 2);
    max03_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 0, 3);

    difference = max12_index + max03_index;
    if (difference < 0)
        return EVENT_RIGHT;
    else  if (difference > 0)
        return EVENT_LEFT;

    return  GESTURE_DEBUG>=2?EVENT_AMBIGUOUS:EVENT_UNKNOWN;
}

static int check_vertical(int *data, int array_len, int buffer_len)
{
    int max01_index,max32_index, difference;
    max01_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 0, 1);
    max32_index = sum_of_absolute_difference_filter(data, array_len, buffer_len, 3, 2);

    difference = max01_index + max32_index;
    if (difference < 0)
        return EVENT_UP;
    else  if (difference > 0)
        return EVENT_DOWN;

    return  GESTURE_DEBUG>=2?EVENT_AMBIGUOUS:EVENT_UNKNOWN;
}

static void save_data(int *data, int length)
{
    struct file *fp;
    char file[40];
    char strbuf[10];
    int  i,  len;

    mm_segment_t old_fs=get_fs ();
    set_fs (KERNEL_DS);

    GES_LOG("file name=ges_data_%d\n", file_count);
    sprintf(file,"/mnt/sdcard/ges_data_%d.txt", file_count++);
    fp = filp_open(file,O_WRONLY|O_CREAT, 0644);
    fp->f_pos = 0;

    for(i=0; i<length; i++)
    {
        len = sprintf(strbuf, "%d\r\n", data[i]);
        fp->f_op->write( fp, strbuf, len, &fp->f_pos);
    }

    filp_close(fp, NULL);
    set_fs (old_fs);

}


static int get_gesture_event(void)
{
    struct timeval  time_cal;
    int idx=0, idx2=0;
    int event;
    int minimum[4] = {520,520,520,520} ;
    int i,j, k, start=0, count;
    int *data, *short_data;
    int p=2, all=p+1;

    if (GESTURE_DEBUG>=2)
        do_gettimeofday(&time_cal);

    for(i= start_idx; i<BUF_SIZE ; i++)
    {
        if(idx>=ges_count)
            break;
        cal_data[idx++]=ges_raws[i];
    }
    if(idx<ges_count)
    {
        for(i= 0; i<start_idx ; i++)
        {
            if(idx>=ges_count)
                break;
            cal_data[idx++]=ges_raws[i];
        }
    }


    for (i = idx-4; i > 4; i-=4)
    {
        if (cal_data[i] - crosstalk >= WORK_TH && cal_data[i-4] - crosstalk < WORK_TH )
        {
            start = i-32;
            if(start<0)
                start = 0;
            idx-=start;
            break;
        }
    }
    short_data = &cal_data[start];

    if (GESTURE_DEBUG>=3)
        save_data(short_data, idx);

    if(opt>=2 && idx*all<=1500)
    {
        count =idx/4;
        idx2 =0;

        for (i = 0; i < count; i++)
        {
            j = i*4;
            for(k=0; k<4; k++)
                interpolated_data[ idx2++] = short_data[j+k];

            if( j+7<idx)
            {
                for (k = 1; k <= p; k++)
                {
                    interpolated_data[idx2++] = (short_data[j + 0] * (all - k) + short_data[j + 4] * k)  / all;
                    interpolated_data[idx2++] = (short_data[j + 1] * (all - k) + short_data[j + 5] * k)  / all;
                    interpolated_data[idx2++] = (short_data[j + 2] * (all - k) + short_data[j + 6] * k)  / all;
                    interpolated_data[idx2++] = (short_data[j + 3] * (all - k) + short_data[j + 7] * k)  / all;
                }
            }
        }
        data = &interpolated_data[0];
        idx = idx2;

    }
    else
    {
        data = short_data;
    }


    for (i = 0; i < idx; i++)
    {
        j = i%4;
        if (data[i] < minimum[j])
            minimum[j] = data[i];
    }
    for (i = 0; i < idx; i++)
    {
        data[i] -=  minimum[i%4];
    }

    average_filter(data, idx, avg_length);

    if(ges_mode== GES_MODE_HORIZONTAL)
        event = check_horizontal( data, idx, 6);
    else if(ges_mode== GES_MODE_VERTICAL)
        event = check_vertical( data, idx, 6);
    else  if(ges_mode== GES_MODE_4D)
        event =check_4D( data ,idx, 6);
    else
        event = EVENT_LEFT;

    if (GESTURE_DEBUG>=2)
    {
        GES_LOG("!! get event cost %d ms\n", get_time_diff(&time_cal));
    }

    return event;
}

static void update_crosstalk(int cmp_data)
{
    if(zoom_enabled==true)
        return;

    if(last_ctk <= cmp_data+ctk_offset && last_ctk >= cmp_data-ctk_offset )
    {
        ctk_count++;
        if(crosstalk!=cmp_data && ctk_count > 300) // ctk_count > hold time
        {
            crosstalk = cmp_data;
            is_event_triger=STATUS_RESTART;
        }
    }
    else
    {
        ctk_count=0;
    }
    last_ctk = cmp_data;

}

static int detect_gesture_event(void)
{
    bool isWork, isHold;
    bool isNotWork;
    int evt = EVENT_UNKNOWN;
    int cmp_data = channel_data[0] ;
    int new_zoom_data;
    int time;
    int var;

    update_crosstalk(cmp_data);
    cmp_data  = channel_data[0]-crosstalk;

    isWork = cmp_data >=WORK_TH ;
    isNotWork = cmp_data <WORK_TH ;

    if(is_event_triger==STATUS_FINISH)
    {
        if(isWork)
        {
            is_event_triger=STATUS_RESTART;
            is_work_triger=false;
        }
        else if( get_time_diff(&event_triger_time)> GES_STABLE_TIME)
        {
            evt =  get_gesture_event();
            is_event_triger=STATUS_LOW;
        }

    }
    else if(isWork)
    {
        if(is_work_triger==false)
        {
            do_gettimeofday(&work_start_time);
        }
        is_work_triger=true;

        isHold = cmp_data >HOLD_TH;

        if(isHold )
        {
            if(is_hold_triger)
            {

                if(zoom_enabled)
                {
                    new_zoom_data = 100000/int_sqrt(cmp_data*10000);

                    if(is_zoom_notify==false)
                    {
                        if( get_time_diff(&hold_start_time)> GES_ZOOM_START_TIME)
                        {
                            evt  = EVENT_ZOOM_START;
                            zoom_count=0;
                            zoom_data = new_zoom_data;
                            is_zoom_notify = true;
                            is_zoom_end =false;
                            do_gettimeofday(&zoom_start_time);
                        }
                    }
                    else if(is_zoom_end==false && is_zoom_notify==true)
                    {
                        time = get_time_diff(&zoom_start_time);
                        var = new_zoom_data-zoom_data;

                        if(abs(var)> zoom_ratio*zoom_delta && time> GES_SPACING_TIME )
                        {
                            zoom_data = new_zoom_data;
                            do_gettimeofday(&zoom_start_time);
                        }
                        else if(abs(var)> zoom_delta && var<0 &&  time> GES_SPACING_TIME)
                        {
                            zoom_count++;
                            zoom_data = new_zoom_data;
                            do_gettimeofday(&zoom_start_time);
                            return EVENT_ZOOM_IN;
                        }
                        else if(abs(var)> zoom_delta  && time> GES_SPACING_TIME)
                        {
                            zoom_count++;
                            zoom_data = new_zoom_data;
                            do_gettimeofday(&zoom_start_time);
                            return EVENT_ZOOM_OUT;
                        }
                        else if(abs(var)<=zoom_delta && time> GES_ZOOM_END_TIME)
                        {
                            is_zoom_end=true;
                            //zoom_count=0;
                            return EVENT_ZOOM_END;
                        }
                    }
                }


                if(is_hold_notify==false &&  zoom_count==0 && get_time_diff(&hold_start_time)> GES_HOLD_TIME)
                {
                    evt  = EVENT_HOLD;
                    is_hold_notify=true;
                    zoom_count=0;
                    //  is_zoom_end=true;
                }
            }
            else
            {
                is_hold_notify = false;
                is_zoom_notify = false;
                is_hold_triger = true;
                do_gettimeofday(&hold_start_time);
            }
        }
        else
        {
            if(zoom_count!=0 && is_zoom_end!=true)
            {
                evt = EVENT_ZOOM_END;
                zoom_count=0;
            }
            is_hold_triger=false;
        }

    }
    else if(isNotWork)
    {
        if(is_work_triger && is_event_triger != STATUS_RESTART)
        {
            int event_time = get_time_diff(&work_start_time);
            if( event_time <= GES_MAX_INTERNAL_TIME && event_time >= GES_MIN_INTERNAL_TIME)
            {
                is_event_triger = STATUS_FINISH;
                do_gettimeofday(&event_triger_time);
            }
            else
            {
                GES_LOG("event_time %d ms, too long\n",event_time);
            }
        }

        if(zoom_count!=0 && is_zoom_end!=true)
        {
            evt = EVENT_ZOOM_END;
            zoom_count=0;
        }
        is_work_triger=false;
        is_hold_triger=false;

        if(is_event_triger != STATUS_FINISH)
            is_event_triger = STATUS_LOW;
    }

    return evt;
}


static void add_gesture_data(u16* data, int ir_on)
{
    int i, j,k, channel_value;
    int idx = start_idx+ges_count;

    if(idx>=BUF_SIZE)
        idx-=BUF_SIZE;

    if(ir_on==0)
    {
        for(i=0; i<4; i++)
        {
            if(channel_data[i] > data[i])
                channel_data[i] = channel_data[i] - data[i];
            else
                channel_data[i] = 0;
        }

        if(opt>=1)
        {
            for (k = 0; k < 4; k++)
            {
                channel_value = channel_data[k];

                channel_data[k]= (channel_value * paras[0]);
                for ( j = 0; j < 11; j++)
                    channel_data[k]+= filter_buffer[(j * 4) + k] * paras[j + 1];
                channel_data[k]/=1024;

                for (j =10; j > 0; j--)
                    filter_buffer[(j * 4) + k] = filter_buffer[(j - 1)*4+k];
                filter_buffer[k] = channel_value;
            }
        }


    }
    else
    {
        for(i=0; i<4; i++)
            channel_data[i] = data[i];

        return;
    }

    for(i=0; i<4; i++)
    {
        ges_raws[idx+i] = channel_data[i];

        if(ges_count>=BUF_SIZE)
        {
            start_idx++;
            if(start_idx>=BUF_SIZE)
                start_idx=0;
        }
    }

    ges_count+=4;
    if(ges_count>=BUF_SIZE)
        ges_count=BUF_SIZE;
}
