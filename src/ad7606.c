/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-06     17549       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drv_common.h>
#include <string.h>
#include <math.h>
#include <common_def.h>

#ifdef DEVICE_XLJC904

#define DBG_TAG "ad7606"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define AD7606_ADD              ((uint32_t)0x68000000)
#define AD7606_BUSY_PIN_VALUE   (rt_pin_read(AD7606_BUSY_GET_PIN))

#define AD7606_CS_GET_PIN       (GET_PIN(G, 4))
#define AD7606_RESET_GET_PIN    (GET_PIN(G, 5))
#define AD7606_BUSY_GET_PIN     (GET_PIN(G, 3))
#define AD7606_CONVST_B_PIN     (GET_PIN(D, 11))
#define AD7606_FRSTDATA_PIN     (GET_PIN(G, 2))

#define LO_SAMPLING_POINTS     (64*10)      //工频的采样点数
#define TEMP_SAMPLING_POINTS   (100)        //温度的采样点数

static uint16_t lo_sample_datacount=0;      // 工频采样数据计数
static uint16_t temp_sample_datacount=0;    // 温度采样数据计数

static struct rt_semaphore data_rcv_sem;
static rt_thread_t ad7606_tid = RT_NULL;

#define PWM_DEV_NAME        "pwm4"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL     1       /* PWM通道 */
struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */

extern SRAM_HandleTypeDef hsram3;

//PT100相关参数
#define     A       3.9083 * pow(10,-3)
#define     B       -5.775 * pow(10,-7)
#define     C       -4.27350 * pow(10,-12)

typedef struct ad7606_data_t
{
    int16_t lodata_raw[LO_SAMPLING_POINTS];                           //ADC工频原始数据
    uint16_t temperature_raw[2][TEMP_SAMPLING_POINTS];                //ADC温度原始数据
    float rms_value;                                                  //计算后的工频有效值
    double_t temperature[2];                                          //计算后的温度有效值
}ad7606_data_t;

static ad7606_data_t ad7606_data;

typedef struct lo_algo_para
{
    float_t lo_calib_ac;           //交流比例
    float_t  lo_calib_dc;           //直流偏置
}lo_algo_para;

static lo_algo_para lo_algo_data;

typedef struct temp_algo_para
{
    float_t  temp_calib_dc[2];           //温度偏置
}temp_algo_para;

static temp_algo_para temp_algo_data;

uint16_t inAD7606_READ1(void)
{
#if 1
    return *(volatile uint16_t*) (AD7606_ADD);
#else
    uint16_t Data;
    HAL_SRAM_Read_16b(&hsram3, (uint32_t *)AD7606_ADD, (uint16_t *)&Data, 1);
    return Data;
#endif
}

static void ad7606_start()
{
    rt_pin_irq_enable(AD7606_BUSY_GET_PIN, PIN_IRQ_ENABLE);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
}

static void ad7606_stop()
{
    rt_pin_irq_enable(AD7606_BUSY_GET_PIN, PIN_IRQ_DISABLE);
    /* 关闭设备通道 */
    rt_pwm_disable(pwm_dev,PWM_DEV_CHANNEL);
}


static void ad7606_reset()
{
    rt_pin_mode(AD7606_RESET_GET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(AD7606_RESET_GET_PIN, PIN_LOW);
    rt_pin_write(AD7606_RESET_GET_PIN, PIN_HIGH);
    //拉高至少50ns
    rt_hw_us_delay(1);
    rt_pin_write(AD7606_RESET_GET_PIN, PIN_LOW);
}

static int16_t test[8] = {0};
static void ad7606_busy_irq()
{
//    while(AD7606_BUSY_PIN_VALUE);
    //V1
    ad7606_data.lodata_raw[lo_sample_datacount++] = inAD7606_READ1();

    //V2
    inAD7606_READ1();

    //V3
    ad7606_data.temperature_raw[0][temp_sample_datacount] = inAD7606_READ1();

    //V4
    ad7606_data.temperature_raw[1][temp_sample_datacount++] = inAD7606_READ1();

    //V5
    test[0] = inAD7606_READ1();
    //V6
    test[1] = inAD7606_READ1();
    //V7
    test[2] = inAD7606_READ1();
    //V8
    test[3] = inAD7606_READ1();
    if(temp_sample_datacount == TEMP_SAMPLING_POINTS)
    {
        temp_sample_datacount = 0;
    }

    if(lo_sample_datacount == LO_SAMPLING_POINTS-1)
    {
        lo_sample_datacount = 0;
        //通知操作
        rt_sem_release(&data_rcv_sem);
    }
}


/**
 * \brief lwgps process thread
 *
 * \param parameter: input parameters.
 */
static void ad7606_thread_entry(void *parameter)
{
    ad7606_data_t *temp_data = &ad7606_data;
    uint64_t temp_rms = 0;
    double_t temp_v[2] = {0};
    double_t temp_r[2] = {0};
    double_t temp_sum = 0;
    float wendu[2][TEMP_SAMPLING_POINTS] = {0};
    ad7606_reset();
    rt_hw_us_delay(1);
    ad7606_start();

    //测试用，先初始化成0，后面读写flash进行保存
    lo_algo_data.lo_calib_ac = 1;
    lo_algo_data.lo_calib_dc = 0;

    temp_algo_data.temp_calib_dc[0] = 0;
    temp_algo_data.temp_calib_dc[1] = 0;

    while (RT_TRUE)
    {
        rt_sem_take(&data_rcv_sem, RT_WAITING_FOREVER);
        temp_rms = 0;
        for(int i = 0 ; i < LO_SAMPLING_POINTS; i++)
        {
            temp_rms += temp_data->lodata_raw[i] * temp_data->lodata_raw[i];
        }
        temp_rms = temp_rms/LO_SAMPLING_POINTS;
        temp_data->rms_value = sqrt((float_t)temp_rms);

        temp_data->rms_value = (10*temp_data->rms_value)/32768.0;

        temp_data->rms_value = (temp_data->rms_value/96.5064/1.051 - lo_algo_data.lo_calib_dc) * lo_algo_data.lo_calib_ac ;

        temp_data->rms_value = temp_data->rms_value*1000*1000/6.8;
//        rt_kprintf("rms: %.5f \r\n",temp_data->rms_value);

        for(int j=0;j<2;j++)
        {
            temp_sum = 0;
            for(int k=0;k<TEMP_SAMPLING_POINTS;k++)
            {
                temp_v[j] = (temp_data->temperature_raw[j][k] * 10) / 32767.0;
                temp_r[j] = (10000*temp_v[j]/(5.11-temp_v[j]));
                wendu[j][k] = (-A + sqrt(A*A - 4 * B + (B*temp_r[j])/250))/(2*B);
                temp_sum += wendu[j][k];
            }
            temp_data->temperature[j] = temp_sum/TEMP_SAMPLING_POINTS - temp_algo_data.temp_calib_dc[j];
//            rt_kprintf("temperature[%d]: %.3lf temp_v[j]:%.3lf temp_r[j]:%.3lf temp_sum:%.3lf\r\n",j,temp_data->temperature[j],temp_v[j],temp_r[j],temp_sum);

        }

//        rt_kprintf("sub :%.03lf \r\n",temp_data->temperature[0] -temp_data->temperature[1]);
    }
}

void get_ad7606_data()
{
    ad7606_data_t ad7606_temp_data;
    memcpy(&ad7606_temp_data,&ad7606_data,sizeof(ad7606_data));
    rt_kprintf("rms: %.3f  temperature[0]: %.3f  temperature[1]: %.3f \r\n",ad7606_temp_data.rms_value,ad7606_temp_data.temperature[0],ad7606_temp_data.temperature[1]);
}
MSH_CMD_EXPORT(get_ad7606_data, show ad7606 data);

void set_algo_data(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: set_algo_data algo_data_str\n");
        rt_kprintf("Like: set_algo_data lo_dc 100\n");
        rt_kprintf("Like: set_algo_data lo_ac show_value true_value\n");
        rt_kprintf("Like: set_algo_data temp1_dc 100\n");
        rt_kprintf("Like: set_algo_data temp2_dc 100\n");
        return ;
    }
    if(rt_strcmp(argv[1], "lo_dc") == 0)
    {
        //这里需要对输入值做一些合法性判断
        lo_algo_data.lo_calib_dc = atof(argv[2]);
    }
    else if(rt_strcmp(argv[1], "lo_ac") == 0)
    {
        //这里需要对输入值做一些合法性判断
        lo_algo_data.lo_calib_ac = (atof(argv[3])/atof(argv[2]));
    }
    else if (rt_strcmp(argv[1], "temp1_dc") == 0) {
        //这里需要对输入值做一些合法性判断
        temp_algo_data.temp_calib_dc[0] = atof(argv[2]);
    }
    else if (rt_strcmp(argv[1], "temp2_dc") == 0) {

        //这里需要对输入值做一些合法性判断
        temp_algo_data.temp_calib_dc[1] = atof(argv[2]);
    }
}
MSH_CMD_EXPORT(set_algo_data, set algo data);


void get_algo_data(int argc, char **argv)
{
    rt_kprintf("lo dc:%.3f \r\n",lo_algo_data.lo_calib_dc);

    rt_kprintf("lo ac:%.3f \r\n",lo_algo_data.lo_calib_ac);

    rt_kprintf("temp1 dc:%.3f \r\n",temp_algo_data.temp_calib_dc[0]);

    rt_kprintf("temp2 dc:%.3f \r\n",temp_algo_data.temp_calib_dc[1]);

}
MSH_CMD_EXPORT(get_algo_data, get algo data);

static int ad7606_init()
{
    rt_uint32_t period, pulse;
    //GPIO初始化
    rt_pin_mode(AD7606_CS_GET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(AD7606_CS_GET_PIN, PIN_HIGH);

    rt_pin_mode(AD7606_BUSY_GET_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(AD7606_BUSY_GET_PIN, PIN_IRQ_MODE_FALLING, ad7606_busy_irq, RT_NULL);

    rt_pin_mode(AD7606_CONVST_B_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(AD7606_CONVST_B_PIN, PIN_HIGH);

    //配置pwm
    period = 312500;         /* 频率为3200hz，单位为纳秒ns */
    pulse = 156250;          /* PWM脉冲宽度值，单位为纳秒ns */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    /* 设置PWM周期和脉冲宽度 */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
    /* 关闭设备通道 */
    rt_pwm_disable(pwm_dev,PWM_DEV_CHANNEL);

    //复位
    ad7606_stop();

    rt_sem_init(&data_rcv_sem, "data_rcv_sem", 0, RT_IPC_FLAG_FIFO);

    ad7606_tid = rt_thread_create("ad7606", ad7606_thread_entry, RT_NULL, 4096, 20, 4);
    if (ad7606_tid != RT_NULL)
        rt_thread_startup(ad7606_tid);

    return 1;
}
INIT_APP_EXPORT(ad7606_init);

#endif
