#ifndef _LVBO_H_
#define _LVBO_H_

#include <stdio.h>
#include <string.h>
#include "deal.h"

/************************ 滑动窗口滤波器 *****************************/
#define     window_size 5            //滑动窗口长度
double buffer_lf[window_size] = {0}; //滑动窗口数据buf
double buffer_rf[window_size] = {0}; //滑动窗口数据buf
double buffer_lb[window_size] = {0}; //滑动窗口数据buf
double buffer_rb[window_size] = {0}; //滑动窗口数据buf
#define FILTER_N 5

/*********************** 滑动窗口滤波函数 ****************************/
Pos_sen sliding_average_filter(Pos_sen pos_sen)
{
  static int data_num = 0;
  Pos_sen output;

  if (data_num < window_size) //不满窗口，先填充
  {
      buffer_lf[data_num++] = pos_sen.pos_lf;
      buffer_lb[data_num++] = pos_sen.pos_lb;
      buffer_rf[data_num++] = pos_sen.pos_rf;
      buffer_rb[data_num++] = pos_sen.pos_rb;
      output = pos_sen; // 返回相同的值
  }
  else
  {
      for(int i=0;i<window_size-1;i++){
        buffer_lf[i] = buffer_lf[i + 1];
        buffer_lb[i] = buffer_lb[i + 1];
        buffer_rf[i] = buffer_rf[i + 1];
        buffer_rb[i] = buffer_rb[i + 1];
      }
      buffer_lf[window_size-1] = pos_sen.pos_lf;
      buffer_lb[window_size-1] = pos_sen.pos_lb;
      buffer_rf[window_size-1] = pos_sen.pos_rf;
      buffer_rb[window_size-1] = pos_sen.pos_rb;

      int i, j;
      float filter_temp, filter_sum_POS_lf = 0, filter_sum_POS_lb = 0, filter_sum_POS_rf = 0,
                         filter_sum_POS_rb = 0;
  

   for(j = 0; j < FILTER_N - 1; j++)
   {
       for(i = 0; i < FILTER_N - 1 - j; i++)
       {
          if(buffer_lf[i] > buffer_lf[i + 1])
          {
              filter_temp = buffer_lf[i];
              buffer_lf[i] = buffer_lf[i + 1];
              buffer_lf[i + 1] = filter_temp;
          }
       }
   }
      for(j = 0; j < FILTER_N - 1; j++)
   {
       for(i = 0; i < FILTER_N - 1 - j; i++)
       {
          if(buffer_lb[i] > buffer_lb[i + 1])
          {
              filter_temp = buffer_lb[i];
              buffer_lb[i] = buffer_lb[i + 1];
              buffer_lb[i + 1] = filter_temp;
          }
       }
   }
         for(j = 0; j < FILTER_N - 1; j++)
   {
       for(i = 0; i < FILTER_N - 1 - j; i++)
       {
          if(buffer_rf[i] > buffer_rf[i + 1])
          {
              filter_temp = buffer_rf[i];
              buffer_rf[i] = buffer_rf[i + 1];
              buffer_rf[i + 1] = filter_temp;
          }
       }
   }

      for(j = 0; j < FILTER_N - 1; j++)
   {
       for(i = 0; i < FILTER_N - 1 - j; i++)
       {
          if(buffer_rb[i] > buffer_rb[i + 1])
          {
              filter_temp = buffer_rb[i];
              buffer_rb[i] = buffer_rb[i + 1];
              buffer_rb[i + 1] = filter_temp;
          }
       }
   }
   // 去除最大最小极值后求平均
    for(i = 1; i < FILTER_N - 1; i++) 
    {
        filter_sum_POS_lf+= buffer_lf[i];
        filter_sum_POS_lb+= buffer_lb[i];
        filter_sum_POS_rf+= buffer_rf[i];
        filter_sum_POS_rb+= buffer_rb[i];
    }
    output.pos_lb=filter_sum_POS_lb;
    output.pos_lf=filter_sum_POS_lf;
    output.pos_rb=filter_sum_POS_rb;
    output.pos_rf=filter_sum_POS_rf;
  }

  return output;
}


#endif