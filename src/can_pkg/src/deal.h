#ifndef _DEAL_H_
#define _DEAL_H_

#include <signal.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
 


#include "iostream"
#include "math.h"
#include "fstream"

//多话题订阅相关头文件
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#define TIME_STEP 32    //时钟
#define NMOTORS 4       //电机数量
#define MAX_SPEED 10   //电机最大速度
using namespace std;
using namespace message_filters;
ofstream ofs1;
double speeds[NMOTORS]={0.0,0.0,0.0,0.0};// 四电机速度值 0～10
double angles[NMOTORS]={0.0,0.0,0.0,0.0};// 四电机角度值 -180～180


//ros::Time t_now=ros::Time::now(),t_last=ros::Time::now();
ros::Time t_now,t_last;
bool first_flag=1;
bool stop_flag=0;
double change_time;
#define radius 0.1
#define PI 3.14159
#define L 1.110
#define D 0.694
extern  void updateSpeed();
double time_last_lf;
double time_last_lb;
double time_last_rf;
double time_last_rb;

struct POSITION{
    float pos_x=0;
    float pos_y=0;
    float angle=0;
} posotion;
struct vtospeed_{
    double speed_Vlf;
    double speed_Vlb;
    double speed_Vrf;
    double speed_Vrb;
};vtospeed_ myvtospeed_;
struct Speed_sen{
    double speed_lf;
    double speed_lb;
    double speed_rf;
    double speed_rb;
};

struct Pos_sen{
    double pos_lf;
    double pos_lb;
    double pos_rf;
    double pos_rb;
};

struct Pos{
    double pos_now;
    double pos_last;
};

Pos pos_lf;
Pos pos_rf;
Pos pos_lb;
Pos pos_rb;

struct SEN_DATA{
    double vx;
    double vy;
    double w;
}sen_data;

Speed_sen  speed_sen;
Pos_sen    pos_sen;

double Rlf,Rlb,Rrf,Rrb;
double Alf,Alb,Arf,Arb;
double Vlf,Vlb,Vrf,Vrb;

double vx,vy,w;

void vtospeed(double Vlf,double Vlb,double Vrf,double Vrb,vtospeed_ &myvtospeed){
        myvtospeed.speed_Vlf = (Vlf+1)*100;
        myvtospeed.speed_Vlb = (Vlb+1)*100;
        myvtospeed.speed_Vrf = (Vrf+1)*100;
        myvtospeed.speed_Vrb = (Vrb+1)*100;
}
void deal1(double &vx,double &vy,double &w,vtospeed_ &myvtospeed)
    {
        float v = sqrt(pow(vx, 2) + pow(vy, 2));
        float R = -v / w;
        float A = atan2(vy, vx);
        double b = R * sin(A);
        double a = R * cos(A);
        double len_car=L;   double wid_car=D;

        cout<<"jinrupingyi"<<endl;
        Alf = A;
        Alb = A;
        Arf = A;
        Arb = A;

        Vlf = sqrt(pow(vx,2)+pow(vy,2));
        Vlb = sqrt(pow(vx,2)+pow(vy,2));
        Vrf = sqrt(pow(vx,2)+pow(vy,2));
        Vrb = sqrt(pow(vx,2)+pow(vy,2)); 

        vtospeed(Vlf, Vlb, Vrf, Vrb, myvtospeed);
    }
void deal2(double &vx,double &vy,double &w,vtospeed_ &myvtospeed)
    {
            if(w>-0.0001 && w <0.0001)     {w=0.00001;}
            if(vx>-0.0001 && vx <0.0001)    {vx=0.00001;}
            if(vy>-0.0001 && vy <0.0001)    {vy=0.00001;}

            cout<<"jinruzixuan"<<endl;
            float v = sqrt(pow(vx, 2) + pow(vy, 2));
            float R = -v / w;
            float A = atan2(vy, vx);
            double b = R * sin(A);
            double a = R * cos(A);
            double len_car=L;   double wid_car=D;

            Alf = -atan2((b + len_car / 2), (a + wid_car / 2));
            Alb = -atan2((b - len_car / 2), (a + wid_car / 2));
            Arf = -atan2((b + len_car / 2), (a - wid_car / 2));
            Arb = -atan2((b - len_car / 2), (a - wid_car / 2));

            Vlf = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R * v;
            Vlb = sqrt((a + wid_car / 2) * (a + wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R * v;
            Vrf = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b + len_car / 2) * (b + len_car / 2)) / R * v;
            Vrb = sqrt((a - wid_car / 2) * (a - wid_car / 2) + (b - len_car / 2) * (b - len_car / 2)) / R * v;


            if      (1.57<Alf && Alf<4.71)   {Alf=Alf-3.14;Vlf=-Vlf;}
            else if (4.71<Alf && Alf<6.28)   {Alf=Alf-6.28;}
            else if (-1.57>Alf&& Alf>-4.71)   {Alf=Alf+3.14;Vlf=-Vlf;}
            else if(-4.71>Alf && Alf>-6.28)   {Alf=Alf+6.28;}

            if      (1.57<Alb && Alb<4.71)   {Alb=Alb-3.14;Vlb=-Vlb;}
            else if (4.71<Alb && Alb<6.28)   {Alb=Alb-6.28;}
            else if (-1.57>Alb&& Alb>-4.71)   {Alb=Alb+3.14;Vlb=-Vlb;}
            else if(-4.71>Alb && Alb>-6.28)   {Alb=Alb+6.28;}

            if      (1.57<Arf && Arf<4.71)   {Arf=Arf-3.14;Vrf=-Vrf;}
            else if (4.71<Arf && Arf<6.28)   {Arf=Arf-6.28;}
            else if (-1.57>Arf&& Arf>-4.71)   {Arf=Arf+3.14;Vrf=-Vrf;}
            else if(-4.71>Arf && Arf>-6.28)   {Arf=Arf+6.28;}

            if      (1.57<Arb && Arb<4.71)   {Arb=Arb-3.14;Vrb=-Vrb;}
            else if (4.71<Arb && Arb<6.28)   {Arb=Arb-6.28;}
            else if (-1.57>Arb&& Arb>-4.71)   {Arb=Arb+3.14;Vrb=-Vrb;}
            else if(-4.71>Arb && Arb>-6.28)   {Arb=Arb+6.28;}
          
       
            vtospeed(Vlf, Vlb, Vrf, Vrb, myvtospeed);
    }
void deal3(double &vx, double &vy, double &w, vtospeed_ &myvtospeed)
    {
        if(w>0){
            float R=vx/w;
            float Rlf,Rrf,Rlb,Rrb;
            cout<<"jinruakm"<<endl;
            Rlf = sqrt(pow(L / 2, 2) + pow(R - D / 2, 2));
            Rrf = sqrt(pow(L / 2, 2) + pow(R + D / 2, 2));
            Rlb = sqrt(pow(L / 2, 2) + pow(R - D / 2, 2));
            Rrb = sqrt(pow(L / 2, 2) + pow(R + D / 2, 2));

            Vlf=w*Rlf;
            Vlb=w*Rlb;
            Vrf=w*Rrf;
            Vrb=w*Rrb;

            Alb = -atan2(L/2, R-D/2);
            Alf=-Alb;
            Arb = -atan2(L/2, R+D/2);
            Arf=-Arb;

            vtospeed(Vlf, Vlb, Vrf, Vrb, myvtospeed);
        }
        if(w<0){
            float R = -vx / w;
            float Rlf, Rrf, Rlb, Rrb;

            Rlf = sqrt(pow(L / 2, 2) + pow(R + D / 2, 2));
            Rrf = sqrt(pow(L / 2, 2) + pow(R - D / 2, 2));
            Rlb = sqrt(pow(L / 2, 2) + pow(R + D / 2, 2));
            Rrb = sqrt(pow(L / 2, 2) + pow(R - D / 2, 2));
            
            Vlf = -w * Rlf;
            Vlb = -w * Rlb;
            Vrf = -w * Rrf;
            Vrb = -w * Rrb;

            Alb = atan2(L / 2, R + D / 2);
            Alf = -Alb;
            Arb = atan2(L / 2, R - D / 2);
            Arf = -Arb;
            vtospeed(Vlf, Vlb, Vrf, Vrb, myvtospeed);
        }

    }
void deal_back(Speed_sen  speed_sen,Pos_sen pos_sen){

    double V = (speed_sen.speed_lb+speed_sen.speed_lf+speed_sen.speed_rb+speed_sen.speed_rf)/4;
    double theta=(pos_sen.pos_lb+pos_sen.pos_lf+pos_sen.pos_rb+pos_sen.pos_rf)/4;
    sen_data.vx = V *cos(theta);
    sen_data.vy = V *sin(theta);
    sen_data.w =0;
    t_now = ros::Time::now();
    double timing = t_now.toSec() - t_last.toSec();
    if(timing>5)    {timing=0;}
    t_last = ros::Time::now();

    if (sen_data.vx == sen_data.vx)
    {
          posotion.pos_x = posotion.pos_x + sen_data.vx * timing*cos( posotion.angle)+sen_data.vy * timing*sin( posotion.angle);
    }
    if (sen_data.vy == sen_data.vy)
    {
        posotion.pos_y = posotion.pos_y + sen_data.vy * timing*cos( posotion.angle)-sen_data.vx*timing*sin(posotion.angle);
    }
       if (sen_data.w == sen_data.w)
    {
         posotion.angle =  posotion.angle + sen_data.w*timing;
    }
}
void deal_back2(Speed_sen speed_sen,Pos_sen pos_sen){
    float R = sqrt(pow(L/2, 2) + pow(D/2, 2));
    sen_data.w =-(6.28*R/speed_sen.speed_lf)*6.28;
    sen_data.vx = 0;
    sen_data.vy = 0;

    t_now = ros::Time::now();
    double timing = t_now.toSec() - t_last.toSec();
    if(timing>5)    {timing=0;}
    t_last = ros::Time::now();
  
    while(posotion.angle>3.141592653 || posotion.angle<-3.141592653 ){
        if(posotion.angle>3.141592653 ) {posotion.angle-=6.2831853;}
        if(posotion.angle<-3.141592653 ) {posotion.angle+=6.2831853;}
    }
    if (sen_data.w == sen_data.w)
    {
         posotion.angle =  posotion.angle + sen_data.w*timing;
    }
    if (sen_data.vx == sen_data.vx)
    {
          posotion.pos_x = posotion.pos_x + sen_data.vx * timing*cos( posotion.angle)+sen_data.vy * timing*sin( posotion.angle);
    }
    if (sen_data.vy == sen_data.vy)
    {
        posotion.pos_y = posotion.pos_y + sen_data.vy * timing*cos( posotion.angle)-sen_data.vx*timing*sin(posotion.angle);
    }
    
}
void deal_back3(Speed_sen speed_sen,Pos_sen pos_sen){
    if (pos_sen.pos_lf > 0)
    {
        sen_data.vx = abs(cos(pos_sen.pos_lf) * speed_sen.speed_lf);
        sen_data.vy = 0;

        float R=L/(2*tan(-pos_sen.pos_lb))+D/2;
        sen_data.w = sen_data.vx / R;
    }
    if (pos_sen.pos_lf < 0)
    {
        sen_data.vx = abs(cos(-pos_sen.pos_lf) * speed_sen.speed_lf);
        sen_data.vy = 0;

        float R=L/(2*tan(-pos_sen.pos_lb))-D/2;
        sen_data.w = -sen_data.vx / R;
    }

    t_now = ros::Time::now();
    double timing = t_now.toSec() - t_last.toSec();
    if(timing>5)    {timing=0;}
    t_last = ros::Time::now();
  
    while(posotion.angle>3.141592653 || posotion.angle<-3.141592653 ){
        if(posotion.angle>3.141592653 ) {posotion.angle-=6.2831853;}
        if(posotion.angle<-3.141592653 ) {posotion.angle+=6.2831853;}
    }
    if (sen_data.w == sen_data.w)
    {
         posotion.angle =  posotion.angle + sen_data.w*timing;
    }
    if (sen_data.vx == sen_data.vx)
    {
          posotion.pos_x = posotion.pos_x + sen_data.vx * timing*cos( posotion.angle)+sen_data.vy * timing*sin( posotion.angle);
    }
    if (sen_data.vy == sen_data.vy)
    {
        posotion.pos_y = posotion.pos_y + sen_data.vy * timing*cos( posotion.angle)-sen_data.vx*timing*sin(posotion.angle);
    }
    
}




#endif