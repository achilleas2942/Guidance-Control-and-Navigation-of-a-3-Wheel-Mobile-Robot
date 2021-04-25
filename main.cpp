#include "mbed.h"
#include "math.h"
#include "MPU9150.h"

/*******************************************************************************
    Mobile Robot
    Achilleas Shady Sheasha,
    3-5-2017
*******************************************************************************/

//******************************************************************************
//****************************Variabled*****************************************
//******************************************************************************
//****************************Global********************************************
//******************************************************************************
char            buffer[14];
                MPU9150 MPU9150;
Serial          pc(USBTX, USBRX);
uint8_t         whoami = MPU9150.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);
double          axmo = 0;
double          aymo = 0;
double          azmo = 0;
double          gxmo = 0;
double          gymo = 0;
double          gzmo = 0;
double          gzold = 0;
double          ayold = 0;
double          velyold = 0;
double          axold = 0;
double          velxold = 0;
double          theta = 0;
double          Tm = 9000;
double          Tpwm = 82000;
double          Tc = 85000;
double          mtime1;
Timer           mtime;
double          disy;
double          vely;
double          disx;
double          velx;

//******************************************************************************
//****************************Button & Led**************************************
//******************************************************************************
DigitalOut      led(LED1);
DigitalIn       button(PC_13);

//******************************************************************************
//****************************Left wheel****************************************
//******************************************************************************
DigitalOut      In1(PC_10);
DigitalOut      In2(PC_11);
PwmOut          EnA(PB_7);
double          leftwheel;

//******************************************************************************
//****************************Right wheel***************************************
//******************************************************************************
DigitalOut      In3(PA_15);
DigitalOut      In4(PA_14);
PwmOut          EnB(PA_13);
double          rightwheel;

//******************************************************************************
//****************************Ultrasound****************************************
//******************************************************************************
DigitalOut      ultraTrig(PC_6);
InterruptIn     ultraEcho(PC_8);
Timer           EchoTime;
Ticker          newdataTrig;
Ticker          UltaTickerTrig;
float           UltraDistance;
float           UltraEchoTime ;

//******************************************************************************
//****************************Functions*****************************************
//******************************************************************************
//****************************Ultrasound Trigstart******************************
//******************************************************************************
void UltraTrigStart()
{
    ultraTrig = 1;
    wait_us(12);
    ultraTrig = 0;
}

//******************************************************************************
//****************************Ultrasound Echostart******************************
//******************************************************************************
void UltraEchoStart()
{
    EchoTime.reset();
    EchoTime.start();
    led= 1;
}

//******************************************************************************
//****************************Ultrasound Echoend********************************
//******************************************************************************
void UltraEchoEnd()
{
    EchoTime.stop();
    UltraEchoTime = EchoTime.read();
    UltraDistance = UltraEchoTime * (340.0 * 50);
    led = 0;
}

//******************************************************************************
//****************************Forward*******************************************
//******************************************************************************
void forward()
{
    In1 = 1;
    In2 = 0;
    In3 = 1;
    In4 = 0;
    EnA.period_us(Tpwm);
    EnA.pulsewidth_us(leftwheel);
    EnB.period_us(Tpwm);
    EnB.pulsewidth_us(rightwheel);
}

//******************************************************************************
//****************************Left**********************************************
//******************************************************************************
void left()
{
    In1 = 0;
    In2 = 1;
    In3 = 1;
    In4 = 0;
    leftwheel = 0;
    rightwheel = 20000;
    EnA.period_us(Tpwm);
    EnA.pulsewidth_us(leftwheel);
    EnB.period_us(Tpwm);
    EnB.pulsewidth_us(rightwheel);
}

//******************************************************************************
//****************************Right*********************************************
//******************************************************************************
void right()
{
    In1 = 1;
    In2 = 0;
    In3 = 0;
    In4 = 1;
    leftwheel = 20000;
    rightwheel = 0;
    EnA.period_us(Tpwm);
    EnA.pulsewidth_us(leftwheel);
    EnB.period_us(Tpwm);
    EnB.pulsewidth_us(rightwheel);
}

//******************************************************************************
//****************************Stop**********************************************
//******************************************************************************
void stop()
{
    In1 = 0;
    In2 = 0;
    EnA = 0;
    In3 = 0;
    In4 = 0;
    EnB = 0;
}

//******************************************************************************
//****************************Compute Data**************************************
//******************************************************************************
void computedata()
{
    MPU9150.readAccelData(accelCount);
    ax = (float)accelCount[0]*aRes - axmo;
    ay = (float)accelCount[1]*aRes - aymo;
    az = (float)accelCount[2]*aRes - azmo;

    MPU9150.readGyroData(gyroCount);
    gx = (float)gyroCount[0]*gRes - gxmo;
    gy = (float)gyroCount[1]*gRes - gymo;
    gz = (float)gyroCount[2]*gRes - gzmo;    

    MPU9150.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);
    
    mtime1 = mtime.read();
    mtime.reset();
    mtime.start();
    theta = theta + (gz + gzold) * mtime1 / 2.0;
    gzold = gz;
    vely = vely + (ay + ayold) * mtime1 / 2.0;
    disy = disy + (vely + velyold) * mtime1 / 2.0;
    ayold = ay;
    velyold = vely;
    velx = velx + (ax + axold) * mtime1 / 2.0;
    disx = disx + (velx + velxold) * mtime1 / 2.0;
    axold = ax;
    velxold = velx;
}

//******************************************************************************
//****************************Control*******************************************
//******************************************************************************
void control()
{
    int         k1 = 1000;
    int         k2 = 1500;
    int         rightwheelmax = 50000;
    int         leftwheelmax = 50000;
    int         rightwheelmin = 10000;
    int         leftwheelmin = 10000;
    double      rightwheelvar = 30000;
    double      leftwheelvar = 30000;
    
    rightwheel = rightwheelvar + k2 * theta;
    leftwheel = leftwheelvar - k1 * theta;
    if(abs(theta)<1)
    {
        rightwheel = rightwheelvar;
        leftwheel = leftwheelvar;
    }
    if(theta>45)
    {
        rightwheel = rightwheelmax;
        leftwheel = leftwheelmin;
    }
    if(theta<-45)
    {
        leftwheel = leftwheelmax;
        rightwheel = rightwheelmin;
    }
    if(rightwheel>45)
    {
        rightwheel = rightwheelmax;
        leftwheel = leftwheelmin;
    }
    if(leftwheel>50)
    {
        rightwheel = rightwheelmin;
        leftwheel = leftwheelmax;
    }
}

//******************************************************************************
//****************************Main Function*************************************
//******************************************************************************
int main()
{
    //**************************************************************************
    //************************Variables*****************************************
    //**************************************************************************
    while(button);
    double      distancey = 12;
    double      distancex = 0;
    double      distance;
    double      dis;
    Timer       errortime;
    double      errortime1;
    Timer       time;
    double      time1;
    Timer       meter;
    double      meter1;
    int         reps = 0;
    int         state = 1;
    //**************************************************************************
    //************************Set Up Ultrasound*********************************
    //**************************************************************************
    UltaTickerTrig.attach(&UltraTrigStart, 0.2);
    ultraEcho.rise(&UltraEchoStart);
    ultraEcho.fall(&UltraEchoEnd);
    //**************************************************************************
    //************************Set Up MPU9150************************************
    //**************************************************************************
    pc.baud(9600);
    i2c.frequency(400000);
    if (whoami == 0x68)
    {
        led = 1;
        wait(1);
        MPU9150.MPU9150SelfTest(SelfTest);
        wait(1);
        MPU9150.resetMPU9150();
        MPU9150.calibrateMPU9150(gyroBias, accelBias);
        wait(1);
        MPU9150.initMPU9150();
        MPU9150.initAK8975A(magCalibration);
    }
    else
    {
        led = 0;
        while(1);
    }
    MPU9150.getAres();
    MPU9150.getGres();
    //**************************************************************************
    //************************Calculate Error***********************************
    //**************************************************************************
    errortime.start();
    errortime1 = errortime.read();
    while(errortime1<2)
    {
        errortime1 = errortime.read();
        if(MPU9150.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01)
        {
            MPU9150.readAccelData(accelCount);
            ax = (float)accelCount[0]*aRes;
            ay = (float)accelCount[1]*aRes;
            az = (float)accelCount[2]*aRes;
    
            MPU9150.readGyroData(gyroCount);
            gx = (float)gyroCount[0]*gRes;
            gy = (float)gyroCount[1]*gRes;
            gz = (float)gyroCount[2]*gRes;    
        
            MPU9150.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);
        }
        axmo = (ax+axmo*reps)/(reps+1);
        aymo = (ay+aymo*reps)/(reps+1);
        azmo = (az+azmo*reps)/(reps+1);
        gxmo = (gx+gxmo*reps)/(reps+1);
        gymo = (gy+gymo*reps)/(reps+1);
        gzmo = (gz+gzmo*reps)/(reps+1);
        reps = reps + 1;
    }
    //**************************************************************************
    //************************Main Calculations*********************************
    //**************************************************************************
    time.start();
    mtime.start();
    while(1)
    {
        computedata();
        theta = 0;
        dis = sqrt(pow(disx,2)+pow(disy,2));
        distance = sqrt(pow(distancex - disx,2)+pow(distancey + disy,2));
        printf("dis = %7.1f   distance = %7.1f  ultra = %7.1f \r\n", dis, distance, UltraDistance);
        switch(state)
        {
            case 1:
                while(dis<distance)
                {
                    forward();
                    if(UltraDistance>50)
                    {
                        time1 = time.read();
                        if(time1>Tm)
                        {
                            computedata();
                            Tm = time1 + Tm;
                        }
                        if(time1>Tc)
                        {
                            control();
                            Tc = time1 + Tc;
                        }
                        forward();
                    }
                    else
                    {
                        time.stop();
                        stop();
                        state = 2;
                        break;
                    }
                } 
            case 2:
                while(theta<90)
                {
                    right();
                }
                stop();
                state = 3;
                break;
            case 3:
                meter.reset();
                meter.start();
                meter1 = meter.read();
                time.start();
                while(meter1<3)
                {
                    meter1 = meter.read();
                    if(UltraDistance>50)
                    {
                        time1 = time.read();
                        if(time1>Tm)
                        {
                            computedata();
                            Tm = time1 + Tm;
                        }
                        if(time1>Tc)
                        {
                            control();
                            Tc = time1 + Tc;
                        }
                        forward();
                    }
                    else
                    {
                        stop();
                        state = 4;
                        break;
                    }
                }
            case 4:
                while(theta<(atan((((distancey + disy)/(distancex - disx))*180)/PI)))
                {
                    right();
                }
                stop();
                state = 1;
                break;
        }
    }
}