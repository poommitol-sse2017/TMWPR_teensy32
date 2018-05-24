/*
    Embeded Software Systems Project 2018, TGGS, KMUTNB   
    Platform Robot main files for Meccanum Platform Robot, 
    Author: Mix, Bekty
*/

/*
    Robot Commanding Equation:
    vx: velocity in X axis (m/s)
    vy: velocity in y axis (m/s)
    wz: angular velocity in z axis (rad/s)
    heading: desired robot heading direction (rad)

    PI/4 = 0.78539816339
    Wheel diameter = 0.1m (10cm)
    Wheel radius = 0.05m (10cm)

    wheel seperation width  (wsw) = 0.15 (30cm / 2)
    wheel seperation length (wsl) = 0.15 (30cm / 2)

                    ^
                    |  +X

            |<-    30cm    ->|

          \\\    FRONT     ///
    - FL  \\\--------------///   FR
    ^     \\\|            |///
    |        |            |
             |            |
    30cm     |            |     --> -Y
             |            |
    |        |------------|
    v     ///|            |\\\
    - RL  ///--------------\\\   RR
          ///              \\\

*/

#include <Arduino.h>
#include <configuration.h>

float current_pos[3] = {0, 0, 0};
float wheel_velo[4] = {0, 0, 0, 0}; //FL FR RL RR
float cmd[4] = {0, 0, 0, 90};
uint8_t data_out[14] = {0xff, 0xf2, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0c, 0x0c, 0x0d, 0x0d, 0x0d, 0x0d, 0xfe};
elapsedMicros loop_locking_time;

void wdtEnable() {
    // Setup WDT
    noInterrupts();                                         // don't allow interrupts while setting up WDOG
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);                                   // Need to wait a bit..
    // for this demo, we will use 1 second WDT timeout (e.g. you must reset it in < 1 sec or a boot occurs)
    WDOG_TOVALH = 0x006d;
    WDOG_TOVALL = 0xdd00;
    // This sets prescale clock so that the watchdog timer ticks at 7.2MHz
    WDOG_PRESC  = 0x400;
    // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
    WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
        WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
        WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
    interrupts();
}
void wdtReset(){
    noInterrupts();                                     //   No - reset WDT
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}

void command(float vx, float vy, float wz, float heading)
{
    /*
        check if vx and vy is zero or not to prevent divided by zero problem 
        which occured when using arctan function 
        EQ: theta = arctan(vy/vx)
        as the arduino math have theta range of -PI to PI, theta value have to be compensated.
        then compensated the actual angle from the heading parameter as well.
    */
    float theta, combined_ang, Vs;
    if (vx == 0)
        if (vy >= 0)
            theta = PI / 2;
        else
            theta = 3 * PI / 2;
    else if (vy == 0)
        if (vx >= 0)
            theta = 0;
        else
            theta = PI;
    else
        theta = atan2f(vy, vx);
    if (theta < 0)
        theta = (2 * PI) + theta;
    combined_ang = heading + theta;
    if (combined_ang > (2 * PI))
        combined_ang = combined_ang - (2 * PI);
    Vs = sqrt(pow(vx, 2) + pow(vy, 2));
    vx = Vs * cos(combined_ang);
    vy = Vs * sin(combined_ang);

    /* 
        Obstacle checking 
        make the velocity reduced to 0 when the system detected obstacle
        in the direction that the command issue.
    */ 
    if (obstacle[S_LEFT] && (vy < 0))
        vy = 0;
    if (obstacle[S_RIGHT] && (vy > 0))
        vy = 0;
    if (obstacle[S_FRONT] && (vx > 0))
        vx = 0;
    if (obstacle[S_REAR] && (vx < 0))
        vx = 0;
    /*
        Convert The velocity into Wheel Rotation speed.
        equation: 
        wheel Front Left = (1/wheel_radius) * (vx - vy - (wsw + wsl) * wz
        wheel Front Left = (1/wheel_radius) * (vx - vy - (wsw + wsl) * wz
        wheel Front Left = (1/wheel_radius) * (vx - vy - (wsw + wsl) * wz
        wheel Front Left = (1/wheel_radius) * (vx - vy - (wsw + wsl) * wz
    */
    wheel_velo[M_FL] = 190.98593171 * (vx - vy - wz * 0.3);
    wheel_velo[M_FR] = 190.98593171 * (vx + vy + wz * 0.3);
    wheel_velo[M_RL] = 190.98593171 * (vx + vy - wz * 0.3);
    wheel_velo[M_RR] = 190.98593171 * (vx - vy + wz * 0.3);
}

void runMotor()
{
    motorFL.computeSpeed();
    motorFL.runRPM(wheel_velo[M_FL]);
    motorFR.computeSpeed();
    motorFR.runRPM(wheel_velo[M_FR]);
    motorRL.computeSpeed();
    motorRL.runRPM(wheel_velo[M_RL]);
    motorRR.computeSpeed();
    motorRR.runRPM(wheel_velo[M_RR]);
}

void brakeMotor()
{
    motorFL.computeSpeed();
    motorFL.brake();
    motorFR.computeSpeed();
    motorFR.brake();
    motorRL.computeSpeed();
    motorRL.brake();
    motorRR.computeSpeed();
    motorRR.brake();
}

void sendData()
{
    int16_t comp = (int16_t)(comps_yaw * 10);
    uint8_t comp_low = comp & 0x00FF;
    uint8_t comp_high = (comp & 0xFF00) >> 8;
    data_out[3] = (int8_t)(((float)distance[0] / 255) * 255); //Sonar FL
    data_out[4] = (int8_t)(((float)distance[1] / 255) * 255); //Sonar FR
    data_out[5] = (int8_t)(((float)distance[2] / 255) * 255); //Sonar RL
    data_out[6] = (int8_t)(((float)distance[3] / 255) * 255); //Sonar RR
    data_out[7] = comp_high;
    data_out[8] = comp_low;
    data_out[9] = (int8_t)motorFL.getSpeedRPM();
    data_out[10] = (int8_t)motorFR.getSpeedRPM();
    data_out[11] = (int8_t)motorRL.getSpeedRPM();
    data_out[12] = (int8_t)motorRR.getSpeedRPM();
    uint8_t datacrc[13];
    for (int i = 0; i < 13; i++)
    {
        datacrc[i] = data_out[i];
    }
    data_out[13] = compute_crc8(datacrc, 13);
    for (int i = 0; i < 14; i++)
    {
        Serial1.write(data_out[i]);
    }
}

void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    // Build the CRC8 table for data validation
    calulatetable_crc8();
    motion_init();
    // Serial1 connected to Xbee, for communication 
    Serial1.begin(19200);
    // Serial2 connected to TTL-485 Module, for polling SONAR sensors
    Serial2.begin(19200);
    // I2C connected to compass
    Wire.begin(); //S initialize i2c for compass
    
    // set PID gain for motor
    motorFL.setGain(0.5, 0.3, 0.03);
    motorFR.setGain(0.5, 0.3, 0.03);
    motorRL.setGain(0.5, 0.3, 0.03);
    motorRR.setGain(0.6, 0.35, 0.03);
    // make sure it stop after bootup
    brakeMotor();
    delay(2000);
    // turn led on, to indicated that the robot is ready!
    digitalWrite(13, HIGH);
    wdtEnable();
}

void loop()
{
    getDistance();
    getYaw();
    command(cmd[0], cmd[1], cmd[2], cmd[3]);
    runMotor();
    sendData();
  
    while (loop_locking_time < 60000);
    loop_locking_time = 0;
}

/* 
    This function supposed to be interrupt handle by arduino framework, but it is not, this is a polling function
    The function is to recieve command from Operator
*/
void serialEvent1()
{
    if (Serial1.available())
    {
        data[ind_data] = Serial1.read();
        if ((ind_data >= 11) && (data[ind_data - 11] == 0xff) && (data[ind_data - 10] == 0xf2))
        {
            int indice = ind_data - 11;
            byte data_valid[11];
            for (int i = 0; i < 12; i++)
            {
                data_correct[i] = data[indice + i];
                if (i < 11)
                    data_valid[i] = data[indice + i];
            }
            if (compute_crc8(data_valid, 11) == data_correct[11])
                dataready = true;
            ind_data = 0;
        }
        else if ((ind_data >= 50))
        {
            cmd[0] = 0;
            cmd[1] = 0;
            cmd[2] = 0;
            cmd[3] = 0;
            ind_data = 0;
        }
        ind_data++;
    }
    if (dataready)
    {
        dataready = false;
        uint16_t vx = 0, vy = 0, wz = 0, ref = 0;
        vx = data_correct[3];
        vx = vx << 8;
        vx = vx | data_correct[4];
        vy = data_correct[5];
        vy = vy << 8;
        vy = vy | data_correct[6];
        wz = data_correct[7];
        wz = wz << 8;
        wz = wz | data_correct[8];
        ref = data_correct[9];
        ref = ref << 8;
        ref = ref | data_correct[10];
        cmd[0] = (float)((int16_t)vx) / 10000; // vx in m/s
        cmd[1] = (float)((int16_t)vy) / 10000; // vy in m/s
        cmd[2] = (float)((int16_t)wz) / 10000; // wz in rad/s
        cmd[3] = ((float)((int16_t)ref) / 10) * (PI / 180);
        // reset the wdt after the corrected data package received
        wdtReset();
    }
}