/*
 * OpenInterface.h
 * create_driver
 *
 * Created by Hazen Eckert on 04/14/12.
 * The Laboratory for Autonomous Robotics and Systems. 
 * The University of Texas at Dallas.
 *
 */ 
 
// This file contains the definitions for the Open Interface

///////////////////////////
//        Opcodes        //
///////////////////////////

#define CmdStart        128
#define CmdBaud         129
#define CmdControl      130
#define CmdSafe         131
#define CmdFull         132
#define CmdSpot         134
#define CmdClean        135
#define CmdDemo         136
#define CmdDrive        137
#define CmdMotors       138
#define CmdLeds         139
#define CmdSong         140
#define CmdPlay         141
#define CmdSensors      142
#define CmdDock         143
#define CmdPWMMotors    144
#define CmdDriveWheels  145
#define CmdOutputs      147
#define CmdStream	148
#define CmdSensorList   149
#define CmdPauseStream	150
#define CmdIRChar       151
#define CmdScript	152
#define CmdPlayScript	153
#define CmdShowScript	154
#define WaitForTime	155
#define WaitForDistance 156
#define WaitForAngle    157
#define WaitForEvent	158

//////////////////////////////////
//    Sensor Packet Offsets     // // (in packets 0, 5 and 6)
//////////////////////////////////

#define SenBumpDrop     0            
#define SenWall         1
#define SenCliffL       2
#define SenCliffFL      3
#define SenCliffFR      4
#define SenCliffR       5
#define SenVWall        6
#define SenOverC        7
#define SenIRChar       10
#define SenButton       11
#define SenDist1        12
#define SenDist0        13
#define SenAng1         14
#define SenAng0         15
#define SenChargeState  16
#define SenVolt1        17
#define SenVolt0        18
#define SenCurr1        19
#define SenCurr0        20
#define SenTemp         21
#define SenCharge1      22
#define SenCharge0      23
#define SenCap1         24
#define SenCap0         25
#define SenWallSig1     26
#define SenWallSig0     27
#define SenCliffLSig1   28
#define SenCliffLSig0   29
#define SenCliffFLSig1  30
#define SenCliffFLSig0  31
#define SenCliffFRSig1  32
#define SenCliffFRSig0  33
#define SenCliffRSig1   34
#define SenCliffRSig0   35
#define SenInputs       36
#define SenAInput1      37
#define SenAInput0      38
#define SenChAvailable  39
#define SenOIMode       40
#define SenOISong       41
#define SenOISongPlay   42
#define SenStreamPckts  43
#define SenVel1         44
#define SenVel0         45
#define SenRad1         46
#define SenRad0         47
#define SenVelR1        48
#define SenVelR0        49
#define SenVelL1        50
#define SenVelL0        51

// Sensor packet sizes
#define Sen0Size        26
#define Sen1Size        10
#define Sen2Size        6
#define Sen3Size        10
#define Sen4Size        14
#define Sen5Size        12
#define Sen6Size        52

// Sensor bit masks
#define WheelDropFront  0x10
#define WheelDropLeft   0x08
#define WheelDropRight  0x04
#define BumpLeft        0x02
#define BumpRight       0x01
#define BumpBoth        0x03
#define BumpEither      0x03
#define WheelDropAll    0x1C
#define ButtonAdvance   0x04
#define ButtonPlay      0x01


// LED Bit Masks
#define LEDAdvance       0x08
#define LEDPlay         0x02
#define LEDsBoth        0x0A

// OI Modes
#define OIPassive       1
#define OISafe          2
#define OIFull          3

// Drive radius special cases
#define RadStraight     32768
#define RadCCW          1
#define RadCW           -1






