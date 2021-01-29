package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.OpModes.BNO055IMU;
import org.firstinspires.ftc.teamcode.Systems.Vision.EasyOpenVision;

public class Robot {
    /*HARDWARE VARIABLES*/

    //WHEELS:
    public static DcMotor leftMotorFront = null;
    public static DcMotor leftMotorRear = null;
    public static DcMotor rightMotorFront = null;
    public static DcMotor rightMotorRear = null;

    //HARDWARE OBJECTS:
    public static Mechanisms mechanisms = new Mechanisms();
    public static EasyOpenVision vision = new EasyOpenVision();

    //IMU SENSOR:
    private static BNO055IMU imu;
    private static Orientation angles;

    //DRIVE TRAIN VARIABLES:

    public static double robotDimensions = 18; //INCHES
    public static double gearRatio = 60;
    public static double wheelDiameter = 3.9;
    public static double wheelCirc = (Math.PI * wheelDiameter);
    public static double TicksPerRev = 1440;
    public static double POSITION_RATIO = 144/700;
}
