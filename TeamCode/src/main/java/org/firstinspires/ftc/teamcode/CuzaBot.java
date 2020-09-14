package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.Sensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


//Clasa adaugata pentru a implementa o singura data fiecare piesa adaugata pe robot

public class CuzaBot  {
    public DcMotor leftMotorFront = null;
    public DcMotor leftMotorBack = null;
    public DcMotor rightMotorFront = null;
    public DcMotor rightMotorBack = null;
    public DcMotor launchMotor = null;

    public ElapsedTime runtime = new ElapsedTime();

    public ColorSensor ColorSensor = null;
    public Rev2mDistanceSensor DistanceSensor = null;

    HardwareMap hwMap = null;

    public void initializeAll(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftMotorFront = hwMap.dcMotor.get("leftFront");
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorFront.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront.setPower(0);

        leftMotorBack = hwMap.dcMotor.get("leftBack");
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorBack.setPower(0);

        rightMotorFront = hwMap.dcMotor.get("rightFront");
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setPower(0);

        rightMotorBack = hwMap.dcMotor.get("rightBack");
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);
        rightMotorBack.setPower(0);

        launchMotor = hwMap.dcMotor.get("launchMotor");
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setDirection(DcMotor.Direction.REVERSE);
        launchMotor.setPower(0);

        ColorSensor = hwMap.get(ColorSensor.class,"ColorSensor");
        DistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "DistanceSensor");
    }
}
