package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//Clasa adaugata pentru a implementa o singura data fiecare piesa adaugata pe robot

enum Directie {
    FORWARD,
    REVERSE,
    LEFT,
    RIGHT
}

public class CuzaBot  {
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    private DcMotor leftMotorFront = null;
    private DcMotor leftMotorBack = null;
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorBack = null;
    private DcMotor intakeMotor = null;

    private double LF, LB, RF, RB, IM;
    public ElapsedTime runtime = new ElapsedTime();

    public ColorSensor ColorSensor = null;
    public Rev2mDistanceSensor DistanceSensor = null;

    HardwareMap hwMap = null;

    public void initializeImu(HardwareMap ahwMap) {
        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.getLinearAcceleration();
    }

    public void initializeAll(HardwareMap ahwMap) {

        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        leftMotorFront = hwMap.dcMotor.get("leftFront");
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorFront.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront.setPower(0);

        leftMotorBack = hwMap.dcMotor.get("leftRear");
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorBack.setPower(0);

        rightMotorFront = hwMap.dcMotor.get("rightFront");
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setPower(0);

        rightMotorBack = hwMap.dcMotor.get("rightRear");
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);
        rightMotorBack.setPower(0);

        intakeMotor = hwMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setPower(0);

        runtime = new ElapsedTime();
    }

    public void powerIntake(double power) {
        double finalPower = Math.max(power, 0.8);
        intakeMotor.setPower(power);
    }

    public void setMotorPowers(double LF, double LB, double RF, double RB) {
        this.LF = LF;
        this.LB = LB;
        this.RF = RF;
        this.RB = RB;
    }

    public void powerMotors(double leftY, double leftX, double rightX) {
        LF = leftY + leftX + rightX;
        RF = leftY - leftX - rightX;
        LB = leftY - leftX + rightX;
        RB = leftY + leftX - rightX;
        powerMotors(LF, LB, RF, RB);
    }

    public void powerMotors(double leftMotorFrontPow, double leftMotorBackPow,
                            double rightMotorFrontPow, double rightMotorBackPow) {
        leftMotorFront.setPower(leftMotorFrontPow);
        leftMotorBack.setPower(leftMotorBackPow);
        rightMotorFront.setPower(rightMotorFrontPow);
        rightMotorBack.setPower(rightMotorBackPow);
    }

    public void powerMotors(double leftPow, double rightPow) {
        leftMotorFront.setPower(leftPow);
        leftMotorBack.setPower(leftPow);
        rightMotorFront.setPower(rightPow);
        rightMotorBack.setPower(rightPow);
    }

    public void powerMotors() {
        leftMotorFront.setPower(LF);
        leftMotorBack.setPower(LB);
        rightMotorFront.setPower(RF);
        rightMotorBack.setPower(RB);
    }

    public void getImuAngles() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public String composeTelemetry() {
        StringBuilder telemetry = new StringBuilder();
        telemetry.append("First angle: " + angles.firstAngle + "\n")
                .append("Second Angle: " + angles.secondAngle + "\n")
                .append("Third angle: " + angles.thirdAngle + "\n");
        return telemetry.toString();
    }
}
