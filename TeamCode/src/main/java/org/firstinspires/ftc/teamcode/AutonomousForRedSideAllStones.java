package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomousForRedSideWithStonesAndTray", group="Autonomous")

public  class AutonomousForRedSideAllStones extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private Servo trayservo1 = null;
    private Servo trayservo2 = null;

    private Servo lowarmUp = null;
    private Servo lowarmDown = null;

    ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");

    public void moveForward(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void moveBackward(long time, double power) {

        power -= power;

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void moveLeft(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);

        sleep(time);
    }

    public void moveRight(long time, double power) {

        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(-power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void turnLeft(long time, double power) {

        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);

        sleep(time);
    }

    public void turnRight(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void grabStone() {

        lowarmUp.setPosition(0.25);
        lowarmDown.setPosition(-1);


    }

    public void releaseStone(){

        lowarmUp.setPosition(-0.5);
        lowarmDown.setPosition(1);
    }

    public void grabtray(){

        trayservo1.setPosition(1);
        trayservo2.setPosition(1);

    }

    public void opentray(){

        trayservo1.setPosition(0);
        trayservo2.setPosition(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rigthFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        lowarmUp = hardwareMap.servo.get("lowarmup");
        lowarmDown = hardwareMap.servo.get("lowarmdown");

        trayservo1 = hardwareMap.servo.get("trayservo1");
        trayservo2 = hardwareMap.servo.get("trayservo2");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lowarmUp.setPosition(-0.5);
        lowarmDown.setPosition(1);
        trayservo1.setPosition(0);
        trayservo2.setPosition(0);


        while(opModeIsActive()) {

            int CurrentPosition=1;
            int StoneTime = 150;

            while(CurrentPosition<=6){

                moveForward(650,1);
                grabStone();
                moveBackward(500,1);
                moveRight(1200+CurrentPosition*StoneTime,1);
                releaseStone();
                if(CurrentPosition == 6)
                    break;
                moveLeft(1200+CurrentPosition*StoneTime,1);
                CurrentPosition=CurrentPosition+1;


            }

            moveBackward(400,1);
            moveRight(700,1);
            moveForward(450,1);
            grabtray();
            turnRight(600,1);
            moveForward(500,1);
            opentray();
            moveBackward(1100,1);







        }


    }


}







