package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomousRedTray", group="Autonomous")

public  class AutonomousRedTray extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private Servo trayservo1 = null;
    private Servo trayservo2 = null;

    private Servo lowarmUp = null;
    private Servo lowarmDown = null;



    public void moveForward(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void moveBackward(long time, double power) {

        power = -power;

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void moveLeft(long time, double power) {

        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);

        sleep(time);
    }

    public void moveRight(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void turnLeft(long time, double power) {

        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(-power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void turnRight(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);

        sleep(time);
    }

    public void grabStone() {

        lowarmUp.setPosition(0.7);
        lowarmDown.setPosition(-1);
        sleep(300);

        lowarmDown.setPosition(-0.7);

    }

    public void releaseStone(){

        lowarmUp.setPosition(-0.5);
        lowarmDown.setPosition(1);
    }

    public void grabtray(){

        trayservo1.setPosition(0.1);
        trayservo2.setPosition(0.1);

    }

    public void opentray(){

        trayservo1.setPosition(1);
        trayservo2.setPosition(1);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
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

        trayservo1.setDirection(Servo.Direction.FORWARD);
        trayservo2.setDirection(Servo.Direction.REVERSE);

        lowarmUp.setPosition(1);
        lowarmDown.setPosition(-0.5);
        trayservo1.setPosition(0.5);
        trayservo2.setPosition(0.5);


        waitForStart();



        moveRight(500,1);
        moveForward(1410,0.5);
        moveForward(300,0.1);
        moveForward(650,0);
        grabtray();
        moveForward(500,0);
        turnRight(500,0.5);
        moveForward(800,0);
        moveBackward(1320,0.7);
        turnRight(1600,1);
        moveForward(800,1);
        moveForward(400,0);

        opentray();
        moveForward(350,0);
        moveRight(500,1);
        moveBackward(1000,0.5);












    }


}







