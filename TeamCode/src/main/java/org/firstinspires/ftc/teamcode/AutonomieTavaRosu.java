package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomieTava", group="Autonomous")

public  class AutonomieTavaRosu extends LinearOpMode {

    int Motor_Tick_Counts = 1120;
    double Circumference = 3.1415*3.93701;
    int SkyStonePosition;
    double tilesize = 22.75;


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

        moveForward(150, 1);
        lowarmUp.setPosition(0.25);
        lowarmDown.setPosition(-1);
        moveBackward(300,1);

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

        moveForward(1500,0.5);
        moveForward(800,0);
        moveRight(500,0.2);
        moveForward(300,0);
        grabtray();
        moveBackward(1500,0.3);
        turnRight(900,0.3);
        moveBackward(1400,0.3);














    }


}







