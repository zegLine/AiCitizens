package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="ParcarePodAlbastru", group="Autonomous")
public class ParcarePod extends LinearOpMode {

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
    public void runOpMode() {

        leftFrontMotor= hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor= hardwareMap.dcMotor.get("rightFront");
        leftRearMotor= hardwareMap.dcMotor.get("leftRear");
        rightRearMotor= hardwareMap.dcMotor.get("rightRear");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        moveForward(1800,0.3);
        moveRight(3200,0.4);









    }

}
