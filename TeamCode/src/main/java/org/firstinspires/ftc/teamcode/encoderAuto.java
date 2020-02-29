package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="encoderAuto", group="")
public class encoderAuto extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    static final int MOTOR_TICK_COUNT = 1120;
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

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //move 20 inches

        double circumferinta = 3.14*3; //pi*diametru
        double rotationsNeeded = 20/circumferinta;
        int target = (int)(rotationsNeeded*1120);

        leftFrontMotor.setTargetPosition(target);
        leftRearMotor.setTargetPosition(target);
        rightFrontMotor.setTargetPosition(target);
        rightRearMotor.setTargetPosition(target);

        leftFrontMotor.setPower(.5);
        leftRearMotor.setPower(.5);
        rightFrontMotor.setPower(.5);
        rightRearMotor.setPower(.5);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || rightRearMotor.isBusy() || leftRearMotor.isBusy()){

            telemetry.addData("Lf", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Rf", rightFrontMotor.getCurrentPosition());
            telemetry.addData("Lr", leftRearMotor.getCurrentPosition());
            telemetry.addData("Rr", rightRearMotor.getCurrentPosition());

            telemetry.update();

        }

        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);


    }

    }