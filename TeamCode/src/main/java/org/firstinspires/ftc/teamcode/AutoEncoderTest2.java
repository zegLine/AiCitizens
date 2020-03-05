package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ENCDR_TST2", group="")
public class AutoEncoderTest2 extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    @Override
    public void runOpMode() {

        leftFrontMotor= hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor= hardwareMap.dcMotor.get("rightFront");
        leftRearMotor= hardwareMap.dcMotor.get("leftRear");
        rightRearMotor= hardwareMap.dcMotor.get("rightRear");

        waitForStart();

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontMotor.setTargetPosition(1120);
        leftRearMotor.setTargetPosition(1120);
        rightFrontMotor.setTargetPosition(1120);
        rightRearMotor.setTargetPosition(1120);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFrontMotor.setPower(1);
        leftRearMotor.setPower(1);
        rightFrontMotor.setPower(1);
        rightRearMotor.setPower(1);

        telemetry.addData("Lf", leftFrontMotor.getCurrentPosition());
        telemetry.addData("Rf", rightFrontMotor.getCurrentPosition());
        telemetry.addData("Lr", leftRearMotor.getCurrentPosition());
        telemetry.addData("Rr", rightRearMotor.getCurrentPosition());

        telemetry.update();

    }

}
