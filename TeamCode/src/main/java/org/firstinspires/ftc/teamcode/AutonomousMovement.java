package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Movement Basic", group="")
public class AutonomousMovement extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    public void initializeAll() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void moveF (int position, double power) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setTargetPosition(position);
        rightFrontMotor.setTargetPosition(position);
        leftRearMotor.setTargetPosition(position);
        rightRearMotor.setTargetPosition(position);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftFrontMotor.isBusy() || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy() || rightRearMotor.isBusy()) {
            telemetry.addData("LeftFront", leftFrontMotor.getCurrentPosition());
            telemetry.addData("RightFront", rightFrontMotor.getCurrentPosition());
            telemetry.addData("LeftRear", leftRearMotor.getCurrentPosition());
            telemetry.addData("RightRear", rightRearMotor.getCurrentPosition());

            telemetry.update();
        }

    }

    @Override
    public void runOpMode() {
        initializeAll();
        waitForStart();

        moveF(1000, 0.5);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
