package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="encoderTest", group="")
public class encoderTest extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;


    @Override
    public void runOpMode() {

        leftFrontMotor= hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor= hardwareMap.dcMotor.get("rightFront");

        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        while(leftFrontMotor.getCurrentPosition() != 0 || rightFrontMotor.getCurrentPosition() != 0)
        {
            leftFrontMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            rightFrontMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            waitOneFullHardwareCycle();
        }

        leftFrontMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFrontMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        leftFrontMotor.setTargetPosition(1120);
        rightFrontMotor.setTargetPosition(1120);

        leftFrontMotor.setPower(.5);
        rightFrontMotor.setPower(.5);

        while(leftFrontMotor.getCurrentPosition() < leftFrontMotor.getTargetPosition() || rightFrontMotor.getCurrentPosition() < rightFrontMotor.getTargetPosition()) { //While target has not been reached
            waitOneFullHardwareCycle(); //Needed within all loops
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);

        telemetry.addData("Lf", leftFrontMotor.getCurrentPosition());
        telemetry.addData("Rf", rightFrontMotor.getCurrentPosition());
        telemetry.addData("Lr", leftRearMotor.getCurrentPosition());
        telemetry.addData("Rr", rightRearMotor.getCurrentPosition());

        telemetry.update();
    }
}