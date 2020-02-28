package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ENCDR_TST3", group="")
public class AutoEncoderTest3 extends LinearOpMode {

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

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setPower(1);
        leftRearMotor.setPower(1);
        rightFrontMotor.setPower(1);
        rightRearMotor.setPower(1);

        sleep(2000);

        telemetry.addData("Lf", leftFrontMotor.getCurrentPosition());
        telemetry.addData("Rf", rightFrontMotor.getCurrentPosition());
        telemetry.addData("Lr", leftRearMotor.getCurrentPosition());
        telemetry.addData("Rr", rightRearMotor.getCurrentPosition());


        telemetry.update();
        
    }

}
