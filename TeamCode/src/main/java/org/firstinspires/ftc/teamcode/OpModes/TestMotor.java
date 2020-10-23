package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CuzaBot;

@TeleOp(name="Test 1 Motor", group="Linear Opmode")
public class TestMotor extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor motor = null;
    private DcMotor motor2 = null;

    @Override
    public void runOpMode() {

        motor = hardwareMap.dcMotor.get("motortest");
        motor2 = hardwareMap.dcMotor.get("motortest2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            motor.setPower(gamepad1.right_trigger);
            motor2.setPower(gamepad1.right_trigger);

        }
    }
}
