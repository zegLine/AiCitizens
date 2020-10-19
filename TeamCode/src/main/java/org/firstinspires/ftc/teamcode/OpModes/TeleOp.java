package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CuzaBot;

public class TeleOp extends LinearOpMode {
    // Declare OpMode members.
    CuzaBot robot = new CuzaBot();

    private double RF, LF, RR, LR;
    private double X1, Y1, X2, Y2;

    private double joyScale = 0.7;

    private double precisionMin = 0.7;
    private double precisionMax = 1.0;

    private double motorMax = 1;

    @Override
    public void runOpMode() {

        robot.initializeAll(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.runtime.reset();
        robot.ColorSensor.enableLed(false);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Activam sistemul de lansare cu rightbumper
            if(gamepad2.right_bumper) {
                robot.launchMotor.setPower(motorMax);
            } else {
                robot.launchMotor.setPower(0);
            }

            // Initialize speeds
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;

            if (gamepad1.x && joyScale == precisionMin) joyScale = precisionMax;
            if (gamepad1.x && joyScale == precisionMax) joyScale = precisionMin;

            /*



                WHEELS MOVEMENT



             */

            // Joystick values
            Y1 = -gamepad1.left_stick_y * joyScale;
            X1 = gamepad1.left_stick_x * joyScale;
            Y2 = -gamepad1.right_stick_y * joyScale;
            X2 = gamepad1.right_stick_x * joyScale;

            // Forward / backward movement
            LF += Y1; RF += Y1; LR += Y1; RR += Y1;

            // Side to side movement
            LF += X1; RF -= X1; LR -= X1; RR += X1;

            // Rotation movement
            LF += X2; RF -= X2; LR += X2; RR -= X2;

            LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LR = Math.max(-motorMax, Math.min(LR, motorMax));
            RR = Math.max(-motorMax, Math.min(RR, motorMax));

            robot.powerMotors(LF, LR, RF, RR);

            // Show telemetry info
            telemetry.addData("Status", "Run Time: " + robot.runtime.toString());

            telemetry.addData("LF", "%.3f", LF);
            telemetry.addData("RF", "%.3f", RF);
            telemetry.addData("LR", "%.3f", LR);
            telemetry.addData("RR", "%.3f", RR);
            telemetry.addData("Distance:", robot.DistanceSensor.getDistance(DistanceUnit.CM));

            telemetry.update();
        }
    }
}
