package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CuzaBot;

@TeleOp(name = "GyroAuto")
public class GyroAuto extends LinearOpMode {

    CuzaBot robot = new CuzaBot();

    @Override
    public void runOpMode() {

        robot.initializeAll(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {

            robot.getImuAngles();

            robot.powerMotors(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if(gamepad1.right_trigger > 0) {
                robot.powerIntake(gamepad1.right_trigger);
            } else if(gamepad1.left_trigger > 0){
                robot.powerIntake(-gamepad1.left_trigger);
            }

            telemetry.addLine(robot.composeTelemetry());
            telemetry.update();
        }
    }
}
