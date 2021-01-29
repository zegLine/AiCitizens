package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.CuzaBot;

@TeleOp(name = "GyroTest")
public class GyroTest extends LinearOpMode {

    CuzaBot robot = new CuzaBot();

    @Override
    public void runOpMode() {

        robot.initializeImu(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {

            robot.getImuAngles();

            if(robot.angles.firstAngle > 5 && gamepad1.x) {
                telemetry.addData("Direction: ", "left");
                //robot.powerMotors(-0.2,-0.2,0.2,0.2);
            } else if(robot.angles.firstAngle < -5 && gamepad1.x) {
                telemetry.addData("Direction: ", "right");
                //robot.powerMotors(0.2,0.2,-0.2,-0.2);
            }

            telemetry.addLine(robot.composeTelemetry());
            telemetry.update();
        }
    }
}
