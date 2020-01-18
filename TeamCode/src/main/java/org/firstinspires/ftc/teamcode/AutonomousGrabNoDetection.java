package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomousTrayGrabOmni", group="Autonomous")
public class AutonomousGrabNoDetection extends LinearOpMode {

   DcMotor leftWheel;
   DcMotor rightwheel;

   Servo lowArmBotServo;
   Servo lowArmHighServo;






    @Override
    public void runOpMode() throws InterruptedException {

        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightwheel = hardwareMap.dcMotor.get("right_wheel");
        lowArmBotServo = hardwareMap.servo.get("low_arm_bot_servo");
        lowArmHighServo = hardwareMap.servo.get("low_arm_high_servo");
        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        lowArmBotServo.setPosition(-0.5);
        lowArmHighServo.setPosition(1);

        waitForStart();

        leftWheel.setPower(1);
        rightwheel.setPower(1);

        sleep(1200);

        leftWheel.setPower(0);
        rightwheel.setPower(0);

        lowArmBotServo.setPosition(0.25);
        lowArmHighServo.setPosition(1);

        leftWheel.setPower(-1);
        rightwheel.setPower(-1);

        sleep(1000);

        leftWheel.setPower(0);
        rightwheel.setPower(0);

        leftWheel.setPower(0.6);
        rightwheel.setPower(-0.6);

        sleep(500);

        leftWheel.setPower(1);
        rightwheel.setPower(1);

        sleep(2000);

        leftWheel.setPower(0);
        rightwheel.setPower(0);

        lowArmBotServo.setPosition(-0.5);
        lowArmHighServo.setPosition(1);

        leftWheel.setPower(-1);
        rightwheel.setPower(-1);

        sleep(1800);















    }
}
