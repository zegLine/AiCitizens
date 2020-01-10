package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomousTrayGrabOmni", group="Autonomous")
public class AutonomousTrayGrabOmni extends LinearOpMode {

   DcMotor leftWheel;
   DcMotor rightwheel;
   Servo trayGrab1;
   Servo trayGrab2;






    @Override
    public void runOpMode() throws InterruptedException {

        leftWheel = hardwareMap.dcMotor.get("LeftMotor");
        rightwheel = hardwareMap.dcMotor.get("RightMotor");
        trayGrab1 = hardwareMap.servo.get("tray_grab_1");
        trayGrab2 = hardwareMap.servo.get("tray_grab_2");
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightwheel.setDirection(DcMotor.Direction.REVERSE);

        trayGrab1.setPosition(0.5);
        trayGrab2.setPosition(0.5);

        waitForStart();

        leftWheel.setPower(1);
        rightwheel.setPower(1);

        sleep(2100);

        leftWheel.setPower(0);
        rightwheel.setPower(0);

        trayGrab1.setPosition(-0.9);
        trayGrab2.setPosition(1);


        sleep(500);

        leftWheel.setPower(-1);
        rightwheel.setPower(-1);

        sleep(2800);







    }
}
