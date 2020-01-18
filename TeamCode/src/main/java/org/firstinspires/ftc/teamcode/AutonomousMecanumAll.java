package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name="AutonomousMecanumAll", group="Autonomous")


public abstract class AutonomousMecanumAll extends LinearOpMode {

     DcMotor leftFrontMotor = null;
     DcMotor rightFrontMotor = null;
     DcMotor leftRearMotor = null;
     DcMotor rightRearMotor = null;
     ColorSensor sensorColor = null;
     Servo trayservo1 = null;
     Servo trayservo2 = null;
     ColorSensor sensorColor = null;


// Initialize motors


    ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");


    @Override
    public void runOpMode() throws InterruptedException  {


        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rigthFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");
        trayservo1 = hardwareMap.servo.get("trayservo1");
        trayservo2 = hardwareMap.servo.get("trayservo2");


        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        trayservo1.setPosition(-0.5);
        trayservo2.setPosition(1);


        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;


        waitForStart();

        while (opModeIsActive()) {






            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            int detectionSkyStone = (sensorColor.red() * sensorColor.blue()) / (sensorColor.blue() * sensorColor.blue());
            int k=1;

            leftFrontMotor.setPower(1);
            rightFrontMotor.setPower(1);
            leftRearMotor.setPower(1);
            rightRearMotor.setPower(1);

            sleep(500);



            if(detectionSkyStone<2){
                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(1);
                leftRearMotor.setPower(1);
                rightRearMotor.setPower(1);
                sleep(150);

                trayservo1.setPosition(0.25);
                trayservo2.setPosition(-1);

                leftFrontMotor.setPower(-1);
                rightFrontMotor.setPower(-1);
                leftRearMotor.setPower(-1);
                rightRearMotor.setPower(-1);
                sleep(200);

                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(-1);
                leftRearMotor.setPower(-1);
                rightRearMotor.setPower(1);
                sleep(2000);

                trayservo1.setPosition(-0.5);
                trayservo2.setPosition(1);

                if(k==1)
                {
                    leftFrontMotor.setPower(-1);
                    rightFrontMotor.setPower(1);
                    leftRearMotor.setPower(1);
                    rightRearMotor.setPower(-1);
                    sleep(3000);
                }



            }






        }




    }


    }









