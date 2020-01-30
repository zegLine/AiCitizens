package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomousForRedSideWithStonesAndTray", group="Autonomous")

public  class AutonomousForBlueSideJustStones extends LinearOpMode {

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private Servo trayservo1 = null;
    private Servo trayservo2 = null;

    private Servo lowarmUp = null;
    private Servo lowarmDown = null;

    ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");

    public void moveForward(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void moveBackward(long time, double power) {

        power -= power;

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void moveLeft(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);

        sleep(time);
    }

    public void moveRight(long time, double power) {

        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(-power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void turnLeft(long time, double power) {

        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);

        sleep(time);
    }

    public void turnRight(long time, double power) {

        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftRearMotor.setPower(-power);
        rightRearMotor.setPower(power);

        sleep(time);
    }

    public void grabStone() {

        moveForward(150, 1);
        lowarmUp.setPosition(0.25);
        lowarmDown.setPosition(-1);
        moveBackward(300,1);

    }

    public void grabtray(){

        trayservo1.setPosition(1);
        trayservo2.setPosition(1);

    }

    public void opentray(){

        trayservo1.setPosition(0);
        trayservo2.setPosition(0);
    }

    @Override
    public void runOpMode() throws InterruptedException  {

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rigthFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        lowarmUp = hardwareMap.servo.get("lowarmup");
        lowarmDown = hardwareMap.servo.get("lowarmdown");

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

        lowarmUp.setPosition(-0.5);
        lowarmDown.setPosition(1);
        trayservo1.setPosition(0);
        trayservo2.setPosition(0);

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

            // Declared a normalization that, if black is under 2, if yellow is over 2
            int detectionSkyStone = (sensorColor.red() * sensorColor.green()) / (sensorColor.blue() * sensorColor.blue());
            int currentStonePos;

            long timeToMoveLeft = 3000;
            long timeAddedSecondSS = 400;

            moveForward(500, 1);

            if (detectionSkyStone <= 2) {
                // first stone

                grabStone();

                currentStonePos = 1;

            } else {

                moveRight(150, 1);

                if (detectionSkyStone <= 2) {
                    // second stone

                    grabStone();

                    currentStonePos = 2;

                } else {
                    // third stone

                    moveRight(150, 1);

                    grabStone();

                    currentStonePos = 3;

                }

            }

            // calculate the time to move right based on witch stone was grabbed
            timeToMoveLeft += 200 * currentStonePos;

            moveRight(timeToMoveLeft,1);

            lowarmUp.setPosition(-0.5);
            lowarmDown.setPosition(1);

            // calculate the time to come back and grab the second skystone based on time right
            moveRight(timeToMoveLeft + 400, 1);
            grabStone();

            // update the time to move right
            timeToMoveLeft += timeAddedSecondSS;
            moveRight(timeToMoveLeft,1);







        }

    }

}









