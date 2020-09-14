package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "AutoRSkystone", group = "RED")
public class AutoSkystone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    //VUFORIA
    private static final String VUFORIA_KEY =
            " AWh3WbD/////AAABmQr66RjvbkVtr+RI6oomXqIgCzVDQtdjwkNT4jkW0JBVLrq3rymbi6vq3sBtaFBrD4rYqleNmM9WFwZWYYNka48h4t85scS+/g7cTt0g84GiuI3J8uqDqL4IKpVlu+JLSEW9J0KkuoQSksN0RIxVCqC87a2MKMF9IRUuSz35PYN59JSwljttQORgO4MJGb5O8nwDbEM0cOPyKO8NpNftDnGr0MeBFJPVv2BBN2KfGdUO9/EyEPrHLfj7tchxBDkXE2Bk5muqA8MY+9cw5HoSw7aHSPd2beotDziYc9YtvbrmpdNc3HlMA0i/wAFAuh39k7che12HYEi5VdEmJ4ZG/yaTDuIsMNqz/wMZMSpjfJGd ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.50;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private Servo lowArmBottomServo = null;
    private Servo lowArmHighServo = null;

    double circumference = 3.1415 * 3.93701;

    public void initializeAll() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the low (bottom) arm
        lowArmBottomServo = hardwareMap.get(Servo.class, "lowarmdown");
        lowArmHighServo = hardwareMap.get(Servo.class, "lowarmup");

        lowArmBottomServo.setDirection(Servo.Direction.REVERSE);
        lowArmHighServo.setDirection(Servo.Direction.FORWARD);
    }

    public void moveF (double distance, double power) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotneed = distance / circumference;
        int position = (int) (rotneed * 1120);

        leftFrontMotor.setTargetPosition(position);
        rightFrontMotor.setTargetPosition(position);
        leftRearMotor.setTargetPosition(position);
        rightRearMotor.setTargetPosition(position);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);


        while ((leftFrontMotor.isBusy() || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy() || rightRearMotor.isBusy()) && Math.abs(leftFrontMotor.getCurrentPosition()) < Math.abs(position)-10) {
            telemetry.addData("DIST", distance);
            telemetry.addData("TICKS NEEDED", position);
            telemetry.addData("LeftFront", leftFrontMotor.getCurrentPosition());
            telemetry.addData("RightFront", rightFrontMotor.getCurrentPosition());
            telemetry.addData("LeftRear", leftRearMotor.getCurrentPosition());
            telemetry.addData("RightRear", rightRearMotor.getCurrentPosition());

            telemetry.update();
        }

    }

    public void moveB (double distance, double power) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotneed = distance / circumference;
        int position = (int) (rotneed * 1120);

        leftFrontMotor.setTargetPosition(-position);
        rightFrontMotor.setTargetPosition(-position);
        leftRearMotor.setTargetPosition(-position);
        rightRearMotor.setTargetPosition(-position);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);


        while ((leftFrontMotor.isBusy() || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy() || rightRearMotor.isBusy())
                && Math.abs(leftFrontMotor.getCurrentPosition()) < Math.abs(position)-10) {
            telemetry.addData("DIST", distance);
            telemetry.addData("TICKS NEEDED", position);
            telemetry.addData("LeftFront", leftFrontMotor.getCurrentPosition());
            telemetry.addData("RightFront", rightFrontMotor.getCurrentPosition());
            telemetry.addData("LeftRear", leftRearMotor.getCurrentPosition());
            telemetry.addData("RightRear", rightRearMotor.getCurrentPosition());

            telemetry.update();
        }

    }

    public void moveL (double distance, double power) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotneed = distance / circumference;
        int position = (int) (rotneed * 1120);

        leftFrontMotor.setTargetPosition(-position);
        rightFrontMotor.setTargetPosition(position);
        leftRearMotor.setTargetPosition(position);
        rightRearMotor.setTargetPosition(-position);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);


        while ((leftFrontMotor.isBusy() || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy() || rightRearMotor.isBusy())
                && Math.abs(leftFrontMotor.getCurrentPosition()) < Math.abs(position)-10) {
            telemetry.addData("DIST", distance);
            telemetry.addData("TICKS NEEDED", position);
            telemetry.addData("LeftFront", leftFrontMotor.getCurrentPosition());
            telemetry.addData("RightFront", rightFrontMotor.getCurrentPosition());
            telemetry.addData("LeftRear", leftRearMotor.getCurrentPosition());
            telemetry.addData("RightRear", rightRearMotor.getCurrentPosition());

            telemetry.update();
        }

    }

    public void moveR (double distance, double power) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotneed = distance / circumference;
        int position = (int) (rotneed * 1120);

        leftFrontMotor.setTargetPosition(position);
        rightFrontMotor.setTargetPosition(-position);
        leftRearMotor.setTargetPosition(-position);
        rightRearMotor.setTargetPosition(position);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);


        while ((leftFrontMotor.isBusy() || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy() || rightRearMotor.isBusy())
                && Math.abs(leftFrontMotor.getCurrentPosition()) < Math.abs(position)-10) {
            telemetry.addData("DIST", distance);
            telemetry.addData("TICKS NEEDED", position);
            telemetry.addData("LeftFront", leftFrontMotor.getCurrentPosition());
            telemetry.addData("RightFront", rightFrontMotor.getCurrentPosition());
            telemetry.addData("LeftRear", leftRearMotor.getCurrentPosition());
            telemetry.addData("RightRear", rightRearMotor.getCurrentPosition());

            telemetry.update();
        }

    }

    public void turnR (double distance, double power) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotneed = distance / circumference;
        int position = (int) (rotneed * 1120);

        leftFrontMotor.setTargetPosition(position);
        rightFrontMotor.setTargetPosition(-position);
        leftRearMotor.setTargetPosition(position);
        rightRearMotor.setTargetPosition(-position);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);


        while (leftFrontMotor.isBusy() || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy() || rightRearMotor.isBusy()) {
            telemetry.addData("DIST", distance);
            telemetry.addData("TICKS NEEDED", position);
            telemetry.addData("LeftFront", leftFrontMotor.getCurrentPosition());
            telemetry.addData("RightFront", rightFrontMotor.getCurrentPosition());
            telemetry.addData("LeftRear", leftRearMotor.getCurrentPosition());
            telemetry.addData("RightRear", rightRearMotor.getCurrentPosition());

            telemetry.update();
        }

    }

    public void turnL (double distance, double power) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotneed = distance / circumference;
        int position = (int) (rotneed * 1120);

        leftFrontMotor.setTargetPosition(-position);
        rightFrontMotor.setTargetPosition(position);
        leftRearMotor.setTargetPosition(-position);
        rightRearMotor.setTargetPosition(position);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);


        while (leftFrontMotor.isBusy() || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy() || rightRearMotor.isBusy()) {
            telemetry.addData("DIST", distance);
            telemetry.addData("TICKS NEEDED", position);
            telemetry.addData("LeftFront", leftFrontMotor.getCurrentPosition());
            telemetry.addData("RightFront", rightFrontMotor.getCurrentPosition());
            telemetry.addData("LeftRear", leftRearMotor.getCurrentPosition());
            telemetry.addData("RightRear", rightRearMotor.getCurrentPosition());

            telemetry.update();
        }

    }


    @Override
    public void runOpMode() {
        initializeAll();
        initVuforia();
        initTfod();
        tfod.activate();
        waitForStart();

        moveF(26, 0.3);
        boolean skystone = false;
        double diff = 0;
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        if (tfod != null) {
            if (recognitions != null) {
                telemetry.addData("# Object Detected", recognitions.size());
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                        moveF(5.5, 0.3);
                        skystone = true;

                    }
                }

                if (!skystone) {
                    diff += 9.5;
                    moveL(9.5, 0.3);
                    sleep(1000);
                    recognitions = tfod.getUpdatedRecognitions();
                    for (Recognition recognition : recognitions) {
                        if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                            moveF(5.5, 0.3);
                            skystone = true;

                        }
                    }
                }

                if (!skystone) {
                    diff += 9.5;
                    moveL(9.5, 0.3);
                    moveF(5.5, 0.3);
                }
            }
        }
        tfod.shutdown();

        lowArmBottomServo.setPosition(1);
        lowArmHighServo.setPosition(0.7);
        sleep(1000);
        lowArmBottomServo.setPosition(1);

        moveB(12, 0.3);
        moveR(47.5 + diff,0.5);
        lowArmHighServo.setPosition(0.5);
        lowArmBottomServo.setPosition(0);
        sleep(500);
        lowArmHighServo.setPosition(1);
        moveL(20, 0.4);
        moveF(8,0.3);
    }
}
