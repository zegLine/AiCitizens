package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="BOTH Park near Wall", group="Red")
public class AutoParkWall extends LinearOpMode {

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
        waitForStart();

        moveF(10, 0.3);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
