/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AiC MecanumTele", group="Linear Opmode")
//@Disabled
public class AiCitizensMecanumTele extends LinearOpMode {

    // Declare OpMode members.

    // Runtime
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // Wheels motors
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private double RF, LF, RR, LR;
    private double X1, Y1, X2, Y2;

    private double joyScale = 0.5;

    private double precisionMin = 0.5;
    private double precisionMax = 0.8;

    private double motorMax = 0.8;

    // Arm motor
    private DcMotor armDrive = null;

    // Claw HEX motor
    private DcMotor clawDrive = null;

    // Claw servos
    private Servo leftServo = null;
    private Servo rightServo = null;

    //tray servos
    private Servo trayservo1 = null;
    private Servo trayservo2 = null;

    //small arm servo
    private Servo lowarmup = null;
    private Servo lowarmdown = null;




    public void initializeAll() {
        // Initialize motors
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rigthFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        // Initialize the arm drive
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");

        // Initialize the claw drive
        clawDrive = hardwareMap.get(DcMotor.class, "claw_drive");

        // Initialize the claw servos
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        trayservo1 = hardwareMap.get(Servo.class, "trayservo1");
        trayservo2 = hardwareMap.get(Servo.class, "trayservo2");
        lowarmup = hardwareMap.get(Servo.class, "lowarmup");
        lowarmdown = hardwareMap.get(Servo.class, "lowarmdown");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armDrive.setDirection(DcMotor.Direction.FORWARD);

        clawDrive.setDirection(DcMotor.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        trayservo1.setPosition(0);
        trayservo2.setPosition(0);
        lowarmup.setPosition(-0.5);
        lowarmdown.setPosition(1);
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeAll();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double servoPosition = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Initialize speeds
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;

            if (gamepad1.x) {
                if (joyScale == precisionMin) joyScale = precisionMax; else joyScale = precisionMin;
            }

            // Joystick values
            Y1 = -gamepad1.right_stick_y * joyScale;
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = -gamepad1.left_stick_y * joyScale;
            X2 = gamepad1.left_stick_x * joyScale;

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

            leftFrontMotor.setPower(LF);
            rightFrontMotor.setPower(RF);
            leftRearMotor.setPower(LR);
            rightRearMotor.setPower(RR);

            double armPower;

            double clawPower;

            double arm = gamepad2.left_stick_y;
            armPower = Range.clip(arm, -0.5, 0.5);

            double claw = gamepad2.right_stick_x;
            clawPower = Range.clip(claw, -0.3, 0.3);

            // Adjust claw power if direction is down
            if (clawPower < 0) clawPower = clawPower / 2.5;

            boolean servoInputUp = gamepad2.right_bumper;
            boolean servoInputDown = gamepad2.left_bumper;

            if (servoInputUp) {
                leftServo.setPosition(0);
                rightServo.setPosition(0);
            } else if (servoInputDown) {
                leftServo.setPosition(1);
                rightServo.setPosition(1);
            } else {
                leftServo.setPosition(0.5);
                rightServo.setPosition(0.5);
            }

            //Fix mechanical flaw aka arm going down
            if (armPower == 0) armPower = -0.01;

            armDrive.setPower(armPower);

            if (clawPower == 0) clawPower = +0.01;

            clawDrive.setPower(clawPower);

            double readServoLeft = leftServo.getPosition();
            double readServoRight = rightServo.getPosition();

            // Show telemetry info
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Precision", "%.3f", joyScale);

            telemetry.addData("LF", "%.3f", LF);
            telemetry.addData("RF", "%.3f", RF);
            telemetry.addData("LR", "%.3f", LR);
            telemetry.addData("RR", "%.3f", RR);

            // telemetry.addData("Servos", "(%.2f)", servoPosition);
            // telemetry.addData("Servo read", "(%.2f)", readServoLeft);
            // telemetry.addData("Servo read", "(%.2f)", readServoRight);

            telemetry.addData("Arm", "(%.2f)", armPower);

            telemetry.update();
        }
    }
}
