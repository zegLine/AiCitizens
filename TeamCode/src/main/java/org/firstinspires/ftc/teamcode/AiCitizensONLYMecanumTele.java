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

@TeleOp(name="NEW MECANUM TELE", group="Linear Opmode")
//@Disabled
public class AiCitizensONLYMecanumTele extends LinearOpMode {

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

    private double precisionMin = 0.7;
    private double precisionMax = 1.0;

    private double motorMax = 1;

    //Tray servos
    private Servo trayServo1 = null;
    private Servo trayServo2 = null;

    // Low arm servos
    private Servo lowArmBottomServo = null;
    private Servo lowArmHighServo = null;


    public void initializeAll() {
        // Initialize motors
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize tray servos
        trayServo1 = hardwareMap.get(Servo.class, "tray_servo_1");
        trayServo2 = hardwareMap.get(Servo.class, "tray_servo_2");

        trayServo1.setDirection(Servo.Direction.FORWARD);
        trayServo2.setDirection(Servo.Direction.REVERSE);

        // Initialize the low (bottom) arm
        lowArmBottomServo = hardwareMap.get(Servo.class, "low_arm_bottom_servo");
        lowArmHighServo = hardwareMap.get(Servo.class, "low_arm_high_servo");

        lowArmBottomServo.setDirection(Servo.Direction.REVERSE);
        lowArmHighServo.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeAll();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Initialize speeds
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;

            if (gamepad1.x && joyScale == precisionMin) joyScale = precisionMax;
            if (gamepad1.x && joyScale == precisionMax) joyScale = precisionMin;

            // Tray servos positions
            boolean closeTray = gamepad1.left_bumper;
            boolean openTray = gamepad1.right_bumper;

            if (openTray){
                trayServo1.setPosition(0.5);
                trayServo2.setPosition(0.5);
            }

            if (closeTray){
                trayServo1.setPosition(0);
                trayServo2.setPosition(0);
            }

            // Calculate LOW ARM positions
            if (gamepad2.y){
                lowArmBottomServo.setPosition(0.25);
                lowArmHighServo.setPosition(-1);
            }

            if (gamepad2.a){
                lowArmBottomServo.setPosition(-0.5);
                lowArmHighServo.setPosition(1);
            }

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

            leftFrontMotor.setPower(LF);
            rightFrontMotor.setPower(RF);
            leftRearMotor.setPower(LR);
            rightRearMotor.setPower(RR);


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



            telemetry.update();
        }
    }
}
