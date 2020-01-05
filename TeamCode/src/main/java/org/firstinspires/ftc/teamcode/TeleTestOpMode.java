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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="TestTeleOp", group="Linear Opmode")
//@Disabled
public class TeleTestOpMode extends LinearOpMode {

    // Declare OpMode members.

    // Wheels motors and runtime
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // Arm motor
    private DcMotor armDrive = null;

    // Claw HEX motor
    private DcMotor clawDrive = null;

    // Claw servos
    private Servo leftServo = null;
    private Servo rightServo = null;

    // Low arm servos
    private Servo lowArmBottomServo = null;
    private Servo lowArmHighServo = null;

    public void initializeAll() {

        // Initialize motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Initialize the arm drive
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");

        // Initialize the claw drive
        clawDrive = hardwareMap.get(DcMotor.class, "claw_drive");

        // Initialize the claw servos
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        // Initialize the low (bottom) arm
        lowArmBottomServo = hardwareMap.get(Servo.class, "low_arm_bottom_servo");
        lowArmHighServo = hardwareMap.get(Servo.class, "low_arm_high_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        armDrive.setDirection(DcMotor.Direction.FORWARD);

        clawDrive.setDirection(DcMotor.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        lowArmBottomServo.setDirection(Servo.Direction.FORWARD);
        lowArmHighServo.setDirection(Servo.Direction.FORWARD);

        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        lowArmBottomServo.setPosition(0.68);
        lowArmHighServo.setPosition(1);
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeAll();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //TODO Organize constants
        double movePrecision = 1.0;


        // LOW ARM = LA
        double LA_INCREMENT = 0.005;

        double lowArmBottomPos = 0.68;
        double lowArmHighPos = 1.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double armPower;

            double clawPower;

            if (gamepad1.x) {
                if (movePrecision == 1) movePrecision = 2.0; else movePrecision = 1.0;
            }

            // POV Mode uses left stick to go forward, and right stick to turn.
            double drive = -gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            leftPower    = Range.clip(( drive + turn ) / movePrecision, -1.0, 1.0) ;
            rightPower   = Range.clip(( drive - turn ) / movePrecision, -1.0, 1.0) ;

            double arm = gamepad2.left_stick_y;
            armPower = Range.clip(arm, -0.5, 0.5);

            double claw = gamepad2.right_stick_x;
            clawPower = Range.clip(claw, -0.5, 0.5);

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

            // Send calculated power to wheels
            //TODO Pile up power sending in separate methods
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //Fix mechanical flaw aka arm going down
            if (armPower == 0) armPower = -0.01;

            armDrive.setPower(armPower);

            if (clawPower == 0) clawPower = +0.01;

            clawDrive.setPower(clawPower);

            // Calculate LOW ARM positions
            if (gamepad2.dpad_up) {
                lowArmBottomPos += LA_INCREMENT;
            }
            if (gamepad2.dpad_down) {
                lowArmBottomPos -= LA_INCREMENT;
            }
            if (gamepad2.dpad_right) {
                lowArmHighPos += LA_INCREMENT;
            }
            if (gamepad2.dpad_left) {
                lowArmHighPos -= LA_INCREMENT;
            }

            // Limit LOW ARM Bottom
            if (lowArmBottomPos > 0.70) lowArmBottomPos = 0.70;

            if (lowArmHighPos < 0.45) lowArmHighPos = 0.45;
            if (lowArmBottomPos < 0.05) lowArmBottomPos = 0.05;

            // Set position to the LOW ARM servos
            lowArmBottomServo.setPosition(lowArmBottomPos);
            lowArmHighServo.setPosition(lowArmHighPos);

            // Show telemetry info
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            telemetry.addData("SBOTTOM", "(%.2f)", lowArmBottomServo.getPosition());
            telemetry.addData("SHIGH", "(%.2f)", lowArmHighServo.getPosition());
            telemetry.addData("DPAD UP", "" + gamepad2.dpad_up);

            //double readServoLeft = leftServo.getPosition();
            //double readServoRight = rightServo.getPosition();

            // telemetry.addData("Servos", "(%.2f)", servoPosition);
            // telemetry.addData("Servo read", "(%.2f)", readServoLeft);
            // telemetry.addData("Servo read", "(%.2f)", readServoRight);

            telemetry.addData("Arm", "(%.2f)", armPower);

            telemetry.update();
        }
    }
}
