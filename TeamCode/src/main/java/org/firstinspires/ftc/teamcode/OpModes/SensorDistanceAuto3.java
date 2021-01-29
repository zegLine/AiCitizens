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

package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;
import android.util.LogPrinter;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.logging.Logger;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Sensor Distance Autos 3")
public class SensorDistanceAuto3 extends LinearOpMode {

    Rev2mDistanceSensor sensorDistance;

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 150;

    //double maxpower = 0.5;
    //double power;
    double manouverpower = 0.3;
    double manouverpower2 = 0.31;
    double distance;
    final int tileSize = 3; //INCHES
    int fieldLength = 60; //TILES, NOT INCLUDING FIRST ONE

    int robotpos = 0;
    //Move to the right the given amount of tiles
    public void moveRight(int tiles) {
        System.out.println("Move right...");
        robotpos += tiles;
        leftFrontMotor.setPower(manouverpower-0.02);
        leftRearMotor.setPower(-manouverpower);
        rightFrontMotor.setPower(-manouverpower);
        rightRearMotor.setPower(manouverpower);
        sleep(tiles * 4500);
        distance = sensorDistance.getDistance(DistanceUnit.INCH);
    }
    //Move to the left the given amount of tiles
    public void moveLeft(int tiles) {
        System.out.println("Move left...");
        robotpos -= tiles;
        leftFrontMotor.setPower(-(manouverpower-0.02));
        leftRearMotor.setPower(manouverpower);
        rightFrontMotor.setPower(manouverpower);
        rightRearMotor.setPower(-manouverpower);
        sleep(tiles * 4500);
        distance = sensorDistance.getDistance(DistanceUnit.INCH);
    }
    //Move forward the given amount of tiles
    public void moveForward(int tiles) {
        System.out.println("Move forward...");
        leftFrontMotor.setPower(manouverpower);
        leftRearMotor.setPower(manouverpower);
        rightFrontMotor.setPower(manouverpower);
        rightRearMotor.setPower(manouverpower);
        sleep(tiles*285);
        fieldLength--;
        distance = sensorDistance.getDistance(DistanceUnit.INCH);
    }

    @Override
    public void runOpMode() {
        sensorDistance = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_distance");

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            if(fieldLength > 0) {
                distance = sensorDistance.getDistance(DistanceUnit.INCH);
                if(distance > tileSize) {
                    telemetry.addData("Postition", robotpos);
                    telemetry.addData("fieldSize", fieldLength);
                    telemetry.addData("Dist", distance);
                    telemetry.update();
                    moveForward(1);
                } else {
                    telemetry.addData("Postition", robotpos);
                    telemetry.addData("fieldSize", fieldLength);
                    telemetry.addData("Dist", distance);
                    telemetry.update();
                    //Schimbam linia de miscare
                    switch (robotpos) {
                        case -1:
                            moveRight(1);
                            leftFrontMotor.setPower(0);
                            leftRearMotor.setPower(0);
                            rightFrontMotor.setPower(0);
                            rightRearMotor.setPower(0);
                            sleep(500);
                            telemetry.addData("Postition", robotpos);
                            telemetry.addData("fieldSize", fieldLength);
                            telemetry.addData("Dist", distance);
                            telemetry.update();
                            if (distance < tileSize) {
                                moveRight(1);
                                leftFrontMotor.setPower(0);
                                leftRearMotor.setPower(0);
                                rightFrontMotor.setPower(0);
                                rightRearMotor.setPower(0);
                                sleep(500);
                                telemetry.addData("Postition", robotpos);
                                telemetry.addData("fieldSize", fieldLength);
                                telemetry.addData("Dist", distance);
                                telemetry.update();
                            }
                            break;
                        case 0:
                            moveLeft(1);
                            leftFrontMotor.setPower(0);
                            leftRearMotor.setPower(0);
                            rightFrontMotor.setPower(0);
                            rightRearMotor.setPower(0);
                            sleep(500);
                            telemetry.addData("Postition", robotpos);
                            telemetry.addData("fieldSize", fieldLength);
                            telemetry.addData("Dist", distance);
                            telemetry.update();
                            if (distance < tileSize) {
                                moveRight(2);
                                leftFrontMotor.setPower(0);
                                leftRearMotor.setPower(0);
                                rightFrontMotor.setPower(0);
                                rightRearMotor.setPower(0);
                                sleep(500);
                                telemetry.addData("Postition", robotpos);
                                telemetry.addData("fieldSize", fieldLength);
                                telemetry.addData("Dist", distance);
                                telemetry.update();
                            }
                            break;
                        case 1:
                            moveLeft(1);
                            leftFrontMotor.setPower(0);
                            leftRearMotor.setPower(0);
                            rightFrontMotor.setPower(0);
                            rightRearMotor.setPower(0);
                            sleep(500);
                            telemetry.addData("Postition", robotpos);
                            telemetry.addData("fieldSize", fieldLength);
                            telemetry.addData("Dist", distance);
                            telemetry.update();
                            if (distance < tileSize) {
                                moveLeft(1);
                                leftFrontMotor.setPower(0);
                                leftRearMotor.setPower(0);
                                rightFrontMotor.setPower(0);
                                rightRearMotor.setPower(0);
                                sleep(500);
                                telemetry.addData("Postition", robotpos);
                                telemetry.addData("fieldSize", fieldLength);
                                telemetry.addData("Dist", distance);
                                telemetry.update();
                            }
                            break;
                    }
                }
            }
            else {
                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);
                sleep(500);
                switch (robotpos){
                    case -1:
                        moveRight(1);
                        leftFrontMotor.setPower(0);
                        leftRearMotor.setPower(0);
                        rightFrontMotor.setPower(0);
                        rightRearMotor.setPower(0);
                        break;
                    case 1:
                        moveLeft(1);
                        leftFrontMotor.setPower(0);
                        leftRearMotor.setPower(0);
                        rightFrontMotor.setPower(0);
                        rightRearMotor.setPower(0);
                        break;
                }
            }

        }

    }
}
