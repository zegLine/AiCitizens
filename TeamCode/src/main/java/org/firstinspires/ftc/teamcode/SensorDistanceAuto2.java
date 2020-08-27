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
@Autonomous(name = "Sensor Distance Autos 2", group = "Sensor")
public class SensorDistanceAuto2 extends LinearOpMode {

    ColorSensor sensorColor;
    DistanceSensor sensorCDistance;
    DigitalChannel sensorTouch;
    Rev2mDistanceSensor sensorDistance;

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 150;

    double maxpower = 0.5;
    double power;
    double manouverpower = 0.3;

    int robotpos = 0;
    //Move to the right the given amount of tiles
    public void moveRight(int tiles) {
        System.out.println("Move right...");
        robotpos += 1;
        leftFrontMotor.setPower(manouverpower);
        leftRearMotor.setPower(-manouverpower);
        rightFrontMotor.setPower(-manouverpower);
        rightRearMotor.setPower(manouverpower);
        sleep(tiles * 2000);
    }
    //Move to the left the given amount of tiles
    public void moveLeft(int tiles) {
        System.out.println("Move left...");
        robotpos -= 1;
        leftFrontMotor.setPower(-manouverpower);
        leftRearMotor.setPower(manouverpower);
        rightFrontMotor.setPower(manouverpower);
        rightRearMotor.setPower(-manouverpower);
        sleep(tiles * 2000);
    }

    @Override
    public void runOpMode() {
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorCDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        sensorTouch = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        sensorDistance = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_distance");

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftRearMotor = hardwareMap.dcMotor.get("leftRear");
        rightRearMotor = hardwareMap.dcMotor.get("rightRear");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()) {

            double distance = sensorDistance.getDistance(DistanceUnit.CM);

            while (distance > 3.5) {

                if (sensorTouch.getState() == false) {
                    break;
                }

                power = (Math.pow(1.05, distance) - 1) / (Math.pow(1.05, distance) + 1);
                if (power > maxpower) power = maxpower;
                if (power < 0) power = 0;

                leftFrontMotor.setPower(power);
                leftRearMotor.setPower(power);
                rightFrontMotor.setPower(power);
                rightRearMotor.setPower(power);

                telemetry.addData("Postition", robotpos);
                telemetry.addData("Power", power);
                telemetry.addData("Dist", distance);
                telemetry.addData("Pressed", sensorTouch.getState());
                telemetry.update();

                distance = sensorDistance.getDistance(DistanceUnit.CM);
            }

            switch (robotpos){
                case -1:
                    moveRight(1);
                    distance = sensorDistance.getDistance(DistanceUnit.CM);
                    if (distance < 5) moveRight(1);
                    break;
                case 0:
                    moveLeft(1);
                    distance = sensorDistance.getDistance(DistanceUnit.CM);
                    if (distance < 5) {
                        moveRight(2);
                    }
                    break;
                case 1:
                    moveLeft(1);
                    distance = sensorDistance.getDistance(DistanceUnit.CM);
                    if (distance < 5) moveLeft(1);
                    break;
            }

        }

        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();

    }
}
