/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutonomieTensorFlowCuAxe", group = "Autonomous")


public class AutonomieTensorFlowCuAxeGyroSiDistanta extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    int Motor_Tick_Counts = 1120;
    double Circumference = 3.1415*3.93701;
    int SkyStonePosition;
    boolean Found = false;
    double tilesize = 22.75;
    double X=0;
    double Y=0;
    int Orientation;
    boolean grabbed=false;
    double SkyStoneSize=0.25;
    private DistanceSensor sensorRange;
    private ColorSensor sensorColor;
    double dist = sensorRange.getDistance(DistanceUnit.INCH);
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;





    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private Servo trayservo1 = null;
    private Servo trayservo2 = null;

    private Servo lowarmUp = null;
    private Servo lowarmDown = null;



    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }



    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double globalAngle = 0;
    }


    public void moveForward(double power,double distance) {

        resetAngle();
        if (getAngle()==0){
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double rotationsNeeded = (distance*tilesize)/Circumference;
            int encoderDrivingTarget=(int)(rotationsNeeded*1120);
            leftFrontMotor.setTargetPosition(encoderDrivingTarget);
            leftRearMotor.setTargetPosition(encoderDrivingTarget);
            rightFrontMotor.setTargetPosition(encoderDrivingTarget);
            rightRearMotor.setTargetPosition(encoderDrivingTarget);
            leftFrontMotor.setPower(power);
            rightFrontMotor.setPower(power);
            leftRearMotor.setPower(power);
            rightRearMotor.setPower(power);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            if(Orientation==1){
                Y = Y+distance;
            }
            if(Orientation==2){
                X = X+distance;
            }
            if(Orientation==3){
                Y=Y-distance;
            }
            if(Orientation==4){
                X=X-distance;
            }
            else {

                if (getAngle() > 0) {

                    leftFrontMotor.setPower(-0.3 - checkDirection());
                    rightFrontMotor.setPower(0.3 + checkDirection());
                    leftRearMotor.setPower(-0.3 - checkDirection());
                    rightRearMotor.setPower(0.3 + checkDirection());

                } else {

                    leftFrontMotor.setPower(0.3 - checkDirection());
                    rightFrontMotor.setPower(-0.3 + checkDirection());
                    leftRearMotor.setPower(0.3 - checkDirection());
                    rightRearMotor.setPower(-0.3 + checkDirection());

                }
            }

            telemetry.addData("X=",X);
            telemetry.addData("Y=",Y);
            telemetry.update();

        }


    }

     public void moveBackward(double power,double distance) {

        resetAngle();
        if(getAngle()==0){
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double rotationsNeeded = (tilesize*distance)/Circumference;
            int encoderDrivingTarget=(int)(rotationsNeeded*1120);
            leftFrontMotor.setTargetPosition(encoderDrivingTarget);
            leftRearMotor.setTargetPosition(encoderDrivingTarget);
            rightFrontMotor.setTargetPosition(encoderDrivingTarget);
            rightRearMotor.setTargetPosition(encoderDrivingTarget);
            leftFrontMotor.setPower(-power);
            rightFrontMotor.setPower(-power);
            leftRearMotor.setPower(-power);
            rightRearMotor.setPower(-power);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(Orientation==1){
                Y = Y-distance;
            }
            if(Orientation==2){
                X = X-distance;
            }
            if(Orientation==3){
                Y=Y+distance;
            }
            if(Orientation==4){
                X=X+distance;
            }
        }

        else {

            if(getAngle()>0){

                leftFrontMotor.setPower(-0.3-checkDirection());
                rightFrontMotor.setPower(0.3+checkDirection());
                leftRearMotor.setPower(-0.3-checkDirection());
                rightRearMotor.setPower(0.3+checkDirection());

            }
            else{

                leftFrontMotor.setPower(0.3-checkDirection());
                rightFrontMotor.setPower(-0.3+checkDirection());
                leftRearMotor.setPower(0.3-checkDirection());
                rightRearMotor.setPower(-0.3+checkDirection());

            }
        }



        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
         telemetry.update();

    }

    public void moveLeft(double power,double distance) {

        resetAngle();
        if(getAngle()==0){
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double rotationsNeeded = (distance*tilesize)/Circumference;
            int encoderDrivingTarget=(int)(rotationsNeeded*1120);
            leftFrontMotor.setTargetPosition(encoderDrivingTarget);
            leftRearMotor.setTargetPosition(encoderDrivingTarget);
            rightFrontMotor.setTargetPosition(encoderDrivingTarget);
            rightRearMotor.setTargetPosition(encoderDrivingTarget);
            leftFrontMotor.setPower(-power);
            rightFrontMotor.setPower(power);
            leftRearMotor.setPower(power);
            rightRearMotor.setPower(-power);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(Orientation==1){
                X = X-distance;
            }
            if(Orientation==2){
                Y = Y+distance;
            }
            if(Orientation==3){
                X=X+distance;
            }
            if(Orientation==4){
                Y=Y-distance;
            }
        }
        else {

            if (getAngle() > 0) {

                leftFrontMotor.setPower(-0.3 - checkDirection());
                rightFrontMotor.setPower(0.3 + checkDirection());
                leftRearMotor.setPower(-0.3 - checkDirection());
                rightRearMotor.setPower(0.3 + checkDirection());

            } else {

                leftFrontMotor.setPower(0.3 - checkDirection());
                rightFrontMotor.setPower(-0.3 + checkDirection());
                leftRearMotor.setPower(0.3 - checkDirection());
                rightRearMotor.setPower(-0.3 + checkDirection());

            }
        }


        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }

    public void moveRight(double power,double distance) {

        resetAngle();
        if(getAngle()==0)
        {
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double rotationsNeeded = (distance*tilesize)/Circumference;
            int encoderDrivingTarget=(int)(rotationsNeeded*1120);
            leftFrontMotor.setTargetPosition(encoderDrivingTarget);
            leftRearMotor.setTargetPosition(encoderDrivingTarget);
            rightFrontMotor.setTargetPosition(encoderDrivingTarget);
            rightRearMotor.setTargetPosition(encoderDrivingTarget);
            leftFrontMotor.setPower(power);
            rightFrontMotor.setPower(-power);
            leftRearMotor.setPower(-power);
            rightRearMotor.setPower(power);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if(Orientation==1){
                X = X+distance;
            }
            if(Orientation==2){
                Y = Y-distance;
            }
            if(Orientation==3){
                X=X-distance;
            }
            if(Orientation==4){
                Y=Y+ distance;
            }
        }

        else {

            if (getAngle() > 0) {

                leftFrontMotor.setPower(-0.3 - checkDirection());
                rightFrontMotor.setPower(0.3 + checkDirection());
                leftRearMotor.setPower(-0.3 - checkDirection());
                rightRearMotor.setPower(0.3 + checkDirection());

            } else {

                leftFrontMotor.setPower(0.3 - checkDirection());
                rightFrontMotor.setPower(-0.3 + checkDirection());
                leftRearMotor.setPower(0.3 - checkDirection());
                rightRearMotor.setPower(-0.3 + checkDirection());

            }
        }

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }

    public void turnLeft(double power,double distance) {

        resetAngle();
        if(getAngle()==0)
        {
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            double rotationsNeeded = (distance*tilesize)/Circumference;
            int encoderDrivingTarget=(int)(rotationsNeeded*1120);
            leftFrontMotor.setTargetPosition(encoderDrivingTarget);
            leftRearMotor.setTargetPosition(encoderDrivingTarget);
            rightFrontMotor.setTargetPosition(encoderDrivingTarget);
            rightRearMotor.setTargetPosition(encoderDrivingTarget);
            leftFrontMotor.setPower(-power);
            rightFrontMotor.setPower(power);
            leftRearMotor.setPower(-power);
            rightRearMotor.setPower(power);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if(Orientation==1){
                Orientation=4;
            }
            if(Orientation==2){
                Orientation=1;
            }
            if(Orientation==3){
                Orientation=2;
            }
            if(Orientation==4){
                Orientation=3;
            }
        }
        else {

            if (getAngle() > 0) {

                leftFrontMotor.setPower(-0.3 - checkDirection());
                rightFrontMotor.setPower(0.3 + checkDirection());
                leftRearMotor.setPower(-0.3 - checkDirection());
                rightRearMotor.setPower(0.3 + checkDirection());

            } else {

                leftFrontMotor.setPower(0.3 - checkDirection());
                rightFrontMotor.setPower(-0.3 + checkDirection());
                leftRearMotor.setPower(0.3 - checkDirection());
                rightRearMotor.setPower(-0.3 + checkDirection());

            }
        }

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();

    }

    public void turnRight(double power,double distance) {

        resetAngle();
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double rotationsNeeded = (distance*tilesize)/Circumference;
        int encoderDrivingTarget=(int)(rotationsNeeded*1120);
        leftFrontMotor.setTargetPosition(encoderDrivingTarget);
        leftRearMotor.setTargetPosition(encoderDrivingTarget);
        rightFrontMotor.setTargetPosition(encoderDrivingTarget);
        rightRearMotor.setTargetPosition(encoderDrivingTarget);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftRearMotor.setPower(power);
        rightRearMotor.setPower(-power);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(Orientation==1){
            Orientation=2;
        }
        if(Orientation==2){
            Orientation=3;
        }
        if(Orientation==3){
            Orientation=4;
        }
        if(Orientation==4){
            Orientation=1;
        }

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }



    public void moverightdistsensor(double power,double distance){

        resetAngle();
        if(getAngle()==0)
        while(distance<dist)
        {
            leftFrontMotor.setPower(power);
            rightFrontMotor.setPower(-power);
            leftRearMotor.setPower(-power);
            rightRearMotor.setPower(power);
        }
        else {

            if (getAngle() > 0) {

                leftFrontMotor.setPower(-0.3 - checkDirection());
                rightFrontMotor.setPower(0.3 + checkDirection());
                leftRearMotor.setPower(-0.3 - checkDirection());
                rightRearMotor.setPower(0.3 + checkDirection());

            } else {

                leftFrontMotor.setPower(0.3 - checkDirection());
                rightFrontMotor.setPower(-0.3 + checkDirection());
                leftRearMotor.setPower(0.3 - checkDirection());
                rightRearMotor.setPower(-0.3 + checkDirection());

            }
        }

    }

    public void moveleftdistsensor(double power,double distance){

        resetAngle();
        if(getAngle()==0)
        while(distance>dist)
        {
            leftFrontMotor.setPower(-power);
            rightFrontMotor.setPower(power);
            leftRearMotor.setPower(power);
            rightRearMotor.setPower(-power);
        }
        else {

            if (getAngle() > 0) {

                leftFrontMotor.setPower(-0.3 - checkDirection());
                rightFrontMotor.setPower(0.3 + checkDirection());
                leftRearMotor.setPower(-0.3 - checkDirection());
                rightRearMotor.setPower(0.3 + checkDirection());

            } else {

                leftFrontMotor.setPower(0.3 - checkDirection());
                rightFrontMotor.setPower(-0.3 + checkDirection());
                leftRearMotor.setPower(0.3 - checkDirection());
                rightRearMotor.setPower(-0.3 + checkDirection());

            }
        }

    }

    public void parking(double power){
        while(sensorColor.red()==0)
        {
            leftFrontMotor.setPower(-power);
            rightFrontMotor.setPower(-power);
            leftRearMotor.setPower(-power);
            rightRearMotor.setPower(-power);
        }
    }





    public void grabStone() {




        lowarmUp.setPosition(-0.5);
        lowarmDown.setPosition(1);


    }

    public void releaseStone(){

        lowarmUp.setPosition(0.7);
        lowarmDown.setPosition(-1);

    }

    public void grabtray(){

        trayservo1.setPosition(0.1);
        trayservo2.setPosition(0.1);

    }

    public void opentray(){

        trayservo1.setPosition(1);
        trayservo2.setPosition(1);
    }

    public void stoneGrabing(){
        moveForward(0.5,1);
        grabStone();
        moveBackward(0.5,0.5);
        grabbed=true;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }




    public void rotate(double power,double degrees){

             resetAngle();
             while(degrees>getAngle())
             {
                 leftFrontMotor.setPower(power);
                 rightFrontMotor.setPower(-power);
                 leftRearMotor.setPower(power);
                 rightRearMotor.setPower(-power);
             }
             while(degrees<getAngle())
             {
                 leftFrontMotor.setPower(-power);
                 rightFrontMotor.setPower(power);
                 leftRearMotor.setPower(-power);
                 rightRearMotor.setPower(power);
             }

    }


    private static final String VUFORIA_KEY =
            " AWh3WbD/////AAABmQr66RjvbkVtr+RI6oomXqIgCzVDQtdjwkNT4jkW0JBVLrq3rymbi6vq3sBtaFBrD4rYqleNmM9WFwZWYYNka48h4t85scS+/g7cTt0g84GiuI3J8uqDqL4IKpVlu+JLSEW9J0KkuoQSksN0RIxVCqC87a2MKMF9IRUuSz35PYN59JSwljttQORgO4MJGb5O8nwDbEM0cOPyKO8NpNftDnGr0MeBFJPVv2BBN2KfGdUO9/EyEPrHLfj7tchxBDkXE2Bk5muqA8MY+9cw5HoSw7aHSPd2beotDziYc9YtvbrmpdNc3HlMA0i/wAFAuh39k7che12HYEi5VdEmJ4ZG/yaTDuIsMNqz/wMZMSpjfJGd ";


    private VuforiaLocalizer vuforia;



    private TFObjectDetector tfod;

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);


    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    @Override
    public void runOpMode() {

        leftFrontMotor= hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor= hardwareMap.dcMotor.get("rightFront");
        leftRearMotor= hardwareMap.dcMotor.get("leftRear");
        rightRearMotor= hardwareMap.dcMotor.get("rightRear");


        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }




        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }



        if (tfod != null) {
            tfod.activate();
        }


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Orientation=1;
                moveForward(0.5,0.2);

                if (tfod != null) {

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> recognitions = tfod.getUpdatedRecognitions();
                    if (recognitions != null) {
                      telemetry.addData("# Object Detected", recognitions.size());

                      for (Recognition recognition : recognitions) {
                           if(recognition.getLabel() == LABEL_SECOND_ELEMENT){
                               double ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                               Found = true;


                               if(ObjectAngle == 0){

                                     SkyStonePosition=2;
                                     stoneGrabing();
                               }
                               if(ObjectAngle<=0){

                                   SkyStonePosition=1;
                                   moveLeft(0.3,0.25);
                                   stoneGrabing();

                               }
                               if(ObjectAngle>=0){

                                   SkyStonePosition=3;
                                   moveRight(0.3,0.25);
                                   stoneGrabing();

                               }


                           }


                      }


                    }



                }
                   moverightdistsensor(0.7,113.75);
                   releaseStone();
                   moveleftdistsensor(0.7,22.75);
                   moveleftdistsensor(0.3,SkyStonePosition*SkyStoneSize);
                   stoneGrabing();
                   moverightdistsensor(0.7,113.75);
                   releaseStone();
                   moverightdistsensor(0.7,156.5);
                   moveForward(0.1,0.1);
                   grabtray();
                   moveBackward(0.4,0.3);
                   rotate(0.3,90);
                   opentray();
                   parking(0.3);



                   if(Found)
                       break;

            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }















}
