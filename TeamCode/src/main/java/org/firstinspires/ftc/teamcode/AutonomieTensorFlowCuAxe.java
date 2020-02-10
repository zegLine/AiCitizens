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


public class AutonomieTensorFlowCuAxe extends LinearOpMode {
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


    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private Servo trayservo1 = null;
    private Servo trayservo2 = null;

    private Servo lowarmUp = null;
    private Servo lowarmDown = null;





    public void moveForward(double power,double distance) {

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


        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();

    }

     public void moveBackward(double power,double distance) {

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

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
         telemetry.update();

    }

    public void moveLeft(double power,double distance) {

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

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }

    public void moveRight(double power,double distance) {

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
        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }

    public void turnLeft(double power,double distance) {

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
        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();

    }

    public void turnRight(double power,double distance) {

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

    public void movediagonalforwardright(double power,double distance){
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
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(power);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(Orientation==1){
            X=X+distance;
            Y=Y+distance;
        }
        if(Orientation==2){
            X=X+distance;
            Y=Y-distance;
        }
        if(Orientation==3){
            X=X-distance;
            Y=Y-distance;
        }
        if(Orientation==4){
            X=X-distance;
            Y=Y+distance;
        }

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }
    public void movediagonalbacwardright(double power,double distance){
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
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(-power);
        rightRearMotor.setPower(0);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(Orientation==1){
            X=X+distance;
            Y=Y-distance;
        }
        if(Orientation==2){
            X=X-distance;
            Y=Y-distance;
        }
        if(Orientation==3){
            X=X-distance;
            Y=Y+distance;
        }
        if(Orientation==4){
            X=X+distance;
            Y=Y+distance;
        }

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }

    public void movediagonalforwardleft(double power,double distance){
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
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(power);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(power);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(Orientation==1){
            X=X+distance;
            Y=Y+distance;
        }
        if(Orientation==2){
            X=X+distance;
            Y=Y-distance;
        }
        if(Orientation==3){
            X=X-distance;
            Y=Y-distance;
        }
        if(Orientation==4){
            X=X-distance;
            Y=Y+distance;
        }

        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
    }



    public void movement(double XN,double YN,double power){

        double difx;
        double dify;
        difx=XN-X;
        dify=YN-Y;
        if(Orientation == 1){
            moveForward(power,dify);
            moveRight(power,difx);
        }
        if(Orientation == 2){
            moveForward(power,difx);
            moveRight(power,dify);


        }

        X=XN;
        Y=YN;
        telemetry.addData("X=",X);
        telemetry.addData("Y=",Y);
        telemetry.update();
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

                   moveRight(0.6,SkyStoneSize*SkyStonePosition+2);
                   releaseStone();
                   moveLeft(0.6,3+SkyStonePosition*SkyStoneSize);
                   stoneGrabing();
                   moveRight(0.6,3+SkyStonePosition*SkyStoneSize);
                   releaseStone();
                   moveLeft(0.3,0.7);


                   if(Found)
                       break;

            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }















}
