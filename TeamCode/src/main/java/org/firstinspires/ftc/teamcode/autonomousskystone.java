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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;







@Autonomous(name = "AutonomousSkyStonedetection", group = "Concept")

public class autonomousskystone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";



    private static final String VUFORIA_KEY =
            " AWh3WbD/////AAABmQr66RjvbkVtr+RI6oomXqIgCzVDQtdjwkNT4jkW0JBVLrq3rymbi6vq3sBtaFBrD4rYqleNmM9WFwZWYYNka48h4t85scS+/g7cTt0g84GiuI3J8uqDqL4IKpVlu+JLSEW9J0KkuoQSksN0RIxVCqC87a2MKMF9IRUuSz35PYN59JSwljttQORgO4MJGb5O8nwDbEM0cOPyKO8NpNftDnGr0MeBFJPVv2BBN2KfGdUO9/EyEPrHLfj7tchxBDkXE2Bk5muqA8MY+9cw5HoSw7aHSPd2beotDziYc9YtvbrmpdNc3HlMA0i/wAFAuh39k7che12HYEi5VdEmJ4ZG/yaTDuIsMNqz/wMZMSpjfJGd ";


    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    DcMotor leftDrive;
    DcMotor rightDrive;
    Servo lowArmBot;
    Servo lowArmHigh;



    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            leftDrive = hardwareMap.dcMotor.get("left_drive");
            rightDrive = hardwareMap.dcMotor.get("right_drive");
            lowArmBot = hardwareMap.servo.get("low_arm_bot");
            lowArmHigh = hardwareMap.servo.get("low_arm_high");

            leftDrive.setDirection(DcMotor.Direction.FORWARD);

           rightDrive.setDirection(DcMotor.Direction.REVERSE);
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();



            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();
            }

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();
            waitForStart();


            {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }






        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


    }








    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);


        telemetry.update();
        double TargetHeightRatio = 0.8;

        boolean Skystonefound = false;


        if (tfod != null) {

            // creem o lista cu recunoasterile
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            //daca avem ceva in lista, atunci
            if (updatedRecognitions != null) {

                //trecem prin fiecare cu for
                int i = 0;
                for (Recognition recognition : updatedRecognitions)
                {

                    double ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);



                    double LeftPower, RightPower;

                    LeftPower = (ObjectAngle/45)*0.25;
                    RightPower = (ObjectAngle/45)*-0.25;

                    double ImageHeight, ObjectHeight, ObjectHeightRatio;
                    ImageHeight = recognition.getImageHeight();
                    ObjectHeight = recognition.getHeight();
                    ObjectHeightRatio = ObjectHeight/ImageHeight;

                    if(ObjectHeightRatio<(TargetHeightRatio-0.05));
                    {
                        if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.2) {
                            LeftPower = (((TargetHeightRatio - 0.05) - ObjectHeightRatio) * 0.5) + 0.035;
                            RightPower = LeftPower;
                        } else if (ObjectHeightRatio > (TargetHeightRatio + 0.05)) ;
                        if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.12) {
                            LeftPower = (((TargetHeightRatio + 0.05) - TargetHeightRatio) * -0.5) + -0.05;
                            RightPower = LeftPower;
                        } else if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.12) {
                            LeftPower = 0;
                            RightPower = 0;
                            lowArmBot.setPosition(0.5);
                            lowArmHigh.setPosition(0.5);

                            leftDrive.setPower(-1);
                            rightDrive.setPower(-1);

                            sleep(1300);

                            leftDrive.setPower(0.6);
                            rightDrive.setPower(-0.6);

                            sleep(500);

                            leftDrive.setPower(1);
                            rightDrive.setPower(1);

                            sleep(2500);

                            lowArmBot.setPosition(0);
                            lowArmHigh.setPosition(0);

                            leftDrive.setPower(-1);
                            rightDrive.setPower(-1);

                            sleep(1300);





                        }

                    }
                    leftDrive.setPower(LeftPower);
                    rightDrive.setPower(RightPower);
                    break;


                }






            }
        }


        tfod.deactivate();
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }





}







