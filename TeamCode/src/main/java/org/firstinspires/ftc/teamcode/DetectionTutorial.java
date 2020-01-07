package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class DetectionTutorial {

    // Importam modelul de stone si skystone (deja pus pe telefon)
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    // declaram functia initVuforia pentru frumusetea codului. nu are legatura daca o folosim asa sau direct
    private void initVuforia() {

        //Initialiam parametrii
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Schimbam unii parametrii (api key si numele camerei video)
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwaremap.get(WebcamName.class, "Webcam 1");


        //initializam vuforia in sine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // la fel ca mai devreme, creem functia ajutatoare pt initializare TensorFlowObjDetection
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // Cheia API VUFORIA
    private static final String VUFORIA_KEY =
            "AayOVVP/////AAABmcIGEK4FhUaZid20JLpM9HEWkEmYrtH7T3/Mz9vqLUS7F0I4huZO58lGIawZmXDrfDjj0w8EoqVTjQ/6zdJ9flR9HFBZJ3qQVhG3I95nf6VEsQI8pTRpvEXFVqHjAuaV1XGykiLMNKLbwsLxYQxx+Jl7DUR18YqnOfpyAvPeY486v7sDhA6wNOYXz6Ir/giGoTMnFZV++7AUFCpAsLopvW4fMNMwAd5NnUsBPlblBTcDqxQn60NFrIcD01pTF3XcrqeEaieE4+Hr/oApZoCf7cMRbHxZNMmfxudjg4bJ0TKyOVho/ZoxkeAf8V+0Wl4iI8zSI0JWp9GJRW2fM55Zt9GxWNQKQq9WfKshjljrtNlu";


    // declaram obiectele vuforia (pt localizer) si TensorFlow detector (pt detectie in sine)
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // AICI INCEPE FUNCTIA RUNOPMODE

    //Initializam vuforia
    initVuforia();

    // daca se poate, initializam Tensorflow Object Detection (TFod e abreviere)
    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        initTfod();
    }

    //daca initializarea tfod a mers, il activam
    if (tfod != null) {
        tfod.activate();
    }

    // OPMODE IS ACTIVE (dupa ce apasa START)
    while (opModeIsActive()) {
        if (tfod != null) {

            // creem o lista cu recunoasterile
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            //daca avem ceva in lista, atunci
            if (updatedRecognitions != null) {

                //trecem prin fiecare cu for
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {

                    //getLabel returneaza 2 posibilitati : STONE, sau SKYSTONE
                    recognition.getLabel();

                    int ObjectAngle = recognition.estimateAngleToObject();

                    if(ObjectAngle > 0)




            }
        }
    }




}
