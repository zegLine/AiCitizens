package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ACDriveHardware {

    public void initializeHardware (ACRobot.ACDriveHardwareConfig config) {
        switch (config){
            case MecanumDrive:
                initializeMecanum();
            case OmniDrive:
                initializeOmni();
        }
    }

    public void initializeMecanum () {
        DcMotor leftFrontMotor = null;
        DcMotor rightFrontMotor = null;
        DcMotor leftRearMotor = null;
        DcMotor rightRearMotor = null;
    }

    public void initializeOmni () {
        DcMotor leftDrive = null;
        DcMotor rightDrive = null;
    }

}
