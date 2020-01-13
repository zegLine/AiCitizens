package org.firstinspires.ftc.teamcode;

public class ACRobot {

    public enum ACDriveHardwareConfig {
        MecanumDrive,
        OmniDrive
    }

    private ACDriveHardwareConfig ACCurrentConfig = null;

    public ACRobot (ACDriveHardwareConfig driverconfig) {
        switch (driverconfig) {
            case MecanumDrive:
                ACCurrentConfig = ACDriveHardwareConfig.MecanumDrive;
            case OmniDrive:
                ACCurrentConfig = ACDriveHardwareConfig.OmniDrive;
        }
    }

    public static void moveRight (double time) {

    }


}
