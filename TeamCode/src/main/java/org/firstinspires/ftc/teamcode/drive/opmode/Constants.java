package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // drivetrain constants
    public static double axialCoefficient = 1, yawCoefficient = 1, lateralCoefficient = 1.1;
    public static double slowModePower = 0.5, regularPower = 1;

    // outtake constants
    public static double CLAW_MAX = 1, CLAW_MIN = 0.75;
    public static double ARM_GROUND = 0.29, ARM_MAX = 0.6, ARM_MIN = 0.0;
    public static double clawTime = 0.5, armTime = 0.7;

    // airplane constants
    public static double AIRPLANE_MAX = 1.0, AIRPLANE_MIN = 0.0;

    // linearslide constants: black black, red red for both
    public static double slideCoeff = 1;
    public static double linearF = 0.05, linearFThreshold = 1500;
    public static double armPreventionThreshold = 500, slidePositionMax = 2400;
    public static double linearLow = 0, linearHigh = 2000, linearError = 50;
    public static double linearKp = 4.8, linearKi = 0, linearKd = 0.5;

    // april tags
    public static double atGap = 4;
    public static double atOff = 5;

    // camera
}
