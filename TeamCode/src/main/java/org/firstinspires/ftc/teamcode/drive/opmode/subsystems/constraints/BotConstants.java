package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.constraints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.IntakeSingleClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.constraints.ArmConstraints;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.constraints.ClawConstraints;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.constraints.SlideConstraints;

@Config
public class BotConstants {
    // Slide Subsystem
    public OuttakeSlides slides;
    public static double p = 3, i = 0, d = 0, f = 0.09;
    public static boolean rev = true;
    public static String slideName = "linearSlide";
    public static int armPreventionThreshold = 500, slidePositionMax = 2000, linearFThreshold = 1000;
    public static int linearLow = 0, linearError = 50;

    // Arm Subsystem
    public OuttakeArm arm;
    public static double high = 0, raised = 0.5, ground = 0.7, drop = 0.95;
    public static String armName = "arm";

    // Claw Subsystem
    public IntakeSingleClaw claw;
    public static double openClaw = 0, closeClaw = 0.9;
    public static String clawName = "claw";
    public static double clawTime = 0.5, armTime = 0.5;


    public static ClawConstraints clawConstraints = new ClawConstraints(openClaw, closeClaw, openClaw);
    public static SlideConstraints slideConstraints = new SlideConstraints(0, rev, p, i, d, f);
    public static ArmConstraints armConstraints = new ArmConstraints(ground, high, raised, ground, drop);
}