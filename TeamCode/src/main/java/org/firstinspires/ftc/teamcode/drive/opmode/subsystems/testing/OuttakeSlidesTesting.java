package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeSlides;

@Config
@Autonomous
public class OuttakeSlidesTesting extends LinearOpMode {
    public static double p = 3, i = 0, d = 0, f = 0;
    public static String name = "linearSlide";
    public static int start = 0;
    public static boolean rev = true;

    public static int pos = 500, error = 50;

    public OuttakeSlides slides;
    public OuttakeArm arm;

    @Override
    public void runOpMode() throws InterruptedException {
    }
}
