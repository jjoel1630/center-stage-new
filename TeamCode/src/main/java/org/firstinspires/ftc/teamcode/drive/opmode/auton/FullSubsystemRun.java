package org.firstinspires.ftc.teamcode.drive.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FullSubsystemRun extends LinearOpMode {
    private DcMotorEx intakeMotor = null; //setting intake motor variable
    private Servo outtakeServo;

    private Servo airplane;

    private DcMotorEx leftLS = null;
    private DcMotorEx rightLS = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // drive train
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // linearSlide
        leftLS = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");
        rightLS = hardwareMap.get(DcMotorEx.class, "linearSlideRight");

        // intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        // reverse intake
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeServo = hardwareMap.servo.get("outtakeServo");

        // April tag localization

        // Path Building
        Pose2d start = new Pose2d(-36.37, -62.20, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence defaulTraj = drive.trajectorySequenceBuilder(new Pose2d(-36.37, -62.20, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.37, -49.44), Math.toRadians(90.00))
                .lineToLinearHeading(new Pose2d(-44.39, -40.38, Math.toRadians(124.11)))
                .lineToLinearHeading(new Pose2d(-54.19, -60.57, Math.toRadians(0.00)))
                .splineTo(new Vector2d(12.17, -60.57), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    // PID tune slides
                    // raise slides
                })
                .splineTo(new Vector2d(50.77, -36.67), Math.toRadians(0.00))
                .build();

        waitForStart();

        while(opModeIsActive()) {
            // detect team object
            // run defaultTraj
            // if correct apriltag is not in sight, then just drop the pixel
            // otherwise, if its in the range and in correct apriltag, then drop pixel
            // otherwise, if its not in range and in correct apriltag, then set another roadrunner path to go there

        }
    }
}
