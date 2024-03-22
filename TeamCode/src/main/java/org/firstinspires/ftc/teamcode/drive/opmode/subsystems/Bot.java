package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import static org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.BotConstants.armConstraints;
import static org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.BotConstants.clawConstraints;
import static org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.BotConstants.slideConstraints;
import static org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.BotConstants.visionConstraints;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Bot {
    private IntakeClaw claw;
    private OuttakeArm arm;
    private OuttakeSlides slides;
    private Vision vision;

    private LinearOpMode opMode;

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;

        this.claw = new IntakeClaw(opMode, "stackedClaw", "singleClaw", clawConstraints);
        this.arm = new OuttakeArm(opMode, "arm", armConstraints);
        this.slides = new OuttakeSlides(opMode, "slides", slideConstraints);
        this.vision = new Vision(opMode, "frontCamera", "backCamera", visionConstraints);
    }

    public void initializeTeleop() {
        this.claw.initialize();
        this.arm.initialize();
        this.slides.initialize();
        this.vision.initBackCamera();
    }

    public void powerSlides(double power) {
        this.slides.powerSlideRaw(power);
    }

    public void moveSlides(int height, int error) {
        this.slides.moveToPosition(height, error);
    }

    public void dropArm() {
        this.arm.drop();
    }

    public void raiseArm() {
        this.arm.raise();
    }

    public void groundArm() {
        this.arm.ground();
    }
}
