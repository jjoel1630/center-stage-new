package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drivetrain extends SampleMecanumDrive {
    private boolean teleop = false;

    public Drivetrain(HardwareMap hwMap) {
        super(hwMap);
    }


}
