package org.firstinspires.ftc.teamcode.drive.opmode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class DSensorTest extends OpMode {
    DistanceSensor dsensor;

    @Override
    public void init()
    {
        dsensor = hardwareMap.get(DistanceSensor.class,"distance sensor");
    }
    public void distance ()
    {
        double value_cm = dsensor.getDistance(DistanceUnit.CM);
        double value_in = dsensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("Distance Inches:", value_in);
        telemetry.addData("Distance CM:",value_cm);

    }

    @Override
    public void loop()
    {
        distance();
    }

}
