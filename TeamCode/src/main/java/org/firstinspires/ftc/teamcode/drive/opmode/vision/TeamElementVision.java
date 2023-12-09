package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class TeamElementVision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        CustomElementPipeline elementPipe = new CustomElementPipeline();

        camera.setPipeline(elementPipe);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                while(!opModeIsActive()) {
                    telemetry.addLine("streaming");
                    int zone = elementPipe.get_element_zone();
                    telemetry.addLine("Element Zone" + zone);
                    telemetry.update();
                }
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        waitForStart();
        while(opModeIsActive()) {
            int zone = elementPipe.get_element_zone();
            telemetry.addLine("Element Zone" + zone);
            telemetry.update();
        }
    }
}
