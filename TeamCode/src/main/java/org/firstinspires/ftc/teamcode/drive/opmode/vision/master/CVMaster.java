package org.firstinspires.ftc.teamcode.drive.opmode.vision.master;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.CustomElementPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.TeamElementPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class CVMaster {
    public OpenCvWebcam camera;
    public AprilMaster aprilTag;
    public TeamElementPipeline basicPipeline;
    public CustomElementPipeline advancedPipeline;

    public LinearOpMode op;

    public boolean streaming = true;
    public boolean red = true;

    public static int width = 1280, height = 960;

    public int zone = 1;

    public CVMaster(LinearOpMode op) {
        this.op = op;

        int cameraMonitorViewId = this.op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.op.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(this.op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
    }

    public int getPosition() {
        CustomElementPipeline elementPipe = new CustomElementPipeline();

        camera.setPipeline(elementPipe);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
                while(!op.opModeIsActive()) {
                    op.telemetry.addLine("streaming");
                    zone = elementPipe.get_element_zone();
                    op.telemetry.addLine("Element Zone" + zone);
                    op.telemetry.update();
                }
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        return zone;
    }
}
