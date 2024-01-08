package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Config
@Autonomous(group = "vision")
public class TeamElementVision extends LinearOpMode {
    public static int zone;
    public static boolean original = true;
    public static int width = 2304, height = 1536;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static boolean USE_WEBCAM = true;

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setLiveViewContainerId(cameraMonitorViewId)
                .setCameraResolution(new Size(1280, 960))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        CustomElementPipeline elementPipeCustom = new CustomElementPipeline();
        TeamElementPipeline elementPipeTeam = new TeamElementPipeline();

        if(original) camera.setPipeline(elementPipeTeam);
        else camera.setPipeline(elementPipeCustom);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                while(!opModeIsActive() && !isStopRequested()) {
                    telemetry.addLine("streaming");
                    if(original) zone = elementPipeTeam.get_element_zone();
                    else zone = elementPipeCustom.get_element_zone();
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
//            int zone = elementPipe.get_element_zone();
            telemetry.addLine("Element Zone" + zone);
            telemetry.update();
        }
    }
}
