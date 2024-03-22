package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.VisionConstraints;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.BlueLeftPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.BluePipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.RedPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.RedRightPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class Vision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private OpenCvCamera frontCamera;

    private BluePipeline blueRightPipeline;
    private BlueLeftPipeline blueLeftPipeline;
    private RedRightPipeline redRightPipeline;
    private RedPipeline redLeftPipeline;
    private String currentPipeline = "blueRight";

    private LinearOpMode opMode;

    private int zone = 1;

    private int frontCameraWidth, frontCameraHeight;
    private int backCameraWidth, backCameraHeight;

    private String backCamName, frontCamName;

    public Vision(LinearOpMode opMode, String backCamName, String frontCamName, VisionConstraints visionConstraints) {
        this.opMode = opMode;

        this.frontCameraHeight = visionConstraints.getFrontCameraHeight();
        this.frontCameraWidth = visionConstraints.getFrontCameraWidth();
        this.backCameraHeight = visionConstraints.getBackCameraHeight();
        this.backCameraWidth = visionConstraints.getBackCameraWidth();

        this.frontCamName = frontCamName;
        this.backCamName = backCamName;
    }

    public void initBackCamera() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .setCamera(this.opMode.hardwareMap.get(WebcamName.class, backCamName))
//                .setLiveViewContainerId(cameraMonitorViewId)
                .setCameraResolution(new Size(backCameraWidth, backCameraHeight))
                .build();

        this.opMode.telemetry.addLine("Successfully Initialized Back Camera (ID): " + cameraMonitorViewId);
        this.opMode.telemetry.update();
    }

    public void initFrontCamera(OpenCvPipeline elementPipeline) {
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(this.opMode.hardwareMap.get(WebcamName.class, frontCamName), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(frontCamera, 0);

        if(elementPipeline instanceof BluePipeline) {
            blueRightPipeline = (BluePipeline) elementPipeline;
            frontCamera.setPipeline(blueRightPipeline);
            this.opMode.telemetry.addLine("Successfully Set Pipeline BlueRightPipeline");
        }
        if(elementPipeline instanceof BlueLeftPipeline) {
            blueLeftPipeline = (BlueLeftPipeline) elementPipeline;
            frontCamera.setPipeline(blueLeftPipeline);
            this.opMode.telemetry.addLine("Successfully Set Pipeline BlueLeftPipeline");
        }
        if(elementPipeline instanceof RedRightPipeline) {
            redRightPipeline = (RedRightPipeline) elementPipeline;
            frontCamera.setPipeline(redRightPipeline);
            this.opMode.telemetry.addLine("Successfully Set Pipeline RedRightPipeline");
        }
        if(elementPipeline instanceof RedPipeline) {
            redLeftPipeline = (RedPipeline) elementPipeline;
            frontCamera.setPipeline(redLeftPipeline);
            this.opMode.telemetry.addLine("Successfully Set Pipeline RedLeftPipeline");
        }

        this.opMode.telemetry.addLine("Successfully Initialized Front Camera (ID): " + cameraMonitorViewId);
        this.opMode.telemetry.update();
    }

    public void streamFrontCamera() {
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                frontCamera.startStreaming(frontCameraWidth, frontCameraHeight, OpenCvCameraRotation.UPRIGHT);
                while(!opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    opMode.telemetry.addLine("Camera Successfully Streaming");

                    if(currentPipeline == "blueRight") zone = blueRightPipeline.get_element_zone();
                    if(currentPipeline == "redRight") zone = redRightPipeline.get_element_zone();
                    if(currentPipeline == "blueLeft") zone = blueLeftPipeline.get_element_zone();
                    if(currentPipeline == "redLeft") zone = redLeftPipeline.get_element_zone();

                    opMode.telemetry.addLine("Current Element Zone: " + zone);
                    opMode.telemetry.update();
                }
            }

            @Override
            public void onError(int errorCode)
            {
                opMode.telemetry.addLine("Error When Streaming Camera: Code " + errorCode);
                opMode.telemetry.update();
            }
        });
    }

    public List<AprilTagDetection> getTagDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());
        opMode.telemetry.update();

        return currentDetections.size() > 0 ? currentDetections : null;
    }
}
