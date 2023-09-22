package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Hexagon Detection OpMode", group = "Testing")
public class HexagonDetectionOpMode extends LinearOpMode {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "pixelcam"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new HexagonDetectionPipeline());
        webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            // The pipeline will handle detection, so nothing is needed here.
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}