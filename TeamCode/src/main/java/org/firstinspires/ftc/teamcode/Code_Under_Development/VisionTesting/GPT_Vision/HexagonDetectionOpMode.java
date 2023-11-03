package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.GPT_Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Disabled
public class HexagonDetectionOpMode extends LinearOpMode {

    public OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "pixelcam"), cameraMonitorViewId);

        // Camera setup from ColorRangeChecker
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                webcam.getExposureControl().setExposure(10, TimeUnit.MILLISECONDS);
                webcam.getGainControl().setGain(30);
                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;
                webcam.getFocusControl().setMode(focusmode);
                if (focusmode == FocusControl.Mode.Fixed) {
                    webcam.getFocusControl().setFocusLength(450);
                }
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Set pipeline
        webcam.setPipeline(new HexagonDetectionPipeline());

        // Start the camera stream to the dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        waitForStart();

        while (opModeIsActive()) {
            // The pipeline will handle everything
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
