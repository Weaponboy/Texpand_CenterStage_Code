package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class ColorRangeChecker extends LinearOpMode {

    FtcDashboard dashboard;

    public static double lowerH = 100;
    public static double lowerS = 90;
    public static double lowerV = 90;

    public static double upperH = 180;
    public static double upperS = 255;
    public static double upperV = 255;

    public static double cameraGain = 30; // Range: 0-255
    public static double cameraExposure = 7; // Range: 0-10000

    public OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.openCameraDevice();

        // Set initial gain and exposure before starting streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);

                webcam.getExposureControl().setExposure(7, TimeUnit.MILLISECONDS);

                webcam.getGainControl().setGain(30);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                webcam.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    webcam.getFocusControl().setFocusLength(450);
                }

                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });

        webcam.setPipeline(new ColorRangePipeline());

        FtcDashboard.getInstance().startCameraStream(webcam,30);


        waitForStart();

        while (opModeIsActive()) {
            // Dynamically update gain and exposure
            webcam.getGainControl().setGain((int) cameraGain);
            webcam.getExposureControl().setExposure((int) cameraExposure, TimeUnit.MILLISECONDS);

            // Create a telemetry packet for the dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Lower H", lowerH);
            packet.put("Lower S", lowerS);
            packet.put("Lower V", lowerV);
            packet.put("Upper H", upperH);
            packet.put("Upper S", upperS);
            packet.put("Upper V", upperV);
            packet.put("Gain", cameraGain);
            packet.put("Exposure", cameraExposure);
            dashboard.sendTelemetryPacket(packet);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public static class ColorRangePipeline extends OpenCvPipeline {

        Mat hsvImage = new Mat();
        Mat mask = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsvImage, new Scalar(lowerH, lowerS, lowerV), new Scalar(upperH, upperS, upperV), mask);
            return mask;
        }
    }
}
