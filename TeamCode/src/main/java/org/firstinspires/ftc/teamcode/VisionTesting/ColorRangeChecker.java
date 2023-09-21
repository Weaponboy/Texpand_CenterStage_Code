package org.firstinspires.ftc.teamcode.VisionTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Color Range Checker", group = "Testing")
public class ColorRangeChecker extends LinearOpMode {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "pixelcam"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new ColorRangePipeline());
        webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            // The pipeline will handle detection, so nothing is needed here.
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public static class ColorRangePipeline extends OpenCvPipeline {

        Mat hsvImage = new Mat();
        Mat mask = new Mat();

        // Replace these values with the HSV values for the color you want to detect
        Scalar lowerBound = new Scalar(100, 30, 30);
        Scalar upperBound = new Scalar(200, 200, 200);

        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

            // Threshold HSV image to select specific color
            Core.inRange(hsvImage, lowerBound, upperBound, mask);

            return mask; // Return the thresholded image
        }
    }
}
