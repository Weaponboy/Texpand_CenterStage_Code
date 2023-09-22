package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class HexagonDetectionPipeline extends OpenCvPipeline {

    private Mat output = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        // Copy the input frame to the output frame for visualization
        input.copyTo(output);

        // Convert to grayscale
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply Gaussian blur
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);

        // Edge detection using Canny
        Mat edged = new Mat();
        Imgproc.Canny(blurred, edged, 50, 150);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edged, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop over the contours to find hexagons
        for (MatOfPoint cnt : contours) {
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(cnt2f, true);  // Reduced approximation error
            Imgproc.approxPolyDP(cnt2f, approx, epsilon, true);

            // Check if it's a hexagon (6 sides) and has a minimum size
            if (approx.total() == 6 && Imgproc.contourArea(cnt) > 50) {
                // Draw bounding rectangle
                Rect rect = Imgproc.boundingRect(cnt);

                // Identify color and label it
                double[] color = input.get((int) (rect.y + rect.height / 2), (int) (rect.x + rect.width / 2));
                String colorLabel = identifyColor(color);
                Scalar boxColor = getColorScalar(colorLabel);

                Imgproc.rectangle(output, rect.tl(), rect.br(), boxColor, 2);
                Imgproc.putText(output, colorLabel, new Point(rect.x, rect.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
            }
        }

        return output;
    }
    private Scalar getColorScalar(String colorLabel) {
        switch (colorLabel) {
            case "White":
                return new Scalar(255, 255, 255);
            case "Purple":
                return new Scalar(255, 0, 255);
            case "Yellow":
                return new Scalar(0, 255, 255);
            case "Green":
                return new Scalar(0, 128, 0);
            default:
                return new Scalar(0, 0, 0);
        }
    }


    private String identifyColor(double[] color) {
        // RGB color detection logic
        if (color[0] > 200 && color[1] > 200 && color[2] > 200) {
            return "White";
        } else if (color[0] < 50 && color[1] > 100 && color[2] > 100) {
            return "Purple";
        } else if (color[0] > 200 && color[1] > 200 && color[2] < 100) {
            return "Yellow";
        } else if (color[0] < 100 && color[1] > 100 && color[2] < 100) {
            return "Green";
        }
        return "Unknown";
    }
}


