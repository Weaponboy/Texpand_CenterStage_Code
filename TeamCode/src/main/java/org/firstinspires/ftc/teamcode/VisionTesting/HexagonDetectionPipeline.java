package org.firstinspires.ftc.teamcode.VisionTesting;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class HexagonDetectionPipeline extends OpenCvPipeline {

    // Initialize Mat objects here to avoid memory leak
    Mat output = new Mat();
    Mat hsvImage = new Mat();
    Mat greenMask = new Mat();
    Mat whiteMask = new Mat();
    Mat purpleMask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Reuse existing Mat objects
        input.copyTo(output);
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsvImage, new Scalar(ColorConstants.GREEN_LOWER_H, ColorConstants.GREEN_LOWER_S, ColorConstants.GREEN_LOWER_V),
                new Scalar(ColorConstants.GREEN_UPPER_H, ColorConstants.GREEN_UPPER_S, ColorConstants.GREEN_UPPER_V), greenMask);

        Core.inRange(hsvImage, new Scalar(ColorConstants.WHITE_LOWER_H, ColorConstants.WHITE_LOWER_S, ColorConstants.WHITE_LOWER_V),
                new Scalar(ColorConstants.WHITE_UPPER_H, ColorConstants.WHITE_UPPER_S, ColorConstants.WHITE_UPPER_V), whiteMask);

        Core.inRange(hsvImage, new Scalar(ColorConstants.PURPLE_LOWER_H, ColorConstants.PURPLE_LOWER_S, ColorConstants.PURPLE_LOWER_V),
                new Scalar(ColorConstants.PURPLE_UPPER_H, ColorConstants.PURPLE_UPPER_S, ColorConstants.PURPLE_UPPER_V), purpleMask);

        Mat[] masks = {greenMask, whiteMask, purpleMask};
        String[] colors = {"Green", "White", "Purple"};

        for (int i = 0; i < masks.length; i++) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(masks[i], contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint cnt : contours) {
                if (Imgproc.contourArea(cnt) > 50) {
                    Rect rect = Imgproc.boundingRect(cnt);
                    Scalar color = new Scalar(0, 255, 0); // Default to green

                    switch (colors[i]) {
                        case "Green":
                            color = new Scalar(0, 128, 0);
                            break;
                        case "White":
                            color = new Scalar(255, 255, 255);
                            break;
                        case "Purple":
                            color = new Scalar(255, 0, 255);
                            break;
                    }

                    Imgproc.rectangle(output, rect.tl(), rect.br(), color, 2);
                    Imgproc.putText(output, colors[i], new Point(rect.x, rect.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 255, 255), 2);
                }
            }
        }

        return output;
    }
}
