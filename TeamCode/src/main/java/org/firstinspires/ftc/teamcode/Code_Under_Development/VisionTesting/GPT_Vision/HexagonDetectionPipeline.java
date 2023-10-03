package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.GPT_Vision;

import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class HexagonDetectionPipeline extends OpenCvPipeline {

    Mat output = new Mat();
    Mat hsvImage = new Mat();
    Mat greenMask = new Mat();
    Mat whiteMask = new Mat();
    Mat purpleMask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
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

        List<Rect> allRects = new ArrayList<>();

        for (int i = 0; i < masks.length; i++) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(masks[i], contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint cnt : contours) {
                if (Imgproc.contourArea(cnt) > 50) {
                    Rect rect = Imgproc.boundingRect(cnt);
                    allRects.add(rect);
                }
            }
        }

        for (Rect rect : allRects) {
            if (!isInsideAnother(rect, allRects)) {
                drawHexagon(rect, output, new Scalar(0, 255, 0));
            }
        }

        return output;
    }

    private void drawHexagon(Rect rect, Mat output, Scalar color) {
        int xCenter = rect.x + rect.width / 2;
        int yCenter = rect.y + rect.height / 2;
        int radius = Math.min(rect.width, rect.height) / 2;

        Point[] hexagon = new Point[6];
        for (int i = 0; i < 6; i++) {
            double angle = Math.PI / 3 * i + Math.PI / 6; // Add Math.PI / 6 for a 30-degree rotation
            int x = (int) (xCenter + radius * Math.cos(angle));
            int y = (int) (yCenter + radius * Math.sin(angle));
            hexagon[i] = new Point(x, y);
        }

        MatOfPoint hexMat = new MatOfPoint();
        hexMat.fromArray(hexagon);

        List<MatOfPoint> hexList = new ArrayList<>();
        hexList.add(hexMat);

        Imgproc.polylines(output, hexList, true, color, 2);
    }

    private boolean isInsideAnother(Rect rect, List<Rect> allRects) {
        for (Rect otherRect : allRects) {
            if (otherRect != rect && otherRect.contains(rect.tl()) && otherRect.contains(rect.br())) {
                return true;
            }
        }
        return false;
    }
}
