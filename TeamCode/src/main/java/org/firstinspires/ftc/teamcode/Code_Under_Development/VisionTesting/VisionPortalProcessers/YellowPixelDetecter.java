package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.GREEN_LOWER_H_PAPER;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.GREEN_LOWER_S_PAPER;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.GREEN_LOWER_V_PAPER;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.GREEN_UPPER_H_PAPER;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.GREEN_UPPER_S_PAPER;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.GREEN_UPPER_V_PAPER;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.dilate_const;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.erode_const;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class YellowPixelDetecter implements VisionProcessor{

    public Rect rect;

    public Mat modifiedMat = new Mat();

    public Scalar MIN_THRESH_GREEN = new Scalar(GREEN_LOWER_H_PAPER, GREEN_LOWER_V_PAPER, GREEN_LOWER_S_PAPER);
    public Scalar MAX_THRESH_GREEN = new Scalar(GREEN_UPPER_H_PAPER, GREEN_UPPER_V_PAPER, GREEN_UPPER_S_PAPER);

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private List<Rect> rects = new ArrayList<>();
    private Scalar black = new Scalar(0,0,0);
    private Scalar green = new Scalar(0, 255, 0); //green
    double PixelWidth = 0;
    double PixelX = 0;
    double PixelY = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frame.copyTo(modifiedMat);

        Imgproc.cvtColor(modifiedMat, modifiedMat, COLOR_RGB2HSV);

        //Apply colour scales
        inRange(modifiedMat, MIN_THRESH_GREEN, MAX_THRESH_GREEN, modifiedMat);

        //erode image
        erode(modifiedMat, modifiedMat, new Mat(erode_const, erode_const, CV_8U));

        //dilate image
        dilate(modifiedMat, modifiedMat, new Mat(dilate_const, dilate_const, CV_8U));

        //find outlines of the objects of the colours in the range
        findContours(modifiedMat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        if(rects.size() > 0) {
            rect = new Rect(rect.x, rect.y, rect.width, rect.height);
        }else {
            rect = new Rect(0, 0, 0, 0);
        }

        contours.clear();
        rects.clear();

        return null;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {

        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }
}
