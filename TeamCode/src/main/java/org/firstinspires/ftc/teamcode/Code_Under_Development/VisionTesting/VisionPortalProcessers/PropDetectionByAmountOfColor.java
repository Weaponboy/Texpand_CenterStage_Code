package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.propPos;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Vision_Utils.VisionUtils;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PropDetectionByAmountOfColor implements VisionProcessor {

    private final PropDetectionByAmountOfColor.color color;
    public Rect rect = new Rect(20, 20, 50, 50);

    public Mat modifiedRight = new Mat();
    public Mat modifiedLeft = new Mat();

    public Scalar MIN_THRESH_RED = new Scalar(140, 50, 50);
    public Scalar MAX_THRESH_RED = new Scalar(220, 255, 255);

    public Scalar MIN_THRESH_BLUE = new Scalar(30, 70, 70);
    public Scalar MAX_THRESH_BLUE = new Scalar(90, 255, 255);

    public enum color{
        blue,
        red
    }

    public Rect leftOfScreen = new Rect(0, 0, 320, 240);

    public Rect rightOfScreen = new Rect(320, 0, 320, 240);

    public PropDetectionByAmountOfColor(color propColor){
        this.color = propColor;
    }

    private ArrayList<MatOfPoint> contoursLeft = new ArrayList<>();
    private ArrayList<MatOfPoint> contoursRight = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public double position1 = 0;
    public double position2 = 0;
    public double position3 = 0;
    double lastRectX;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frame.copyTo(modifiedLeft);
        frame.copyTo(modifiedRight);

        Imgproc.cvtColor(modifiedRight, modifiedRight, COLOR_RGB2HSV);
        Imgproc.cvtColor(modifiedLeft, modifiedLeft, COLOR_RGB2HSV);

        modifiedLeft.submat(leftOfScreen);
        modifiedRight.submat(rightOfScreen);

        switch (color) {
            case blue:
                inRange(modifiedLeft, MIN_THRESH_BLUE, MAX_THRESH_BLUE, modifiedLeft);
                inRange(modifiedRight, MIN_THRESH_RED, MAX_THRESH_RED, modifiedRight);
                break;
            case red:
                inRange(modifiedLeft, MIN_THRESH_RED, MAX_THRESH_RED, modifiedLeft);
                inRange(modifiedRight, MIN_THRESH_RED, MAX_THRESH_RED, modifiedRight);
                break;
            default:
                break;
        }

        //erode image
        erode(modifiedLeft, modifiedLeft, new Mat(5, 5, CV_8U));
        erode(modifiedRight, modifiedRight, new Mat(5, 5, CV_8U));

        //dilate image
        dilate(modifiedLeft, modifiedLeft, new Mat(5, 5, CV_8U));
        dilate(modifiedRight, modifiedRight, new Mat(5, 5, CV_8U));

        //find outlines of the objects of the colours in the range
        findContours(modifiedLeft, contoursLeft, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(modifiedRight, contoursRight, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);


        if (contoursLeft.size() - contoursRight.size() > 10){
            position1++;
        }else if (contoursRight.size() - contoursLeft.size() > 10){
            position2++;
        }else{
            position3++;
        }


        if (position1 > position2 && position1 > position3){
            propPos = 1;
        }else if (position2 > position1 && position2 > position3){
            propPos = 2;
        }else{
            propPos = 3;
        }

        contoursLeft.clear();
        contoursRight.clear();

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
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(leftOfScreen, scaleBmpPxToCanvasPx), rectPaint);

        Paint rectPaint2 = new Paint();
        rectPaint.setColor(Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(leftOfScreen, scaleBmpPxToCanvasPx), rectPaint2);
    }
}

