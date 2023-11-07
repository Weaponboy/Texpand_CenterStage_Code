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

public class pixelDetection implements VisionProcessor {

    private final pixelDetection.pixelColor pixelColor;
    public Rect rect = new Rect(20, 20, 50, 50);

    public Mat greenPixel = new Mat();
    public Mat yellowPixel = new Mat();
    public Mat purplePixel = new Mat();
    public Mat whitePixel = new Mat();

    public Scalar MIN_THRESH_PURPLE = new Scalar(84, 82, 87);
    public Scalar MAX_THRESH_PURPLE = new Scalar(156, 155, 155);

    public Scalar MIN_THRESH_GREEN = new Scalar(22, 74, 21);
    public Scalar MAX_THRESH_GREEN = new Scalar(80, 120, 56);

    public Scalar MIN_THRESH_WHITE = new Scalar(255, 255, 255);
    public Scalar MAX_THRESH_WHITE = new Scalar(255, 255, 255 );

    public Scalar MIN_THRESH_YELLOW = new Scalar(117, 108, 57);
    public Scalar MAX_THRESH_YELLOW = new Scalar(157, 153, 105);

    public enum pixelColor{
        purple,
        yellow,
        green,
        white,
        all
    }

    public pixelDetection(pixelColor propColor){
        this.pixelColor = propColor;
    }
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private ArrayList<MatOfPoint> yellowcontours = new ArrayList<>();
    private ArrayList<MatOfPoint> whitecontours = new ArrayList<>();
    private ArrayList<MatOfPoint> purplecontours = new ArrayList<>();
    private ArrayList<MatOfPoint> greencontours = new ArrayList<>();

    private Mat hierarchy = new Mat();

    private List<Rect> whiterects = new ArrayList<>();
    private List<Rect> yellowrects = new ArrayList<>();
    private List<Rect> purplerects = new ArrayList<>();
    private List<Rect> greenrects = new ArrayList<>();

    private List<Rect> GreenPixelsRawHeight = new ArrayList<>();
    private List<Rect> GreenPixelsRawWidth = new ArrayList<>();
    private List<Rect> GreenPixels = new ArrayList<>();

    private List<Rect> PurplePixelsRawHeight = new ArrayList<>();
    private List<Rect> PurplePixelsRawWidth = new ArrayList<>();
    private List<Rect> PurplePixels = new ArrayList<>();

    private List<Rect> WhitePixelsRawHeight = new ArrayList<>();
    private List<Rect> WhitePixelsRawWidth = new ArrayList<>();
    private List<Rect> WhitePixels = new ArrayList<>();

    private List<Rect> YellowPixelsRawHeight = new ArrayList<>();
    private List<Rect> YellowPixelsRawWidth = new ArrayList<>();
    private List<Rect> YellowPixels = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        frame.copyTo(greenPixel);
        frame.copyTo(whitePixel);
        frame.copyTo(yellowPixel);
        frame.copyTo(purplePixel);

        Imgproc.cvtColor(greenPixel, greenPixel, COLOR_RGB2HSV);
        Imgproc.cvtColor(yellowPixel, yellowPixel, COLOR_RGB2HSV);
        Imgproc.cvtColor(purplePixel, purplePixel, COLOR_RGB2HSV);
        Imgproc.cvtColor(whitePixel, whitePixel, COLOR_RGB2HSV);

        inRange(greenPixel, MIN_THRESH_GREEN, MAX_THRESH_GREEN, greenPixel);
        inRange(yellowPixel, MIN_THRESH_YELLOW, MAX_THRESH_YELLOW, yellowPixel);
        inRange(purplePixel, MIN_THRESH_PURPLE, MAX_THRESH_PURPLE, purplePixel);
        inRange(whitePixel, MIN_THRESH_WHITE, MAX_THRESH_WHITE, whitePixel);

        //erode image
        erode(whitePixel, whitePixel, new Mat(5, 5, CV_8U));
        erode(purplePixel, purplePixel, new Mat(5, 5, CV_8U));
        erode(yellowPixel, yellowPixel, new Mat(5, 5, CV_8U));
        erode(greenPixel, greenPixel, new Mat(5, 5, CV_8U));

        //dilate image
        dilate(whitePixel, whitePixel, new Mat(5, 5, CV_8U));
        dilate(purplePixel, purplePixel, new Mat(5, 5, CV_8U));
        dilate(yellowPixel, yellowPixel, new Mat(5, 5, CV_8U));
        dilate(greenPixel, greenPixel, new Mat(5, 5, CV_8U));

        //find outlines of the objects of the colours in the range
        findContours(whitePixel, whitecontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(purplePixel, purplecontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(yellowPixel, yellowcontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(greenPixel, greencontours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < whitecontours.size(); i++){
            Rect rect = boundingRect(whitecontours.get(i));
            whiterects.add(rect);
        }
        for (int i = 0; i < yellowcontours.size(); i++){
            Rect rect = boundingRect(yellowcontours.get(i));
            yellowrects.add(rect);
        }
        for (int i = 0; i < purplecontours.size(); i++){
            Rect rect = boundingRect(purplecontours.get(i));
            purplerects.add(rect);
        }
        for (int i = 0; i < greencontours.size(); i++){
            Rect rect = boundingRect(greencontours.get(i));
            greenrects.add(rect);
        }

        GreenPixelsRawHeight = VisionUtils.sortRectsByMaxOption(greenrects.size(), VisionUtils.RECT_OPTION.HEIGHT, greenrects);
        GreenPixelsRawWidth = VisionUtils.sortRectsByMaxOption(greenrects.size(), VisionUtils.RECT_OPTION.WIDTH, greenrects);

        for (int i = 0; i < GreenPixelsRawHeight.size(); i++){
            if (Math.abs((GreenPixelsRawHeight.get(i).height)*(GreenPixelsRawWidth.get(i).width)) > 0.75 && Math.abs((GreenPixelsRawHeight.get(i).height)*(GreenPixelsRawWidth.get(i).width)) < 0.85) {
                Rect rect = GreenPixelsRawHeight.get(i);
                GreenPixels.add(rect);
            }
        }

        YellowPixelsRawHeight = VisionUtils.sortRectsByMaxOption(yellowrects.size(), VisionUtils.RECT_OPTION.HEIGHT, yellowrects);
        YellowPixelsRawWidth = VisionUtils.sortRectsByMaxOption(yellowrects.size(), VisionUtils.RECT_OPTION.WIDTH, yellowrects);


        for (int i = 0; i < GreenPixelsRawHeight.size(); i++){
            if (Math.abs((GreenPixelsRawHeight.get(i).height)*(GreenPixelsRawWidth.get(i).width)) > 0.75 && Math.abs((GreenPixelsRawHeight.get(i).height)*(GreenPixelsRawWidth.get(i).width)) < 0.85) {
                Rect rect = GreenPixelsRawHeight.get(i);
                GreenPixels.add(rect);
            }
        }

        whitecontours.clear();
        yellowcontours.clear();
        greencontours.clear();
        purplecontours.clear();

        whiterects.clear();
        yellowrects.clear();
        greenrects.clear();
        purplerects.clear();

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

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }
}

