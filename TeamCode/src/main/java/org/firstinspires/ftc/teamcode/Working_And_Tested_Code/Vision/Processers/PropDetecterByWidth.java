package org.firstinspires.ftc.teamcode.Working_And_Tested_Code.Vision.Processers;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.RED_LOWER_Cb_PROP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.RED_LOWER_Cr_PROP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.RED_LOWER_Y_PROP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.RED_UPPER_Cb_PROP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.RED_UPPER_Cr_PROP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.RED_UPPER_Y_PROP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.dilate_const;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.Constants.ColorConstants.erode_const;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2YCrCb;
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

public class PropDetecterByWidth implements VisionProcessor {

    public Rect rect = new Rect(20, 20, 50, 50);

    public Mat modifiedMat = new Mat();

    public Scalar MIN_THRESH_RED = new Scalar(RED_LOWER_Y_PROP, RED_LOWER_Cr_PROP, RED_LOWER_Cb_PROP);
    public Scalar MAX_THRESH_RED = new Scalar(RED_UPPER_Y_PROP, RED_UPPER_Cr_PROP, RED_UPPER_Cb_PROP);

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private List<Rect> rects = new ArrayList<>();
    private List<Rect> OrderedByWidthrects = new ArrayList<>();
    private int HighRect = -1;
    private Rect TargetHighRect;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frame.copyTo(modifiedMat);

        Imgproc.cvtColor(modifiedMat, modifiedMat, COLOR_RGB2YCrCb);

        //Apply colour scales
        inRange(modifiedMat, MIN_THRESH_RED, MAX_THRESH_RED, modifiedMat);

        //erode image
        erode(modifiedMat, modifiedMat, new Mat(erode_const, erode_const, CV_8U));

        //dilate image
        dilate(modifiedMat, modifiedMat, new Mat(dilate_const, dilate_const, CV_8U));

        //find outlines of the objects of the colours in the range
        findContours(modifiedMat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        if (rects.size() > 0) {

            OrderedByWidthrects = VisionUtils.sortRectsByMaxOption(rects.size(), VisionUtils.RECT_OPTION.WIDTH, rects);

            //find the widths expected for a high pole and a medium pole
            for (int i = 0; i < OrderedByWidthrects.size(); i++) {
                if (OrderedByWidthrects.get(i).width < 105 && (OrderedByWidthrects.get(i).width > 90)) {
                    HighRect = i;
                }
            }
            if (HighRect > -1) {
                TargetHighRect = OrderedByWidthrects.get(HighRect);

                rect = new Rect(TargetHighRect.x, TargetHighRect.y, TargetHighRect.width, TargetHighRect.height);
            }else {
                rect = new Rect(0,0,0,0);
            }
        }

        contours.clear();
        rects.clear();
        HighRect = -1;

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

