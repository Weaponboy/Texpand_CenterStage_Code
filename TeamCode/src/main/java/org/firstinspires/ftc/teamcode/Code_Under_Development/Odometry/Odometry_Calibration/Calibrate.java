package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.Odometry_Calibration;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Calibrate extends OpMode {

    public static double trackwidth = 35.95;
    public static double centerPodOffset = 10.768;
    public static double wheelRadius = 1.75;
    public static double podTicks = 8192;

    public static double cm_per_tick = 2.0 * Math.PI * wheelRadius / podTicks;

    public int currentRightPod = 0;
    public int currentLeftPod = 0;
    public int currentCenterPod = 0;

    public int oldRightPod = 0;
    public int oldLeftPod = 0;
    public int oldCenterPod = 0;

    public double startX, startY, startHeading;

    public static PIDFController drivePID;
    public static PIDFController strafePID;
    public static PIDFController PivotPID;

    public static double Xdist = 0;
    public static double Ydist = 0;

    public static double rotdist = 0;

    public static double XdistForStop = 0;
    public static double YdistForStop = 0;

    public static double rotdistForStop = 0;

    public static double RRXdist = 0;
    public static double RRYdist = 0;
    public static double Horizontal = 0;
    public static double Vertical = 0;

    public static double Pivot = 0;

    public static double ConvertedHeading = 0;

    public double X = startX, Y = startY, heading = startHeading;

    public double dtheta;

    public double dx;
    public double dy;

    public double factor = 0;

    public BNO055IMU imu = null;

    Orientation YawAngle;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        oldCenterPod = currentCenterPod;
        oldLeftPod = currentLeftPod;
        oldRightPod = currentRightPod;

        currentCenterPod = 0;
        currentLeftPod = 0;
        currentRightPod = 0;

        int dn1 = currentLeftPod - oldLeftPod;
        int dn2 = currentRightPod - oldRightPod;
        int dn3 = currentCenterPod - oldCenterPod;

        dtheta = cm_per_tick * ((dn2-dn1) / trackwidth);
        dx = cm_per_tick * (dn1+dn2)/2.0;
        dy = cm_per_tick * (dn3 - (dn2-dn1) * centerPodOffset / trackwidth);

        double theta = heading + (dtheta / 2.0);
        X += dx * Math.cos(theta) - dy * Math.sin(theta);
        Y += dx * Math.sin(theta) + dy * Math.cos(theta);
        heading += dtheta;

        factor = heading/360;

        if(factor > 1) {
            heading = heading - 360*(int)factor;
        }

        telemetry.addData("right", currentRightPod);
        telemetry.addData("left", currentLeftPod);
        telemetry.addData("center", currentCenterPod);
        telemetry.addData("X", X);
        telemetry.addData("Y", Y);
        telemetry.addData("Heading", heading);
        telemetry.update();
        
    }

}