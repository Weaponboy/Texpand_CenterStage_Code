package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.botHeading;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Odometry {

    DcMotor LF;
    DcMotor RF;
    DcMotor LB;
    DcMotor RB;

    DcMotor leftPod;
    DcMotor rightPod;
    DcMotor centerPod;

    HardwareMap hardwareMap;

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

    public Odometry(double startX, double startY, double startHeading){
        this.startX = Math.toRadians(startX);
        this.startY = startY;
        this.startHeading = startHeading;
    }

    public double X = startX, Y = startY, heading = startHeading;

    public double dtheta;

    public double dx;
    public double dy;

    public double factor = 0;

    public BNO055IMU imu = null;

    Orientation YawAngle;

    public void update(){

        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        botHeading = -YawAngle.firstAngle;

        oldCenterPod = currentCenterPod;
        oldLeftPod = currentLeftPod;
        oldRightPod = currentRightPod;

        currentCenterPod = centerPod.getCurrentPosition();
        currentLeftPod = -leftPod.getCurrentPosition();
        currentRightPod = rightPod.getCurrentPosition();

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

    }

    public void init(HardwareMap hardwareMap2){

       hardwareMap = hardwareMap2;

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPod = LB;
        rightPod = RF;
        centerPod = LF;
    }

    public void resetHeadingUsingImu(){

        update();

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        botHeading = -YawAngle.firstAngle;

        heading = Math.toRadians(botHeading);

    }

    public void updateIMUHeading(){
        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        botHeading = -YawAngle.firstAngle;
    }

}
