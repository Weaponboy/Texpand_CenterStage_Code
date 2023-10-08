package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.botHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveF;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationF;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeF;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeP;

import com.arcrobotics.ftclib.controller.PIDFController;
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

    Drivetrain drivetrain = new Drivetrain();

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

        drivetrain.init(hardwareMap);

        drivePID = new PIDFController(driveP, 0, driveD, driveF);

        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);

        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

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

    public void Odo_Drive(double targetX, double targetY, double targetRot) {

        do {

            update();

            //GET CURRENT X
            double CurrentXPos = X;

            //GET CURRENT Y
            double CurrentYPos = Y;

            //GET START HEADING WITH ODOMETRY
            double Heading = Math.toDegrees(heading);

            //SET DISTANCE TO TRAVEL ERROR
            Xdist = (-targetX - CurrentXPos);
            Ydist = (targetY - CurrentYPos);

            XdistForStop = (-targetX - CurrentXPos);
            YdistForStop = (targetY - CurrentYPos);

            //CONVERT HEADING FOR TRIG CALCS
            if (Heading <= 0) {
                ConvertedHeading = (360 + Heading);
            } else {
                ConvertedHeading = (0 + Heading);
            }

            rotdist = (targetRot - Heading) * 1.55;

            rotdistForStop = (targetRot - Heading);

            if (rotdist < -180) {
                rotdist = (360 + rotdist);
            } else if (rotdist > 180) {
                rotdist = (rotdist - 360);
            }

            if (rotdistForStop < -180) {
                rotdistForStop = (360 + rotdistForStop);
            } else if (rotdistForStop > 180) {
                rotdistForStop = (rotdistForStop - 360);
            }

            //CONVERT TARGET TO ROBOT RELATIVE TARGET
            RRXdist = Xdist * Math.cos(Math.toRadians(360 - ConvertedHeading)) - Ydist * Math.sin(Math.toRadians(360 - ConvertedHeading));
            RRYdist = Xdist * Math.sin(Math.toRadians(360 - ConvertedHeading)) + Ydist * Math.cos(Math.toRadians(360 - ConvertedHeading));

            //SET DRIVE CONSTANTS TO THE PIDF CONTROL LOOPS
            Vertical = drivePID.calculate(RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            //SET MOTOR POWER USING THE PID OUTPUT
            drivetrain.RF.setPower(-Pivot + (Vertical + Horizontal));
            drivetrain.RB.setPower((-Pivot * 1.4) + (Vertical - (Horizontal * 1.3)));
            drivetrain.LF.setPower(Pivot + (Vertical - Horizontal));
            drivetrain.LB.setPower((Pivot * 1.4) + (Vertical + (Horizontal * 1.3)));

        }while ((Math.abs(XdistForStop) > 0.8 ) || (Math.abs(YdistForStop) > 0.8 ) || (Math.abs(rotdistForStop) > 0.8));

        drivetrain.RF.setPower(0);
        drivetrain.RB.setPower(0);
        drivetrain.LF.setPower(0);
        drivetrain.LB.setPower(0);

    }


}
