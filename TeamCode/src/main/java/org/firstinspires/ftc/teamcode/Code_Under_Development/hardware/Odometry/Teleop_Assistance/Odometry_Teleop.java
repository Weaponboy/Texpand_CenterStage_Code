package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Teleop_Assistance;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.throttle;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Odometry_Calibration.Tune_PID_And_Drive_Direction.heading;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@TeleOp
public class Odometry_Teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odo = new Odometry(0, 0, 0);

    private double vertical;
    private double horizontal;
    private double rotation;

    double RRXdist = 0;
    double RRYdist = 0;

    double denominator;

    double botHeading;

    public PIDFController drivePID;
    public PIDFController strafePID;
    public PIDFController PivotPID;

    public double driveP = 0.1;
    public double driveD = 0.01;
    public double driveF = 0;

    public double strafeP = 0.1;
    public double strafeD = 0.005;
    public double strafeF = 0;

    public double rotationP = 0.05;
    public double rotationD = 0.005;
    public double rotationF = 0;

    public double XdistForStop = 0;
    public double YdistForStop = 0;

    public double rotdistForStop = 0;

    public double Horizontal = 0;
    public double Vertical = 0;

    public double Pivot = 0;

    public double ConvertedHeading = 0;

    public double Xdist = 0;
    public double Ydist = 0;
    public double rotdist = 0;

    public double targetX = 0;
    public double targetY = 0;
    public double targetHeading = 0;

    public boolean RunOdo = false;

    @Override
    public void loop() {

        /**Drive code*/
        driveCode(0.6);

        if (gamepad1.x){
            RunOdo = true;
            targetX = 10;
            targetY = 10;
            targetHeading = 0;
        }

        telemetry.addData("target x", targetX);
        telemetry.addData("target y", targetY);
        telemetry.addData("target heading", targetHeading);
        telemetry.update();

    }

    @Override
    public void init() {

        drive.init(hardwareMap);
        odo.init(hardwareMap);

        drivePID = new PIDFController(driveP, 0, driveD, driveF);
        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

    }

    public void driveCode(double Throttle){

        if (gamepad1.a){
            RunOdo = false;
        }

        if (RunOdo && gamepad1.atRest()){

            Xdist = (targetX - odo.X);
            Ydist = (targetY - odo.Y);

            rotdist = (targetHeading - heading);

            RRXdist = Ydist * Math.sin(Math.toRadians(heading)) + Xdist * Math.cos(Math.toRadians(heading));
            RRYdist = Ydist * Math.cos(Math.toRadians(heading)) - Xdist * Math.sin(Math.toRadians(heading));

            Vertical = drivePID.calculate(-RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            double denominator = Math.max(Math.abs(Vertical) + Math.abs(Horizontal) + Math.abs(Pivot), 1);

            double left_Front = (Vertical + Horizontal + Pivot) / denominator;
            double left_Back = (Vertical - Horizontal + Pivot) / denominator;
            double right_Front = (Vertical - Horizontal - Pivot) / denominator;
            double right_Back = (Vertical + Horizontal - Pivot) / denominator;

            drive.RF.setPower(right_Front);
            drive.RB.setPower(right_Back);
            drive.LF.setPower(left_Front);
            drive.LB.setPower(left_Back);

        }else{

            throttle = Throttle;

            vertical = -gamepad1.right_stick_y;
            horizontal = -gamepad1.right_stick_x * 1.2;
            rotation = gamepad1.left_stick_x;

            denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(rotation), 1);

            drive.RF.setPower(throttle*(-rotation + (vertical - horizontal)) / denominator);
            drive.RB.setPower(throttle*(-rotation + (vertical + horizontal)) / denominator);
            drive.LF.setPower(throttle*(rotation + (vertical + horizontal)) / denominator);
            drive.LB.setPower(throttle*(rotation + (vertical - horizontal)) / denominator);
        }

    }
}
