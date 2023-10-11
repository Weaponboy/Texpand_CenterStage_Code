package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.Teleop_Assistance;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.throttle;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;

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

        throttle = Throttle;

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x * 1.2;
        rotation = gamepad1.left_stick_x;

        if (RunOdo && gamepad1.atRest()){

            odo.update();
            double CurrentXPos = odo.X;
            double CurrentYPos = odo.Y;
            double Heading = Math.toDegrees(odo.heading);

            XdistForStop = (-targetX - CurrentXPos);
            YdistForStop = (targetY - CurrentYPos);

            if (Heading <= 0) {
                ConvertedHeading = (360 + Heading);
            } else {
                ConvertedHeading = (0 + Heading);
            }

            rotdist = (targetHeading - Heading) * 1.55;

            rotdistForStop = (targetHeading - Heading);

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

            RRXdist = Xdist * Math.cos(Math.toRadians(360 - ConvertedHeading)) - Ydist * Math.sin(Math.toRadians(360 - ConvertedHeading));
            RRYdist = Xdist * Math.sin(Math.toRadians(360 - ConvertedHeading)) + Ydist * Math.cos(Math.toRadians(360 - ConvertedHeading));

            Vertical = drivePID.calculate(RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            horizontal = Math.max(horizontal, Horizontal);
            vertical = Math.max(vertical, Vertical);
            rotation = Math.max(rotation, Pivot);
        }

        denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(rotation), 1);

        drive.RF.setPower(throttle*(-rotation + (vertical - horizontal)) / denominator);
        drive.RB.setPower(throttle*(-rotation + (vertical + horizontal)) / denominator);
        drive.LF.setPower(throttle*(rotation + (vertical + horizontal)) / denominator);
        drive.LB.setPower(throttle*(rotation + (vertical - horizontal)) / denominator);
    }
}
