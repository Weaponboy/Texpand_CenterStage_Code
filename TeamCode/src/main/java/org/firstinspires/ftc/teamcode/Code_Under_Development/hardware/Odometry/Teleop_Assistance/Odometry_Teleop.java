package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Teleop_Assistance;

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
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.throttle;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@TeleOp
public class Odometry_Teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odo = new Odometry(0, 0, 0);

    private double horizontal;
    private double rotation;

    double denominator;

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

        odo.drivePID = new PIDFController(driveP, 0, driveD, driveF);
        odo.strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        odo.PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

    }

    public void driveCode(double Throttle){

        if (gamepad1.a){
            RunOdo = false;
        }

        if (RunOdo && gamepad1.atRest()){

            odo.update();

            odo.Xdist = (targetX - odo.X);
            odo.Ydist = (targetY - odo.Y);

            if (botHeading <= 0) {
                odo.ConvertedHeading = (360 + botHeading);
            } else {
                odo.ConvertedHeading = (0 + botHeading);
            }

            odo.rotdist = (targetHeading - odo.ConvertedHeading);

            if (odo.rotdist < -180) {
                odo.rotdist = (360 + odo.rotdist);
            } else if (odo.rotdist > 360) {
                odo.rotdist = (odo.rotdist - 360);
            }

            odo.RRXdist = odo.Ydist * Math.sin(Math.toRadians(odo.ConvertedHeading)) + odo.Xdist * Math.cos(Math.toRadians(odo.ConvertedHeading));
            odo.RRYdist = odo.Ydist * Math.cos(Math.toRadians(odo.ConvertedHeading)) - odo.Xdist * Math.sin(Math.toRadians(odo.ConvertedHeading));

            odo.Vertical = odo.drivePID.calculate(-odo.RRXdist);
            odo.Horizontal = odo.strafePID.calculate(-odo.RRYdist);
            odo.Pivot = odo.PivotPID.calculate(-odo.rotdist);

            double denominator = Math.max(Math.abs(odo.Vertical) + Math.abs(odo.Horizontal) + Math.abs(odo.Pivot), 1);

            double left_Front = (odo.Vertical + odo.Horizontal + odo.Pivot) / denominator;
            double left_Back = (odo.Vertical - odo.Horizontal + odo.Pivot) / denominator;
            double right_Front = (odo.Vertical - odo.Horizontal - odo.Pivot) / denominator;
            double right_Back = (odo.Vertical + odo.Horizontal - odo.Pivot) / denominator;

            drive.RF.setPower(right_Front);
            drive.RB.setPower(right_Back);
            drive.LF.setPower(left_Front);
            drive.LB.setPower(left_Back);

        }else{

            throttle = Throttle;

            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x * 1.2;
            rotation = gamepad1.left_stick_x;

            denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(rotation), 1);

            drive.RF.setPower(throttle*(-rotation + (vertical - horizontal)) / denominator);
            drive.RB.setPower(throttle*(-rotation + (vertical + horizontal)) / denominator);
            drive.LF.setPower(throttle*(rotation + (vertical + horizontal)) / denominator);
            drive.LB.setPower(throttle*(rotation + (vertical - horizontal)) / denominator);
        }

    }
}
