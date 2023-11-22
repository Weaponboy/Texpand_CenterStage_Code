package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Teleop_Assistance;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.checkXObstacles;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.checkYObstacles;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.getDriveToBackboardControlPoint;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.getDriveToCollectionControlPoint;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.ConvertedHeadingForPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Auto_Control_Points.controlPoints;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.TargetPoint;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.pathBuilder;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@TeleOp
public class TestingPathingGenForTeleop extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(93,23,270);

    pathBuilder path = new pathBuilder();

    Vector2D robotPos = new Vector2D();

    controlPoints controlPoints = new controlPoints();

    mecanumFollower follower = new mecanumFollower();

    boolean pathing = false;

    double targetHeading;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        robotPos.set(odometry.X, odometry.Y);

    }

    @Override
    public void loop() {

        odometry.update();

        robotPos.set(odometry.X, odometry.Y);

        if (gamepad1.b){

            if (getDriveToBackboardControlPoint(robotPos) != null){
                path.buildPath(TargetPoint.blueBackBoard, robotPos, getDriveToBackboardControlPoint(robotPos));
            }else {
                path.buildPath(TargetPoint.blueBackBoard, robotPos);
            }

            targetHeading = 180;

            pathing = true;

            follower.setPath(path.followablePath, path.pathingVelocity);
        }



//        if (gamepad1.a){
//
//            if (getDriveToCollectionControlPoint(robotPos) != null){
//                path.buildPath(TargetPoint.blueCollection, getDriveToCollectionControlPoint(robotPos), robotPos);
//            }else {
//                path.buildPath(TargetPoint.blueBackBoard, robotPos);
//            }
//
//            targetHeading = 90;
//
//            pathing = true;
//
//            follower.setPath(path.followablePath, path.pathingVelocity);
//        }

        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            pathing = false;
        }

        if (pathing && gamepad1.atRest()){
            follower.followPathTeleop(true, targetHeading, false, odometry, drive);
        }else {

            vertical = -gamepad1.right_stick_x;
            horizontal = -gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);

            drive.RF.setPower((-pivot + (vertical - horizontal)) / denominator);
            drive.RB.setPower((-pivot + (vertical + horizontal)) / denominator);
            drive.LF.setPower((pivot + (vertical + horizontal)) / denominator);
            drive.LB.setPower((pivot + (vertical - horizontal)) / denominator);

        }

        telemetry.addData("x", odometry.X);
        telemetry.addData("y", odometry.Y);
        telemetry.addData("heading", odometry.heading);
        telemetry.update();
    }

}
