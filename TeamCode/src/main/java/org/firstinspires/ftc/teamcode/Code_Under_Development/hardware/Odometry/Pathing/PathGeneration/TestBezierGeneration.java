package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.PathGeneration.buildPath;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.PathGeneration.firstDerivative;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.PathGeneration.path;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.PathGeneration.velocityX;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.PathGeneration.velocityY;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.ConvertedHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.Pivot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.DriveAtAngle;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingPower;

import java.util.ArrayList;

@TeleOp
public class TestBezierGeneration extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Vector2D startPoint;
    Vector2D endPoint;
    Vector2D controlPoint;

    PathingPower pathingPower = new PathingPower();

    DriveAtAngle driveAtAngle = new DriveAtAngle();

    @Override
    public void runOpMode() throws InterruptedException {
        startPoint = new Vector2D();
        controlPoint = new Vector2D();
        endPoint = new Vector2D();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

//        //drop off purple pixel
//        startPoint.set(93, 33);
//        endPoint.set(95, 85);
//        buildPath(startPoint, endPoint, true);
//
//        //curve to under door
//        startPoint.set(endPoint.getX(), endPoint.getY());
//        endPoint.set(154, 157);
//        controlPoint.set(90, 160);
//        buildPath(startPoint, controlPoint, endPoint);
//
//        //curve to backboard
//        startPoint.set(endPoint.getX(), endPoint.getY());
//        controlPoint.set(302, 154);
//        endPoint.set(314, 75);
//        buildPath(startPoint, controlPoint, endPoint);
//
//        firstDerivative();
//
//        int i = 0;
//
//        while (opModeIsActive()){
//            telemetry.addData("velocity x", velocityX.get(i));
//            telemetry.addData("velocity y", velocityY.get(i));
//            telemetry.addData("point on path", path.get(i));
//            telemetry.update();
//            i += 3;
//            sleep(1000);
//        }

        while (opModeIsActive()){
            pathingPower = driveAtAngle.driveAtAngle(45);

            vertical = pathingPower.getVertical();
            horizontal = pathingPower.getHorizontal();

            ConvertedHeading = 0;

            vertical = horizontal * Math.sin(Math.toRadians(ConvertedHeading)) + vertical * Math.cos(Math.toRadians(ConvertedHeading));
            horizontal = horizontal * Math.cos(Math.toRadians(ConvertedHeading)) - vertical * Math.sin(Math.toRadians(ConvertedHeading));
            Pivot = 0;

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(Pivot), 1);

            double left_Front = (vertical + horizontal + Pivot) / denominator;
            double left_Back = (vertical - horizontal + Pivot) / denominator;
            double right_Front = (vertical - horizontal - Pivot) / denominator;
            double right_Back = (vertical + horizontal - Pivot) / denominator;

            telemetry.addData("left_Front", left_Front);
            telemetry.addData("left_Back", left_Back);
            telemetry.addData("right_Front", right_Front);
            telemetry.addData("right_Back", right_Back);
            telemetry.addLine();
            telemetry.addData("y before", pathingPower.getHorizontal());
            telemetry.addData("x before", pathingPower.getVertical());
            telemetry.addLine();
            telemetry.addData("y after", horizontal);
            telemetry.addData("x after", vertical);
            telemetry.update();
        }

    }

}
