package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.robotPos;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.pathBuilder;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.whatPath;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@TeleOp
public class TestingNewGen extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    pathBuilder pathFirst = new pathBuilder();

    pathBuilder pathSecond = new pathBuilder();

    Odometry odometry = new Odometry(93,23,270);

    Drivetrain drive = new Drivetrain();

    Vector2D robotposition = new Vector2D();

    ElapsedTime elapsedTime = new ElapsedTime();

    robotPos botFullPos = new robotPos();

    mecanumFollower follower = new mecanumFollower();

    double lastLoopTime;

    double loopTime;

    double Heading = 0;

    boolean busyPathing;

    int counter;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        //build path
        pathFirst.buildPath(whatPath.testCurve);

        follower.setPath(pathFirst.followablePath, pathFirst.pathingVelocity);

        odometry.update();

        waitForStart();

        follower.followPath(true, 180, false, odometry, drive);

        odometry.update();

        pathSecond.buildPath(whatPath.blueRight);

        follower.setPath(pathSecond.followablePath, pathSecond.pathingVelocity);

        odometry.update();

        follower.followPath(true, 180, false, odometry, drive);

        while (opModeIsActive()){

            dashboardTelemetry.addData("x opmode", odometry.X);
            dashboardTelemetry.addData("y opmode", odometry.Y);
            dashboardTelemetry.addData("heading opmode", odometry.heading);
            dashboardTelemetry.update();

        }

//        while (opModeIsActive()) {
//
//            counter++;
//
//            odometry.update();
//
//            robotpos.set(odometry.X, odometry.Y);
//
//            if (counter > 50){
//                counter = 0;
//                loopTime = elapsedTime.milliseconds() - lastLoopTime;
//            }
//
//            lastLoopTime = elapsedTime.milliseconds();
//
//            botFullPos.set(odometry.X, odometry.Y, ConvertedHeadingForPosition);
//
////            odometry.update();
////
////            Heading = Odometry.ConvertedHeadingForPosition;
////
////            robotPos.set(odometry.X, odometry.Y);
////
////            //use follower methods to get motor power
////            PathingPower correctivePower;
////            correctivePower = follower.getCorrectivePower(robotPos, Heading);
////
////            Vector2D correctivePosition;
////            correctivePosition = follower.getCorrectivePosition(robotPos);
////
////            PathingPower pathingPower;
////            pathingPower = follower.getPathingPower(robotPos);
////
////            //apply motor power in order of importance
////            if (Math.abs(correctivePosition.getX()) > 5 || Math.abs(correctivePosition.getY()) > 5 && busyPathing) {
////                vertical = correctivePower.getVertical();
////                horizontal = correctivePower.getHorizontal();
////            } else if (!busyPathing) {
////                vertical = correctivePower.getVertical();
////                horizontal = correctivePower.getHorizontal();
////            } else {
////                horizontal = pathingPower.getHorizontal();
////                vertical = pathingPower.getVertical();
////            }
////
////            if (pathingPower.getHorizontal() < 0.08 && pathingPower.getVertical() < 0.08){
////                busyPathing = false;
////            }else {
////                busyPathing = true;
////            }
////
////            pivot = follower.getTurnPower(0, Heading);
////
////            //apply motor powers
////            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);
////
////            double left_Front = (vertical + horizontal + pivot) / denominator;
////            double left_Back = (vertical - horizontal + pivot) / denominator;
////            double right_Front = (vertical - horizontal - pivot) / denominator;
////            double right_Back = (vertical + horizontal - pivot) / denominator;
////
////            drive.RF.setPower(right_Front);
////            drive.RB.setPower(right_Back);
////            drive.LF.setPower(left_Front);
////            drive.LB.setPower(left_Back);
//
//            telemetry.addData("loop time ", loopTime);
//            telemetry.addData("Power X", vertical);
//            telemetry.addData("Power Y", horizontal);
//            telemetry.addData("turn power", pivot);
//            telemetry.addData("Robot pos", robotpos);
//            telemetry.update();
//        }

    }
}
