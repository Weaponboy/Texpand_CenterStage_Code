package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Teleop_Assistance;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.driveToBackboardPath;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Auto_Control_Points.controlPoints;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.FollowPath;
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

    Odometry odometry = new Odometry();

    pathBuilder path = new pathBuilder();

    Vector2D robotPos = new Vector2D();

    controlPoints controlPoints = new controlPoints();

    mecanumFollower follower = new mecanumFollower();

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        drive.init(hardwareMap);
//
//        odometry.init(hardwareMap);

        robotPos.set(61, 75);

        if (driveToBackboardPath(robotPos) != null){
            path.buildPath(TargetPoint.blueBackBoard, driveToBackboardPath(robotPos), robotPos);
        }else {
            path.buildPath(TargetPoint.blueBackBoard, robotPos);
        }

        follower.setPath(path.followablePath, path.pathingVelocity);

    }

    @Override
    public void loop() {
        telemetry.addData("length", follower.getPathLength());
        telemetry.addData("lastPoint", driveToBackboardPath(robotPos));
        telemetry.update();
    }

}
