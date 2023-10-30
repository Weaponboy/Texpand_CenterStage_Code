package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.Path;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingPower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingVelocity;

public class mecanumFollower {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    PIDController XCorrective;
    PIDController YCorrective;
    PIDController headingPID;

    Path path = new Path();

    DriveAtAngle driveAtAngle = new DriveAtAngle();

    public void buildPath(){
        path.blueRightRandomization();
    }

    public PathingPower getPathingPower(Vector2D robotPos){
        PathingPower pathingPower;

        pathingPower = driveAtAngle.driveAtAngle(path.findAngle(path.getTargetVelocity(path.getClosestPositionOnPath(robotPos))));

        return pathingPower;
    }

    public PathingPower getCorrectivePower(Vector2D robotPos, double heading){

        XCorrective = new PIDController(driveP, 0, driveD);
        YCorrective = new PIDController(strafeP, 0, strafeD);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = path.getErrorToPath(robotPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        correctivePower.set(XCorrective.calculate(-robotRelativeXError), YCorrective.calculate(-robotRelativeYError));

        return correctivePower;
    }

    public Vector2D getCorrectivePosition(Vector2D robotPos){

        Vector2D error;

        error = path.getErrorToPath(robotPos);

        return error;
    }

    public double getTurnError(double targetHeading, double currentHeading){

        double turnError;

        turnError = targetHeading - currentHeading;

        return Math.abs(turnError);
    }

    public double getTurnPower(double targetHeading, double currentHeading){

        double turnPower;

        headingPID = new PIDController(rotationP, 0, rotationD);

        double error = targetHeading - currentHeading;

        turnPower = headingPID.calculate(-error);

        return turnPower;
    }

    public Vector2D getCurrentStopPosition(PathingVelocity currentVelocity){
        Vector2D stopPoint = new Vector2D();


        return null;
    }
    
}
