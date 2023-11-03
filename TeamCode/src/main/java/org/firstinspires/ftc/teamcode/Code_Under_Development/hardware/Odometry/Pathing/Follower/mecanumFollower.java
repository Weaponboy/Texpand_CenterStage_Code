package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.DriveAtAngle.getMaxVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingPower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingVelocity;

import java.util.ArrayList;

public class mecanumFollower {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    PIDController XCorrective;
    PIDController YCorrective;
    PIDController headingPID;

    FollowPath pathfollow;

    DriveAtAngle driveAtAngle = new DriveAtAngle();

    public mecanumFollower(ArrayList<Vector2D> trajectory, ArrayList<PathingVelocity> pathingVelocity){
        pathfollow = new FollowPath(trajectory, pathingVelocity);
    }

    public PathingPower getPathingPower(Vector2D robotPos){
        PathingPower pathingPower;
        PathingPower actualPathingPower = new PathingPower();

        PathingVelocity pathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        pathingVelocity = pathfollow.getTargetVelocity(closestPos);
        
        double XPowerFactor = Math.abs(pathingVelocity.getXVelocity() / getMaxVelocity());
        double YPowerFactor = Math.abs(pathingVelocity.getYVelocity() / getMaxVelocity());

        double travelAngle = pathfollow.findAngle(pathingVelocity);

        pathingPower = driveAtAngle.driveAtAngle(travelAngle);
        
        actualPathingPower.set(pathingPower.getVertical()*XPowerFactor, pathingPower.getHorizontal()*YPowerFactor);

        return actualPathingPower;
    }

    public PathingPower getCorrectivePower(Vector2D robotPos, double heading){

        XCorrective = new PIDController(driveP, 0, driveD);
        YCorrective = new PIDController(strafeP, 0, strafeD);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = pathfollow.getErrorToPath(robotPos);

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        correctivePower.set(XCorrective.calculate(-robotRelativeXError), YCorrective.calculate(-robotRelativeYError));

        return correctivePower;
    }

    public Vector2D getCorrectivePosition(Vector2D robotPos){

        Vector2D error;

        error = pathfollow.getErrorToPath(robotPos);

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

    //still need to finish this
    public Vector2D getCurrentStopPosition(PathingVelocity currentVelocity){
        Vector2D stopPoint = new Vector2D();

        return null;
    }

    public double getErrorBetweenPoints(int index){
        Vector2D firstPoint;
        Vector2D secondPoint;

        firstPoint = new Vector2D(pathfollow.getPointOnFollowable(index).getX(), pathfollow.getPointOnFollowable(index).getY());
        index++;
        secondPoint = new Vector2D(pathfollow.getPointOnFollowable(index).getX(), pathfollow.getPointOnFollowable(index).getY());

        return Math.hypot(firstPoint.getX() - secondPoint.getX(), firstPoint.getY() - secondPoint.getY());
    }

    public Vector2D getPointOnPath(int index){
        return pathfollow.getPointOnFollowable(index);
    }

    public double getPathLength(){
        return this.pathfollow.calculateTotalDistance();
    }

    public double calculateDistance(Vector2D point1, Vector2D point2) {
        double dx = point2.getX() - point1.getX();
        double dy = point2.getY() - point1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    
}
