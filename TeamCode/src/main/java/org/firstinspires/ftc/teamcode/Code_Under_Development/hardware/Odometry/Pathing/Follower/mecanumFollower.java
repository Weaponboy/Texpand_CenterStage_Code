package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.botHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.realHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.scaleFactor;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.DriveAtAngle.getMaxVelocity;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.ConvertedHeadingForPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.robotPos;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingPower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingVelocity;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.RobotPower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

import java.util.ArrayList;

public class mecanumFollower {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Drivetrain drive = new Drivetrain();

    HardwareMap hardwareMap;

    PIDController XCorrective;
    PIDController YCorrective;
    PIDController headingPID;

    FollowPath pathfollow;

    DriveAtAngle driveAtAngle = new DriveAtAngle();

    public void setPath(ArrayList<Vector2D> trajectory, ArrayList<PathingVelocity> pathingVelocity){
        pathfollow = new FollowPath(trajectory, pathingVelocity);
    }

    public PathingPower getPathingPower(Vector2D robotPos, double heading){

        PathingPower pathingPower;
        PathingPower actualPathingPower = new PathingPower();

        double ky = 0.0234;
        double kx = 0.0154;

        PathingVelocity pathingVelocity;

        int closestPos = pathfollow.getClosestPositionOnPath(robotPos);

        pathingVelocity = pathfollow.getTargetVelocity(closestPos);

        vertical = kx * pathingVelocity.getXVelocity() * scaleFactor;
        horizontal = ky * pathingVelocity.getYVelocity();

        double xPower = horizontal * Math.sin(Math.toRadians(ConvertedHeadingForPosition)) + vertical * Math.cos(Math.toRadians(ConvertedHeadingForPosition));
        double yPower = horizontal * Math.cos(Math.toRadians(ConvertedHeadingForPosition)) - vertical * Math.sin(Math.toRadians(ConvertedHeadingForPosition));

        pathingPower = new PathingPower(xPower, yPower);

        return pathingPower;
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

    public Vector2D followPath(robotPos robotpos, HardwareMap map, boolean power, double targetHeading){

        hardwareMap = map;

        drive.init(hardwareMap);

        Odometry odometryTest = new Odometry(robotpos.getX(), robotpos.getY(), robotpos.getHeading());

        odometryTest.init(hardwareMap);

        Vector2D robotVector = new Vector2D(robotpos.getX(), robotpos.getY());

        Vector2D targetPoint = pathfollow.getPointOnFollowable(pathfollow.getLastPoint());

        Vector2D targetPointMiddle = pathfollow.getPointOnFollowable(pathfollow.getClosestPositionOnPath(robotVector));

        PathingVelocity targetvelo = pathfollow.getTargetVelocity(1);

        int lastIndex = pathfollow.getLastPoint();

        boolean busyPathing = true;

        String pathing;

        boolean busyCorrecting;

        do {

            odometryTest.update();

            robotVector.set(odometryTest.X, odometryTest.Y);

            //use follower methods to get motor power
            PathingPower correctivePower;
            correctivePower = getCorrectivePower(robotVector, odometryTest.heading);

            Vector2D correctivePosition;
            correctivePosition = getCorrectivePosition(robotVector);

            PathingPower pathingPower;
            pathingPower = getPathingPower(robotVector, odometryTest.heading);

            //apply motor power in order of importance
            if (Math.abs(correctivePosition.getX()) > 5 || Math.abs(correctivePosition.getY()) > 5 && busyPathing) {
                vertical = correctivePower.getVertical();
                horizontal = correctivePower.getHorizontal();
                pathing = "Corrective on path";
            } else if (!busyPathing) {
                vertical = correctivePower.getVertical();
                horizontal = correctivePower.getHorizontal();
                pathing = "Corrective at end";
            } else {
                horizontal = pathingPower.getHorizontal();
                vertical = pathingPower.getVertical();
                pathing = "pathing";
            }

            if (Math.abs(pathingPower.getHorizontal()) < 0.08 && Math.abs(pathingPower.getVertical()) < 0.08){
                busyPathing = false;
            }else {
                busyPathing = true;
            }

            if (Math.abs(robotVector.getX() - targetPoint.getX()) < 0.8 && Math.abs(robotVector.getY() - targetPoint.getY()) < 0.8  && !busyPathing){
                busyCorrecting = false;
            }else {
                busyCorrecting = true;
            }

//            horizontal = pathingPower.getHorizontal();
//            vertical = pathingPower.getVertical();

            pivot = getTurnPower(targetHeading, odometryTest.heading);

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

            double left_Front = (vertical + horizontal + pivot) / denominator;
            double left_Back = (vertical - horizontal + pivot) / denominator;
            double right_Front = (vertical - horizontal - pivot) / denominator;
            double right_Back = (vertical + horizontal - pivot) / denominator;

            if (power){
                drive.RF.setPower(right_Front);
                drive.RB.setPower(right_Back);
                drive.LF.setPower(left_Front);
                drive.LB.setPower(left_Back);
            }

            dashboardTelemetry.addData("heading", odometryTest.heading);
            dashboardTelemetry.addData("target pos", targetPoint);
            dashboardTelemetry.addData("target pos middle", targetPointMiddle);
            dashboardTelemetry.addData("robotPos", robotVector);

            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("target velo x", targetvelo.getXVelocity());
            dashboardTelemetry.addData("target velo y", targetvelo.getYVelocity());

            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("vertical", vertical);
            dashboardTelemetry.addData("horizontal", horizontal);
            dashboardTelemetry.addData("power", pathing);
            dashboardTelemetry.update();


        }while(busyCorrecting);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        return robotVector;

    }

    public double getTurnError(double targetHeading, double currentHeading){

        double turnError;

        turnError = targetHeading - currentHeading;

        return Math.abs(turnError);
    }

    public double getTurnPower(double targetHeading, double currentHeading){

        double turnPower;

        double rotdist = (targetHeading - currentHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 360) {
            rotdist = (rotdist - 360);
        }

//        rotdist = Math.toRadians(rotdist);

        headingPID = new PIDController(rotationP, 0, rotationD);

        turnPower = headingPID.calculate(-rotdist);

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
