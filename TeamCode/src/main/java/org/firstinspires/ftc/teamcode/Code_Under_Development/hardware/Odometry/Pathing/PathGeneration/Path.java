package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.DriveAtAngle.getMaxVelocity;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingVelocity;

import java.util.ArrayList;
import java.util.List;

public class Path {
    static Vector2D onTheCurve;

    static ArrayList<Vector2D> path = new ArrayList<>();

    static ArrayList<Vector2D> stopPoints = new ArrayList<>();

    static List<Double> velocityX = new ArrayList<>();
    static List<Double> velocityY = new ArrayList<>();

    //got to get th correct value here once i have the velocity of the robot
    static double deltaTime = 0.1;

    static double t = 0.0;

    public void blueRightRandomization(){

        Vector2D startPoint = new Vector2D();
        Vector2D endPoint = new Vector2D();
        Vector2D controlPoint = new Vector2D();

        //drop off purple pixel
        startPoint.set(93, 33);
        endPoint.set(95, 85);
        buildPath(startPoint, endPoint, true);

        //curve to under door
        startPoint.set(endPoint.getX(), endPoint.getY());
        endPoint.set(154, 157);
        controlPoint.set(90, 160);
        buildPath(startPoint, controlPoint, endPoint);

        //curve to backboard
        startPoint.set(endPoint.getX(), endPoint.getY());
        controlPoint.set(302, 154);
        endPoint.set(314, 75);
        buildPath(startPoint, controlPoint, endPoint);

        firstDerivative();
    }

    private void buildPath(Vector2D startPoint, Vector2D endPoint){
        t = 0;

        do{
            onTheCurve = calculateLine(startPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    private void buildPath( Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint){
        t = 0;

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    private void buildPath(Vector2D startPoint, Vector2D endPoint, boolean stopPoint){
        stopPoints.add(endPoint);

        t = 0;

        do{
            onTheCurve = calculateLine(startPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    private void buildPath( Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint, boolean stopPoint){
        stopPoints.add(endPoint);
        t = 0;

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D control, Vector2D end, double t) {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;

        double x = uu * start.getX() + 2 * u * t * control.getX() + tt * end.getX();
        double y = uu * start.getY() + 2 * u * t * control.getY() + tt * end.getY();

        return new Vector2D(x, y);
    }

    private Vector2D calculateLine(Vector2D start, Vector2D end, double t) {
        double u = 1 - t;

        double x = start.getX() * u + end.getX() * t;
        double y = start.getY() * u + end.getY() * t;

        return new Vector2D(x, y);
    }

    private void firstDerivative(){
        for (int i = 0; i < path.size() - 1; i++) {
            Vector2D currentPoint = path.get(i);
            Vector2D nextPoint = path.get(i + 1);

            double deltaX = nextPoint.getX() - currentPoint.getX();
            double deltaY = nextPoint.getY() - currentPoint.getY();

            deltaTime = Math.hypot(deltaY, deltaX) / getMaxVelocity();

            double velocityXValue = deltaX / deltaTime;
            double velocityYValue = deltaY / deltaTime;

            velocityX.add(velocityXValue);
            velocityY.add(velocityYValue);
        }
    }

    public double findAngle(PathingVelocity targetVelocity){

        double magnitude = Math.sqrt(targetVelocity.getXVelocity() * targetVelocity.getXVelocity() + targetVelocity.getYVelocity() * targetVelocity.getYVelocity());

        double radians = Math.atan2(targetVelocity.getYVelocity(), targetVelocity.getXVelocity());

        double degrees = Math.toDegrees(radians);

        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    public int getClosestPositionOnPath(Vector2D robotPos) {

        int index = 0;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : path) {
            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = path.indexOf(pos);
            }
        }

        return index;
    }

    public Vector2D getErrorToPath(Vector2D robotPos) {

        int index = 0;

        Vector2D error = new Vector2D();

        Vector2D position = new Vector2D();

        double lookaheadDistance;

        double incrementDistance = 1;

        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : path) {
            double distance = Math.sqrt(
                    Math.pow(robotPos.getX() - pos.getX(), 2) +
                            Math.pow(robotPos.getY() - pos.getY(), 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                index = path.indexOf(pos);
                position.set(pos.getX(), pos.getY());
            }
        }

        lookaheadDistance = Math.abs(Math.hypot(position.getX() - robotPos.getX(), position.getY() - robotPos.getY()));

        index += (int)lookaheadDistance;

        position = path.get(index);

        error.set(position.getX() - robotPos.getX(), position.getY() - robotPos.getY());

        return error;
    }

    public PathingVelocity getTargetVelocity(int index){

        PathingVelocity targetVelocity = new PathingVelocity();

        index += 1;

        targetVelocity.set(velocityX.get(index), velocityY.get(index));

        return targetVelocity;
    }


}
