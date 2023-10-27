package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.DriveAtAngle.getMaxVelocity;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Odometry_Calibration.MaxVelocity;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

public class PathGeneration {
    static Vector2D onTheCurve;

    static ArrayList<Vector2D> path = new ArrayList<>();

    static ArrayList<Vector2D> stopPoints = new ArrayList<>();

    static List<Double> velocityX = new ArrayList<>();
    static List<Double> velocityY = new ArrayList<>();

    //got to get th correct value here once i have the velocity of the robot
    static double deltaTime = 0.1;

    static double t = 0.0;

    public static void buildPath(Vector2D startPoint, Vector2D endPoint){

        do{
            onTheCurve = calculateLine(startPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    public static void buildPath( Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint){

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    public static void buildPath(Vector2D startPoint, Vector2D endPoint, boolean stopPoint){

        do{
            onTheCurve = calculateLine(startPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    public static void buildPath( Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint, boolean stopPoint){
        stopPoints.add(endPoint);

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            t += 0.01;

            path.add(onTheCurve);

        }while (t <= 1.0);

    }

    private static Vector2D calculateQuadraticBezier(Vector2D start, Vector2D control, Vector2D end, double t) {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;

        double x = uu * start.getX() + 2 * u * t * control.getX() + tt * end.getX();
        double y = uu * start.getY() + 2 * u * t * control.getY() + tt * end.getY();

        return new Vector2D(x, y);
    }

    private static Vector2D calculateLine(Vector2D start, Vector2D end, double t) {
        double u = 1 - t;

        double x = start.getX() * u + end.getX() * t;
        double y = start.getY() * u + end.getY() * t;

        return new Vector2D(x, y);
    }

    public static void firstDerivative(){
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
}
