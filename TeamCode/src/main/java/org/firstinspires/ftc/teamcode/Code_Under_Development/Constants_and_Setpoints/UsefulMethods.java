package org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.robotRadius;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.ObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

public class UsefulMethods {

    /**Need to add the code to this when we have tested Peter's code*/
    public static boolean isYellow(){
        return false;
    }

    public static boolean isPurple(){
        return false;
    }

    public static boolean isGreen(){
        return false;
    }

    public static boolean isWhite(){
        return false;
    }

    public static String checkPixelColor(){
        return null;
    }

    /**Object avoidance*/
    public static double checkXObstacles(Vector2D robotPos, double verticalPower, Odometry odometry){

        Vector2D closestPosition = ObstacleMap.findClosestPosition(robotPos);

        if (closestPosition.getX() > robotPos.getX() && verticalPower > 0){
            if (closestPosition.getX() - (robotPos.getX() + robotRadius) <= 5 && closestPosition.getY() > robotPos.getY() - robotRadius && closestPosition.getY() < robotPos.getY() + robotRadius){
                verticalPower = 0;
            }else if (closestPosition.getX() - (robotPos.getX() + robotRadius) >= 5 && closestPosition.getX() - (robotPos.getX() + robotRadius) <= 10 && odometry.getVerticalVelocity() > 10) {
                verticalPower = 0;
            }
        }

        if (closestPosition.getX() < robotPos.getX() && verticalPower < 0){
            if ((robotPos.getX() - robotRadius) - closestPosition.getX() <= 5 && closestPosition.getY() > robotPos.getY() - robotRadius && closestPosition.getY() < robotPos.getY() + robotRadius){
                verticalPower = 0;
            }else if ((robotPos.getX() - robotRadius) - closestPosition.getX() >= 5 && (robotPos.getX() - robotRadius) - closestPosition.getX() <= 10 && odometry.getVerticalVelocity() < -10) {
                verticalPower = 0;
            }
        }

        return verticalPower;

    }

    public static double checkYObstacles(Vector2D robotPos, double horizontalPower, Odometry odometry){

        Vector2D closestPosition = ObstacleMap.findClosestPosition(robotPos);

        if (closestPosition.getY() > robotPos.getY() && horizontalPower > 0){
            if (closestPosition.getY() - (robotPos.getY() + robotRadius) <= 5 && closestPosition.getX() > robotPos.getX() - robotRadius && closestPosition.getX() < robotPos.getX() + robotRadius) {
                horizontalPower = 0;
            }else if (closestPosition.getY() - (robotPos.getY() + robotRadius) >= 5 && closestPosition.getY() - (robotPos.getY() + robotRadius) <= 10 && odometry.getHorizontalVelocity() > 10){
                horizontalPower = 0;
            }
        }

        if (closestPosition.getY() < robotPos.getY() && horizontalPower < 0){
            if ((robotPos.getY() - robotRadius) - closestPosition.getY() <= 5 && closestPosition.getX() > robotPos.getX() - robotRadius && closestPosition.getX() < robotPos.getX() + robotRadius) {
                horizontalPower = 0;
            }else if ((robotPos.getY() - robotRadius) - closestPosition.getY() >= 5 && (robotPos.getY() - robotRadius) - closestPosition.getY() <= 10 && odometry.getHorizontalVelocity() < -10){
                horizontalPower = 0;
            }
        }

        return horizontalPower;
    }

    /**Check loop time*/
    public static double getLoopTime(double currentTime){
        return 0;
    }

    /**find control points for pathing in teleop*/
    public static Vector2D driveToBackboardPath(Vector2D currentPosition){

        Vector2D controlPoint;

        if (currentPosition.getY() < 95 && currentPosition.getX() < 95){
            controlPoint = new Vector2D(61, 183);
        } else if (currentPosition.getY() > 221 && currentPosition.getX() < 95) {
            controlPoint = new Vector2D(61, 183);
        }else{
            controlPoint = null;
        }

        return controlPoint;

    }
}
