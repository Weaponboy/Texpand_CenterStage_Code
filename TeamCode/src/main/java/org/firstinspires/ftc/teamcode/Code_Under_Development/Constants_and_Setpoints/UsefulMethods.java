package org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.robotRadius;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.ObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;

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
    public static double checkXObstacles(Vector2D robotPos, double verticalPower){

        Position currentPos = new Position(DistanceUnit.CM, robotPos.getX(), robotPos.getY(), 0.0, 0);

        Position closestPosition = ObstacleMap.findClosestPosition(currentPos);

        if (Math.abs(closestPosition.x - (robotPos.getX() + robotRadius)) <= 5 && closestPosition.y > robotPos.getY() - robotRadius && closestPosition.y < robotPos.getY() +robotRadius){

            if (closestPosition.x > robotPos.getX() && verticalPower > 0){
                verticalPower = 0;
            } else if (closestPosition.x < robotPos.getX() && verticalPower < 0){
                verticalPower = 0;
            }

        }

        return verticalPower;

    }

    public static double checkYObstacles(Vector2D robotPos, double horizontalPower){
        Position currentPos = new Position(DistanceUnit.CM, robotPos.getX(), robotPos.getY(), 0.0, 0);

        Position closestPosition = ObstacleMap.findClosestPosition(currentPos);

        if (Math.abs(closestPosition.y - (robotPos.getY() + robotRadius)) <= 5 && closestPosition.x > robotPos.getX() - robotRadius && closestPosition.x < robotPos.getX() + robotRadius) {

            if (closestPosition.y > robotPos.getY() && horizontalPower > 0){
                horizontalPower = 0;
            } else if (closestPosition.y < robotPos.getY() && horizontalPower < 0){
                horizontalPower = 0;
            }
        }
        return horizontalPower;
    }

    /**Check loop time*/
    public static double getLoopTime(double currentTime){
        return 0;
    }
}
