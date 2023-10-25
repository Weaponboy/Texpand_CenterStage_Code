package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.IllegalFormatCodePointException;

public class ObstacleMap {

    public static ArrayList<Position> positionList = new ArrayList<>();

    public static Position findClosestPosition(Position currentPos) {
        SetMap();
        Position closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Position pos : positionList) {
            double distance = Math.sqrt(
                    Math.pow(currentPos.x - pos.x, 2) +
                            Math.pow(currentPos.y - pos.y, 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                closest = pos;
            }
        }

        return closest;
    }

    public static void SetMap(){

        // wall blue truss
        Rectangle(115, 0, 188, 5);

        // middle blue truss
        Rectangle(115, 57, 188, 63);

        // door blue truss
        Rectangle(115, 117, 188, 123);

        // door red truss
        Rectangle(115, 241, 188, 247);

        // middle red truss
        Rectangle(115, 301, 188, 307);

        // wall red truss
        Rectangle(115, 360, 188, 365);

    }

    public static void Rectangle(double xStart, double yStart, double xEnd, double yEnd){

        double xPosition = xStart;
        double yPosition = yStart;

        for (int i =0; i < ((xEnd - xStart)*2); i++){
            positionList.add(new Position(DistanceUnit.CM, xPosition, yPosition, 0.0, 0));
            xPosition++;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            positionList.add(new Position(DistanceUnit.CM, xPosition, yPosition, 0.0, 0));
            yPosition++;
            if (yPosition == yEnd){
                yPosition = yStart;
                xPosition = xEnd;
            }
        }

    }

}
