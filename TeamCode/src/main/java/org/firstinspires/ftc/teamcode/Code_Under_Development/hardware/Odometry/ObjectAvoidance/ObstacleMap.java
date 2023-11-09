package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.IllegalFormatCodePointException;

public class ObstacleMap {

    public static ArrayList<Vector2D> positionList = new ArrayList<>();

    //Quad quadTree = new QuadTree(new Rectangle(0, 0, 365, 365)); // Define the bounding box of your field

    public static Vector2D findClosestPosition(Vector2D currentPos) {
        Vector2D closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Vector2D pos : positionList) {
            double distance = Math.sqrt(
                    Math.pow(currentPos.getX() - pos.getX(), 2) +
                            Math.pow(currentPos.getY() - pos.getY(), 2)
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
            positionList.add(new Vector2D(xPosition, yPosition));
            xPosition++;
            if (xPosition == xEnd){
                xPosition = xStart;
                yPosition = yEnd;
            }
        }

        xPosition = xStart;
        yPosition = yStart;

        for (int i =0; i < ((yEnd - yStart)*2); i++){
            positionList.add(new Vector2D(xPosition, yPosition));
            yPosition++;
            if (yPosition == yEnd){
                yPosition = yStart;
                xPosition = xEnd;
            }
        }

    }

}
