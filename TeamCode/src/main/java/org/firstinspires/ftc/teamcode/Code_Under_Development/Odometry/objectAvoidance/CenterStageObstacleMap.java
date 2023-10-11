package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance;

import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles.Obstacle;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles.ObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Obstacles.RectangularObstacle;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class CenterStageObstacleMap implements ObstacleMap {
    private ArrayList<Obstacle> additionalObstacles;
    private final Obstacle[] obstacles;
    private double robotSize;


    public CenterStageObstacleMap(double tileSize, double robotSize) {
        this.additionalObstacles = additionalObstacles;
        this.obstacles = new Obstacle[]{
                // walls

                //red wall
                new RectangularObstacle( 310, -5, 310, 305),
//                //blue wall
//                new RectangularObstacle(Units.MILLIMETER, 3 * tileSize, -3.1 * tileSize, 3.1 * tileSize, 3.1 * tileSize),
//                //backdrop wall
//                new RectangularObstacle(Units.MILLIMETER, -3.1 * tileSize, 3 * tileSize, 3.1 * tileSize, 3.1 * tileSize),
//                //landing zone
//                new RectangularObstacle(Units.MILLIMETER, -3.1 * tileSize, -3.1 * tileSize, 3.1 * tileSize, -3 * tileSize),


                // trusses

//                // blue
//                // furthest
//                new RectangularObstacle(Units.MILLIMETER, -1 * tileSize - Units.INCH.toMillimeters(1), 3 * tileSize - Units.INCH.toMillimeters(1), Units.INCH.toMillimeters(1), 3 * tileSize),
//                // middle
//                new RectangularObstacle(Units.MILLIMETER, -1 * tileSize - Units.INCH.toMillimeters(1), 2 * tileSize - Units.INCH.toMillimeters(0.5), Units.INCH.toMillimeters(1), 2 * tileSize + Units.INCH.toMillimeters(0.5)),
//                // closest
//                new RectangularObstacle(Units.MILLIMETER, -1 * tileSize - Units.INCH.toMillimeters(1), 1 * tileSize - Units.INCH.toMillimeters(0.5), Units.INCH.toMillimeters(1), 1 * tileSize + Units.INCH.toMillimeters(0.5)),
//
//                // red
//                // closest
//                new RectangularObstacle(Units.MILLIMETER, -1 * tileSize - Units.INCH.toMillimeters(1), -1 * tileSize - Units.INCH.toMillimeters(0.5), Units.INCH.toMillimeters(1), -1 * tileSize + Units.INCH.toMillimeters(0.5)),
//                // middle
//                new RectangularObstacle(Units.MILLIMETER, -1 * tileSize - Units.INCH.toMillimeters(1), -2 * tileSize - Units.INCH.toMillimeters(0.5), Units.INCH.toMillimeters(1), -2 * tileSize + Units.INCH.toMillimeters(0.5)),
//                // furthest
//                new RectangularObstacle(Units.MILLIMETER, -1 * tileSize - Units.INCH.toMillimeters(1), -3 * tileSize, Units.INCH.toMillimeters(1), -3 * tileSize + Units.INCH.toMillimeters(1)),
//
//
//                // backdrops
//
//                // blue
//                new RectangularObstacle(Units.MILLIMETER, 2.5 * tileSize, 1 * tileSize, 3 * tileSize, 2 * tileSize),
//                // red
//                new RectangularObstacle(Units.MILLIMETER, 2.5 * tileSize, -1 * tileSize, 3 * tileSize, -2 * tileSize),

        };
    }

    @Override
    public double getRobotSize() {
        return robotSize;
    }

    @Override
    public ArrayList<Obstacle> getAdditionalObstacles() {
        return additionalObstacles;
    }

    @Override
    public Obstacle[] getObstacles() {
        return obstacles;
    }

}