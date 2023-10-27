package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingPower;

public class DriveAtAngle{

    //in cm's per sec, these need to be tuned
    private static final double maxYVelocity = 5;
    private final double maxXVelocity = 7;

    private final double scaleFactor = maxYVelocity/maxXVelocity;

    private double horizontal;
    private double vertical;

    PathingPower pathingPower = new PathingPower();

    public PathingPower driveAtAngle(double targetDriveDirection){

        if (targetDriveDirection >= 0 && targetDriveDirection <= 90){
            double normalized_heading = (targetDriveDirection - 0) / (90);
            vertical = (1 + (-1) * normalized_heading);
            horizontal = 0 + normalized_heading * (1);
        }else if (targetDriveDirection >= 90 && targetDriveDirection <= 180){
            double normalized_heading = (targetDriveDirection - 90) / (180 - 90);
            horizontal = (1 + (-1) * normalized_heading);
            vertical = (0 + normalized_heading * (-1));
        }else if (targetDriveDirection >= 180 && targetDriveDirection <= 270){
            double normalized_heading = (targetDriveDirection - 180) / (270 - 180);
            horizontal = (0 + (-1) * normalized_heading);
            vertical = (-1 + normalized_heading * (1));
        } else if (targetDriveDirection >= 270 && targetDriveDirection <= 360) {
            double normalized_heading = (targetDriveDirection - 270) / (360 - 270);
            horizontal = (-1 + normalized_heading * (1));
            vertical = (0 + (1) * normalized_heading);
        }

        if (targetDriveDirection != 0 && targetDriveDirection != 360 && targetDriveDirection != 180){
            vertical = vertical * scaleFactor;
        }

        pathingPower.set(vertical, horizontal);

        return pathingPower;
    }

    public static double getMaxVelocity(){
        return maxYVelocity;
    }

}
