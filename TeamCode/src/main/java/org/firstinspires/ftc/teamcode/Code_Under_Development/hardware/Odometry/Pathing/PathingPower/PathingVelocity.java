package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.jetbrains.annotations.NotNull;

public class PathingVelocity {

    private double xVelocity;
    private double yVelocity;

    public PathingVelocity(double xVelocity, double yVelocity){
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
    }

    public PathingVelocity() {
        this(0, 0);
    }

    public PathingVelocity set(double xVelocity, double yVelocity) {
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        return this;
    }

    public double getXVelocity(){
        return xVelocity;
    }

    public double getYVelocity(){
        return yVelocity;
    }

    public PathingVelocity add(@NotNull PathingVelocity other) {
        return this.add(other.xVelocity, other.yVelocity);
    }

    private PathingVelocity add(double xVelocity, double yVelocity) {
        return new PathingVelocity(this.xVelocity + xVelocity, this.yVelocity + yVelocity);
    }

}
