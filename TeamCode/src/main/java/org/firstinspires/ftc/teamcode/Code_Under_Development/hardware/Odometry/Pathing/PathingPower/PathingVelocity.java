package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower;

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

}
