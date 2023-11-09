package org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Auto_Control_Points;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;

public class controlPoints {

    /**blue right start to backboard*/
    public Vector2D sPFirstSeg = new Vector2D(93, 23);
    public Vector2D ePFirstSeg = new Vector2D(95, 85);

    public Vector2D sPSecondSeg = new Vector2D(95, 85);
    public Vector2D cPSecondSeg = new Vector2D(90, 160);
    public Vector2D ePSecondSeg = new Vector2D(154, 157);

    public Vector2D sPThirdSeg = new Vector2D(ePSecondSeg.getX(), ePSecondSeg.getY());
    public Vector2D cPThirdSeg = new Vector2D(302, 154);
    public static Vector2D ePThirdSeg = new Vector2D(314, 75);

    /**test curve*/
    public Vector2D sPTest = new Vector2D(0, 0);
    public Vector2D cPTest = new Vector2D(75, 0);
    public Vector2D ePTest = new Vector2D(75, 75);

    public Vector2D sTest = new Vector2D(75, 75);
    public Vector2D eTest = new Vector2D(130, 50);

}
