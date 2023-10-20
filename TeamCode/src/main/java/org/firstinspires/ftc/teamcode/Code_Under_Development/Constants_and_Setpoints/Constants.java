package org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints;

public class Constants {

    /**
     * !!!!!
     * BE CAREFUL WHEN CHANGING VALUES, IT WILL AFFECT ALL OF THE CODE
     * !!!!!!
     * */

    /**Drive PID's*/

    public static double driveP = 0.1;
    public static double driveD = 0.01;
    public static double driveF = 0;

    public static double strafeP = 0.1;
    public static double strafeD = 0.005;
    public static double strafeF = 0;

    public static double rotationP = 0.05;
    public static double rotationD = 0.001;
    public static double rotationF = 0;

    /**randomization position*/
    public static int propPos = 0;

    /**Odometry constants*/
    public static double botHeading;

    public static boolean collection_on = false;

    public static boolean drop_pixel_area = false;

    /**Pivot Pid values*/
    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001, pivot_f = 0.1;

    /**Slide Pid values*/
    public static double slide_p = 0.004, slide_i = 0, slide_d = 0.0001, slide_f = 0;

    /**teleop driver constants*/
    public static double throttle = 0.6;

    public static double vertical;
    public static double horizontal;
    public static double pivot;

}
