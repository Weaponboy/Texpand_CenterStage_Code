package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.checkXObstacles;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.checkYObstacles;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.ConvertedHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.RRXdist;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.RRYdist;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;

@Config
@TeleOp
public class testObstacleAvoidance extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Vector2D robotPos = new Vector2D();



    public static double X = 130;
    public static double Y = 221;


    @Override
    public void init() {
        drive = new Drivetrain();

        robotPos.set(X,Y);

        vertical = 1;

        horizontal = 1;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x * 1.2;
        pivot = gamepad1.left_stick_x;

        vertical = horizontal * Math.sin(Math.toRadians(ConvertedHeading)) + vertical * Math.cos(Math.toRadians(ConvertedHeading));
        horizontal = horizontal * Math.cos(Math.toRadians(ConvertedHeading)) - vertical * Math.sin(Math.toRadians(ConvertedHeading));

        vertical = checkXObstacles(robotPos, vertical);
        horizontal = checkYObstacles(robotPos, horizontal);

        double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);

        drive.RF.setPower((-pivot + (RRXdist - RRYdist)) / denominator);
        drive.RB.setPower((-pivot + (RRXdist + RRYdist)) / denominator);
        drive.LF.setPower((pivot + (RRXdist + RRYdist)) / denominator);
        drive.LB.setPower((pivot + (RRXdist - RRYdist)) / denominator);

    }


}





