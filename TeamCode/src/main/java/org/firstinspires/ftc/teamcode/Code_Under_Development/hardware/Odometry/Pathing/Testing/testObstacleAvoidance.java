package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Testing;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.robotRadius;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.checkXObstacles;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.UsefulMethods.checkYObstacles;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.ObstacleMap.SetMap;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.ConvertedHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.ConvertedHeadingForPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.ObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@Config
@TeleOp
public class testObstacleAvoidance extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ElapsedTime elapsedTime = new ElapsedTime();

    Odometry odometry = new Odometry(213, 342, 90);

    Vector2D robotPos = new Vector2D();

    int counter;

    public static double X = 130;
    public static double Y = 221;

    double lastLoopTime;

    double loopTime;

    @Override
    public void init() {
        drive = new Drivetrain();

        odometry.init(hardwareMap);

        SetMap();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

//        counter++;

//        if (counter > 50){
//            counter = 0;
//            loopTime = elapsedTime.milliseconds() - lastLoopTime;
//        }

//        if (counter == 0){
//            odometry.Odo_Drive(10, 10, 0);
//        }

        lastLoopTime = elapsedTime.milliseconds();

        odometry.update();

        robotPos.set(90, 61);

        vertical = -gamepad1.right_stick_x;
        horizontal = -gamepad1.right_stick_y;
        pivot = gamepad1.left_stick_x;

        double xPower = horizontal * Math.sin(Math.toRadians(ConvertedHeadingForPosition)) + vertical * Math.cos(Math.toRadians(ConvertedHeadingForPosition));
        double yPower = horizontal * Math.cos(Math.toRadians(ConvertedHeadingForPosition)) - vertical * Math.sin(Math.toRadians(ConvertedHeadingForPosition));

        xPower = checkXObstacles(robotPos, xPower, odometry);
        yPower = checkYObstacles(robotPos, yPower, odometry);

        double denominator = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(pivot), 1);

        drive.RF.setPower((-pivot + (xPower - yPower)) / denominator);
        drive.RB.setPower((-pivot + (xPower + yPower)) / denominator);
        drive.LF.setPower((pivot + (xPower + yPower)) / denominator);
        drive.LB.setPower((pivot + (xPower - yPower)) / denominator);

        telemetry.addData("loop time", loopTime);
        telemetry.addData("x", odometry.X);
        telemetry.addData("y", odometry.Y);
        telemetry.addData("center", odometry.currentCenterPod);
        telemetry.addData("right", odometry.currentRightPod);
        telemetry.addData("left", odometry.currentLeftPod);
        telemetry.addData("heading", ConvertedHeadingForPosition);
        telemetry.addData("vertical", xPower);
        telemetry.addData("horizontal", yPower);
        telemetry.addData("robot pos", robotPos);
        telemetry.update();

    }


}





