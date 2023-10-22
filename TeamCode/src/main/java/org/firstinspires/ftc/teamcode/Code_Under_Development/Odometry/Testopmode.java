package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.ObjectAvoidance.ObstacleMap.findClosestPosition;

import android.icu.text.Transliterator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.ObjectAvoidance.Vector2D;


import java.util.ArrayList;

@Config
@TeleOp
public class Testopmode extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Vector2D robotPos = new Vector2D();

    public static double X = 200;
    public static double Y = 4;

    public Position currentPos;
    public Position closestPosition;

    @Override
    public void init() {
        robotPos.set(X,Y);

        vertical = -1;

        horizontal = 1;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        currentPos = new Position(DistanceUnit.CM, robotPos.getX(), robotPos.getY(), 0.0, 0);

        closestPosition = findClosestPosition(currentPos);

        if (Math.abs(closestPosition.x - robotPos.getX()) <= 5 && closestPosition.y == robotPos.getY()){

            if (closestPosition.x > robotPos.getX() && vertical > 0){
                vertical = 0;
            } else if (closestPosition.x < robotPos.getX() && vertical < 0){
                vertical = 0;
            }

        }else if (Math.abs(closestPosition.y - robotPos.getY()) <= 5 && closestPosition.x == robotPos.getX()) {

            if (closestPosition.y > robotPos.getY() && horizontal < 0){
                horizontal = 0;
            } else if (closestPosition.y < robotPos.getY() && horizontal > 0){
                horizontal = 0;
            }

        }

        telemetry.addData("closest obstruction", closestPosition);
        telemetry.addData("vertical", vertical);
        telemetry.addData("horizontal", horizontal);
        telemetry.update();
    }


}





