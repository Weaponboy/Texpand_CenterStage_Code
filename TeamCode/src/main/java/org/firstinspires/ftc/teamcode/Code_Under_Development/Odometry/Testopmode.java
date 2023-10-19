package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry;

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
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Angle.Units;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.CenterStageObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.EmptyObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Vector.Vector2D;

import java.util.ArrayList;

@Config
@TeleOp
public class Testopmode extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ArrayList<Position> positionList = new ArrayList<>();

    Vector2D robotPos = new Vector2D();

    public static double X = 0;
    public static double Y = 60;

    public Position currentPos;
    public Position closestPosition;

    @Override
    public void init() {
        robotPos.set(X,Y);

        positionList.add(new Position(DistanceUnit.INCH, 12.0, 24.0, 0.0, 0));
        positionList.add(new Position(DistanceUnit.INCH, 18.0, 36.0, 0.0, 0));
        positionList.add(new Position(DistanceUnit.INCH, 24.0, 48.0, 0.0, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robotPos.set(X,Y);

        currentPos = new Position(DistanceUnit.INCH, robotPos.getX(), robotPos.getY(), 0.0, 0);

        closestPosition = findClosestPosition(currentPos, positionList);

        telemetry.addData("closest odstruction", closestPosition);
        telemetry.update();
    }

    public static Position findClosestPosition(Position currentPos, ArrayList<Position> positions) {
        Position closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Position pos : positions) {
            double distance = Math.sqrt(
                    Math.pow(currentPos.x - pos.x, 2) +
                            Math.pow(currentPos.y - pos.y, 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                closest = pos;
            }
        }

        return closest;
    }
}





