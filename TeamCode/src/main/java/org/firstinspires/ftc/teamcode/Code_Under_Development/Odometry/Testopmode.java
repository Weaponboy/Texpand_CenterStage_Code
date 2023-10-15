package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.CenterStageObstacleMap;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.objectAvoidance.Vector.Vector2D;

@Config
@Disabled
public class Testopmode extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    CenterStageObstacleMap map = new CenterStageObstacleMap( 610, 0);

    Vector2D robotPos = new Vector2D();

    public static double X = 0;
    public static double Y = 60;

    @Override
    public void init() {
        robotPos.set(X,Y);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robotPos.set(X,Y);

        telemetry.addData("closest odstruction", map.obstacleAvoidanceVector(robotPos));
        telemetry.update();
    }

}
