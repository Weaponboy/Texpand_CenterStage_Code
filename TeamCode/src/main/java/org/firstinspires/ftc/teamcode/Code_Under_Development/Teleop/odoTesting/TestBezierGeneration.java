package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop.odoTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.PathingPoint;
import org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.ObjectAvoidance.Vector2D;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
public class TestBezierGeneration extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Vector2D robotPos = new Vector2D();
    Vector2D startPoint;
    Vector2D endPoint;
    Vector2D controlPoint;

    Vector2D onTheCurve;

    ArrayList<Vector2D> path = new ArrayList<>();

    double t = 0.0; // Parameter 't' to control progress along the curve

    @Override
    public void runOpMode() throws InterruptedException {
        
        robotPos.set(0, 0);
        startPoint = new Vector2D(robotPos.getX(), robotPos.getY());
        endPoint = new Vector2D(60, 60);
        controlPoint = new Vector2D(60, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            // Calculate a point on the quadratic Bezier curve using the control points
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            // Increment 't' for the next point
            t += 0.01;

            // Display the x and y coordinates of the point
            telemetry.addData("X", onTheCurve.getX());
            telemetry.addData("Y", onTheCurve.getY());
            telemetry.update();

            // Store the point on the curve
            path.add(onTheCurve);

            // Sleep for a short duration to control the loop rate
            sleep(5000);

            // End the loop once 't' reaches 1.0
            if (t >= 1.0) {
                break;
            }
        }
    }

    // Function to calculate a point on the quadratic Bezier curve
    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D control, Vector2D end, double t) {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;

        double x = uu * start.getX() + 2 * u * t * control.getX() + tt * end.getX();
        double y = uu * start.getY() + 2 * u * t * control.getY() + tt * end.getY();

        return new Vector2D(x, y);
    }
}
