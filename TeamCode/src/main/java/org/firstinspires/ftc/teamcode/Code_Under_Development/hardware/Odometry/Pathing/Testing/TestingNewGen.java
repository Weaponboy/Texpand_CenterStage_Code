package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Testing;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathGeneration.pathBuilder;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingPower;

@TeleOp
public class TestingNewGen extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    pathBuilder pathBuilder = new pathBuilder();

    Vector2D robotPos = new Vector2D();

    mecanumFollower follower;

    double Heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pathBuilder.blueRight();

        pathBuilder.pathBuilder(pathBuilder.originalPath);

        pathBuilder.motionProfile();

        follower = new mecanumFollower(pathBuilder.followablePath, pathBuilder.pathingVelocity);

        waitForStart();

        while (opModeIsActive()){

            robotPos.set(106, 138.6);

            PathingPower correctivePower;
            correctivePower = follower.getCorrectivePower(robotPos, Heading);

            Vector2D correctivePosition;
            correctivePosition = follower.getCorrectivePosition(robotPos);

            PathingPower pathingPower;
            pathingPower = follower.getPathingPower(robotPos);

            if (Math.abs(correctivePosition.getX()) > 5 || Math.abs(correctivePosition.getY()) > 5){
                vertical = correctivePower.getVertical();
                horizontal = correctivePower.getHorizontal();
            }else {
                horizontal = pathingPower.getHorizontal();
                vertical = pathingPower.getVertical();
            }

            pivot = follower.getTurnPower(0, Heading);

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

            double left_Front = (vertical + horizontal + pivot) / denominator;
            double left_Back = (vertical - horizontal + pivot) / denominator;
            double right_Front = (vertical - horizontal - pivot) / denominator;
            double right_Back = (vertical + horizontal - pivot) / denominator;

            telemetry.addData("RF", right_Front);
            telemetry.addData("LF", left_Front);
            telemetry.addData("LB", left_Back);
            telemetry.addData("RB", right_Back);
            telemetry.addData("Power X", vertical);
            telemetry.addData("Power Y", horizontal);
            telemetry.addData("turn power", pivot);
//            telemetry.addData("velocity point X", pathBuilder.pathingVelocity.get(1000).getXVelocity());
//            telemetry.addData("velocity point Y", pathBuilder.pathingVelocity.get(1000).getYVelocity());
//            telemetry.addData("point", pathBuilder.followablePath.get(1000));
            telemetry.update();
        }
    }
}
