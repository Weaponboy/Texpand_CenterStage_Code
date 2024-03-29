package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Testing.Old;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingPower;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pathing.PathingPower.PathingVelocity;

import java.util.ArrayList;

@TeleOp
@Disabled
public class TestPathFollower extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Vector2D robotPos = new Vector2D();

    ArrayList<Vector2D> trajectory = new ArrayList<>();

    ArrayList<PathingVelocity> velcity = new ArrayList<>();

    mecanumFollower follower = new mecanumFollower();

    ElapsedTime elapsedTime = new ElapsedTime();
    public double loopTime;
    public double lastTime;
    public int counter;

    double Heading = 60;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Build Path
//        follower.buildPath();

        waitForStart();

        elapsedTime.reset();
        elapsedTime.startTime();

        while (opModeIsActive()){

            robotPos.set(180, 156);

            PathingPower correctivePower;
            correctivePower = follower.getCorrectivePowerOnPath(robotPos, Heading);

            Vector2D correctivePosition;
            correctivePosition = follower.getCorrectivePosition(robotPos);

            PathingPower pathingPower;
            pathingPower = follower.getPathingPower(robotPos, Heading);

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

            telemetry.addData("error1", follower.getPathLength());


//            telemetry.addData("RF", right_Front);
//            telemetry.addData("LF", left_Front);
//            telemetry.addData("LB", left_Back);
//            telemetry.addData("RB", right_Back);
//            telemetry.addData("Power X", vertical);
//            telemetry.addData("Power Y", horizontal);
//            telemetry.addData("turn power", pivot);
            telemetry.update();

        }

    }

}
