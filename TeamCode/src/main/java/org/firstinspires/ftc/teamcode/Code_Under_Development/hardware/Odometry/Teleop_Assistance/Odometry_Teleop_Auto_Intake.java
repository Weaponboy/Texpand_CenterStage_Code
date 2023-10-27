package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Teleop_Assistance;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.botHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.ConvertedHeadingForPosition;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.Pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry.rotdist;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@Disabled
public class Odometry_Teleop_Auto_Intake extends LinearOpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(60, 0, 270);

    public static FtcDashboard dashboard = FtcDashboard.getInstance();

    public static Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        odometry.update();

        waitForStart();

        while (opModeIsActive()){
            TelemetryMap();
        }

//        odometry.Odo_Drive(0, 60, 180);

    }

    void TelemetryMap(){
        telemetry.addData("rotation", rotdist);
        telemetry.addData("Pivot power", Pivot);
        telemetry.addData("Y --", odometry.Y);
        telemetry.addData("imu heading", botHeading);
        telemetry.addData("Converted heading", ConvertedHeadingForPosition);
        telemetry.addLine();
        telemetry.addData("Odometry", "Position");
        telemetry.addData("X |", odometry.X);
        telemetry.addData("Y --", odometry.Y);
        telemetry.addData("imu heading", botHeading);
        telemetry.addData("Converted heading", ConvertedHeadingForPosition);
        telemetry.update();
    }

}
