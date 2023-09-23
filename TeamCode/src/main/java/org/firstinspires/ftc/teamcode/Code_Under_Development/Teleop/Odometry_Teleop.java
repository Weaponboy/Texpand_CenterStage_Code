package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.botHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.collection_on;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.drop_pixel_area;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.throttle;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.X;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Y;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.heading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Intake_And_Pivot;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;

import java.util.List;

public class Odometry_Teleop extends OpMode{

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry();

    Intake_And_Pivot collect = new Intake_And_Pivot();

    @Override
    public void loop() {

        /**Drive code*/

        collection_on = X < 75 && Y < 120;
        drop_pixel_area = X > 274 && Y > 213;

        if(collection_on){
            throttle = 0.4;
        }else if (drop_pixel_area){
            throttle = 0.2;
        }else {
            throttle = 0.6;
        }

        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x*1.5;
        pivot = gamepad1.left_stick_x;

        drive.RF.setPower(throttle*(-pivot + (vertical - horizontal)));
        drive.RB.setPower((throttle*1.15)*(-pivot + (vertical + horizontal)));
        drive.LF.setPower(throttle*(pivot + (vertical + horizontal)));
        drive.LB.setPower((throttle*1.15)*(pivot + (vertical - horizontal)));

        /**Intake Toggle*/

        if (gamepad1.b || collection_on){
            collect.Intake.setPower(-0.4);
        }else if (gamepad1.y) {
            collect.Intake.setPower(0);
        }

        TelemetryMap();

        odometry.update();

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        collect.init(hardwareMap);

        currentGamepad1 = new Gamepad();

        previousGamepad1 = new Gamepad();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    void TelemetryMap(){
        telemetry.addData("Pod", "encoder readings");
        telemetry.addData("left pod", odometry.currentLeftPod);
        telemetry.addData("right pod", odometry.currentRightPod);
        telemetry.addData("center pod", odometry.currentCenterPod);
        telemetry.addLine();
        telemetry.addData("Odometry", "Position");
        telemetry.addData("X |", X);
        telemetry.addData("Y --", Y);
        telemetry.addData("odo heading", heading);
        telemetry.addData("imu heading", botHeading);
        telemetry.update();
    }

}
