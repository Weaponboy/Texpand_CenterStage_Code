package org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.Odometry_Calibration;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;

@TeleOp
public class MaxVelocity extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        odometry = new Odometry(0,0,0);

        odometry.init(hardwareMap);
        drive.init(hardwareMap);


    }


}
