package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Odometry_Calibration;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@TeleOp
public class MaxVelocity extends LinearOpMode {

    double currentXVelocity;
    double highestXVelocity;

    double currentYVelocity;
    double highestYVelocity;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        odometry = new Odometry(0,0,0);

        odometry.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();

        drive.setAllPower(1);

        while(currentXVelocity > highestXVelocity){
            highestXVelocity = currentXVelocity;
            currentXVelocity = odometry.getMaxVerticalVelocity();
            telemetry.addData("X velocity", highestXVelocity);
            telemetry.update();
        }

        drive.RF.setPower(1);
        drive.RB.setPower(-1);
        drive.LF.setPower(-1);
        drive.LB.setPower(1);

        while(currentYVelocity > highestYVelocity){
            highestYVelocity = currentYVelocity;
            currentYVelocity = odometry.getMaxHorizontalVelocity();
            telemetry.addData("y velocity", highestYVelocity);
            telemetry.update();
        }

        while (opModeIsActive()){
            telemetry.addData("y velocity", highestYVelocity);
            telemetry.addData("x velocity", highestXVelocity);
            telemetry.update();
        }

    }


}
