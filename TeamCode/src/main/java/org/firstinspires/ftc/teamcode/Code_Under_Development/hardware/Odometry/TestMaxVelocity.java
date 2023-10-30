package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;

import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

@TeleOp
public class TestMaxVelocity extends LinearOpMode {

    Odometry odo = new Odometry();
    Drivetrain drive = new Drivetrain();

    double maxVecticalVelo;
    double maxHorzontalVelo;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odo.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            double vertical = -gamepad1.right_stick_y;
            double horizontal = gamepad1.right_stick_x*1.5;
            double pivot = gamepad1.left_stick_x;

            drive.RF.setPower(-pivot + (vertical - horizontal));
            drive.RB.setPower(-pivot + (vertical + horizontal));
            drive.LF.setPower(pivot + (vertical + horizontal));
            drive.LB.setPower(pivot + (vertical - horizontal));

            double differenceX = odo.getMaxVerticalVelocity() - maxVecticalVelo;

            if (differenceX > 0){
                maxVecticalVelo += differenceX;
            }

            double differenceY = odo.getMaxHorizontalVelocity() - maxHorzontalVelo;

            if (differenceY > 0){
                maxHorzontalVelo += differenceY;
            }

            telemetry.addData("x velo", maxVecticalVelo);
            telemetry.addData("y velo", maxHorzontalVelo);
            telemetry.update();
        }
    }
}
