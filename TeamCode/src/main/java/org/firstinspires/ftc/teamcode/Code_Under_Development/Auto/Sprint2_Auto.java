package org.firstinspires.ftc.teamcode.Code_Under_Development.Auto;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.botHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveF;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_d;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_i;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_p;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationF;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeF;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.strafeP;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.delivery;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.deliverySlides;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.odometry;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.sensors;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.propDetecterRed;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Setpoints.Pivot_Target;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.Testopmode.X;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Odometry.Testopmode.Y;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ConvertedHeading;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.ConvertedHeadingForPosition;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Horizontal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Pivot;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.PivotPID;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.RRXdist;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.RRYdist;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.Vertical;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.drivePID;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.rotdist;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry.strafePID;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetecterByHeight;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetectorTest;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Sensors;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class Sprint2_Auto extends LinearOpMode {

    double Xdist;

    double Ydist;

    Odometry odometry = new Odometry(93, 33, 270);

    public WebcamName frontCam;

    public VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {

        initialization();

        while (opModeInInit()){
            odometry.update();
            telemetry.addData("Position", propPos);
            telemetry.addData("1", propDetecterRed.position1);
            telemetry.addData("2", propDetecterRed.position2);
            telemetry.addData("3", propDetecterRed.position3);
            telemetry.update();
        }
        
        waitForStart();

        if (propPos == 1){

            odometry.Odo_Drive(95, 85, 180);

            dropPurplePixel();

            odometry.Odo_Drive(100, 150, 180);

            odometry.Odo_Drive(310, 150, 180);

            odometry.Odo_Drive(314, 75, 180);

            dropYellowPixel();

        } else if (propPos == 2) {

            odometry.Odo_Drive(50, 58, 270);

            dropPurplePixel();

            odometry.Odo_Drive(20, 58, 180);

            sleep(500);

            odometry.Odo_Drive(20, 125, 180);

        } else if (propPos == 3) {

            odometry.Odo_Drive(75, 90, 0);

            dropPurplePixel();

            odometry.Odo_Drive(60, 130, 180);
        }
    }

    public void initialization(){

        drive = new Drivetrain();
        sensors = new Sensors();

        odometry.init(hardwareMap);
        drive.init(hardwareMap);

        frontCam = hardwareMap.get(WebcamName.class, "frontCam");

        propDetecterRed = new PropDetecterByHeight();

        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetecterRed);
    }

    public void dropPurplePixel(){

    }

    public void dropYellowPixel(){
        deliverySlides.DeliverySlides(150, 1);
        //code for depositer
    }

    public void Top_Pivot_Position_With_Feedforward(){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.6;

        double pivot_f = 0.1;

        double ticks_in_degrees = 2550 / 180.0;

        double Pivot_FF = Math.cos(Math.toRadians(Pivot_Target / ticks_in_degrees)) * pivot_f;

        double Pivot_Power = Top_Pivot_PID + Pivot_FF;

        delivery.Pivot.setPower(Pivot_Power);

    }



}
