package org.firstinspires.ftc.teamcode.Code_Under_Development.Auto;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_d;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_i;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_p;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.delivery;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.deliverySlides;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.odometry;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.sensors;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.propDetecterRed;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Setpoints.Pivot_Target;

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

public class Sprint2_Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        
        waitForStart();

        if (propPos == 1){

            odometry.Odo_Drive(205, 275, 270);

            dropPurplePixel();

            odometry.Odo_Drive(285, 244, 180);

            dropYellowPixel();

        } else if (propPos == 2) {

            odometry.Odo_Drive(183, 260, 270);

            dropPurplePixel();

            odometry.Odo_Drive(270, 244, 180);

            dropYellowPixel();

        } else if (propPos ==3) {

            odometry.Odo_Drive(183, 265, 315);

            dropPurplePixel();

            odometry.Odo_Drive(255, 244, 180);

            dropYellowPixel();

        }
    }

    public void initialization(){
        //create hardware objects
        drive = new Drivetrain();
        odometry = new Odometry(183, 0, 270);
        deliverySlides = new Delivery_Slides();
        sensors = new Sensors();
        delivery = new Delivery();

        //init hardware
        odometry.init(hardwareMap);
        drive.init(hardwareMap);
        deliverySlides.init(hardwareMap);
        sensors.init(hardwareMap);
        propDetecterRed = new PropDetecterByHeight();
        sensors.portal = VisionPortal.easyCreateWithDefaults(sensors.frontCam, propDetecterRed);
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
