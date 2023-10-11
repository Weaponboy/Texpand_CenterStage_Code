package org.firstinspires.ftc.teamcode.Code_Under_Development.Auto;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.propDetectorTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetectorTest;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Sensors;
import org.firstinspires.ftc.vision.VisionPortal;

public class Sprint2_Auto extends LinearOpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(183, 305, 90);

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Sensors sensors = new Sensors();

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
        odometry.init(hardwareMap);
        drive.init(hardwareMap);
        deliverySlides.init(hardwareMap);
        sensors.init(hardwareMap);
        propDetectorTest = new PropDetectorTest();
        sensors.portal = VisionPortal.easyCreateWithDefaults(sensors.frontCam, propDetectorTest);
    }

    public void dropPurplePixel(){

    }

    public void dropYellowPixel(){

    }

}
