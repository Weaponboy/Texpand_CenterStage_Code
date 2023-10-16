package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop.CompitionTeleops.ScrimmageTeleop;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.deliverySlides;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.sensors;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.portal;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.propDetecterRed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetecterByHeight;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetectorTest;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Sensors;
import org.firstinspires.ftc.vision.VisionPortal;


public class Scrimmage_Red extends OpMode {

    @Override
    public void init() {
        sensors = new Sensors();
        drive = new Drivetrain();
        deliverySlides = new Delivery_Slides();

        drive.init(hardwareMap);
        deliverySlides.init(hardwareMap);
        sensors.init(hardwareMap);

        propDetecterRed = new PropDetecterByHeight();
        portal = VisionPortal.easyCreateWithDefaults(sensors.frontCam, propDetecterRed);
    }

    @Override
    public void loop() {
    }

}
