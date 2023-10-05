package org.firstinspires.ftc.teamcode.Working_And_Tested_Code.Vision.Portals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetecterByHeight;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetecterTest;
import org.firstinspires.ftc.teamcode.Working_And_Tested_Code.Vision.Processers.PropDetecterByWidth;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous()
public class DetectionOpmode extends OpMode {

    PropDetecterByWidth propDetecterTest;
    PropDetecterByHeight propDetecterheight;
    private VisionPortal visionPortal;
    WebcamName webcamName;

    @Override
    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        propDetecterTest = new PropDetecterByWidth();
        propDetecterheight = new PropDetecterByHeight();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, propDetecterTest, propDetecterheight);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
   }

    @Override
    public void loop() {
    }
}
