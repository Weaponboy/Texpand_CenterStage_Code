package org.firstinspires.ftc.teamcode.Working_And_Tested_Code.Vision.Portals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetecterTest;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous()
public class DetectionOpmode extends OpMode {

    PropDetecterTest propDetecterTest;
    private VisionPortal visionPortal;
    WebcamName webcamName;

    @Override
    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        propDetecterTest = new PropDetecterTest();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, propDetecterTest);
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
