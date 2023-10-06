package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetectorTest;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class DetectionOpmodeTest extends OpMode {

//   private GreenPixelDetecter greenPixel;
//   YellowPixelDetecter yellowPixel;
    PropDetectorTest propDetectorTest;
    private VisionPortal visionPortal;
    WebcamName webcamName;

   @Override
   public void init() {
       webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//       greenPixel = new GreenPixelDetecter();
//       yellowPixel = new YellowPixelDetecter();
        propDetectorTest = new PropDetectorTest();
       visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, propDetectorTest);
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
