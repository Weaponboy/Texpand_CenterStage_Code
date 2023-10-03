package org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetecterTest;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled
public class DetectionOpmodeTest extends OpMode {

//   private GreenPixelDetecter greenPixel;
//   YellowPixelDetecter yellowPixel;
    PropDetecterTest propDetecterTest;
    private VisionPortal visionPortal;
    WebcamName webcamName;

   @Override
   public void init() {
       webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//       greenPixel = new GreenPixelDetecter();
//       yellowPixel = new YellowPixelDetecter();
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
