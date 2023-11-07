package org.firstinspires.ftc.teamcode.Code_Under_Development.Auto;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_d;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_i;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.pivot_p;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.delivery;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.deliverySlides;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Hardware_objects.sensors;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.propDetecterRed;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Setpoints.Pivot_Target;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Code_Under_Development.VisionTesting.VisionPortalProcessers.PropDetecterByHeight;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Sensors;
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

        propDetecterRed = new PropDetecterByHeight(PropDetecterByHeight.color.red);

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
