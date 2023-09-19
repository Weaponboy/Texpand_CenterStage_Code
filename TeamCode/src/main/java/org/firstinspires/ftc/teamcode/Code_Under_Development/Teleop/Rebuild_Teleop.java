package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Slidey_Gripper_Pivot;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;


import java.util.List;

@TeleOp
public class Rebuild_Teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Slidey_Gripper_Pivot slidey = new Slidey_Gripper_Pivot();

    Gamepad currentGamepad1 = new Gamepad();

    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();

    PIDFController Slide_Power;

    int SlidesTarget = 0;

    double SlidePower;

    int Pivot_Target = 0;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    double throttle = 0.6;

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    @Override
    public void loop() {

        /**Drive code*/
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        throttle = 0.6;

        throttle = (gamepad1.left_trigger * 0.4) + throttle;

        if (gamepad1.right_bumper) {
            throttle = 0.3;
        }

        double vertical = -gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x*1.5;
        double pivot = gamepad1.left_stick_x;

        drive.RF.setPower(throttle*(-pivot + (vertical - horizontal)));
        drive.RB.setPower((throttle*1.15)*(-pivot + (vertical + horizontal)));
        drive.LF.setPower(throttle*(pivot + (vertical + horizontal)));
        drive.LB.setPower((throttle*1.15)*(pivot + (vertical - horizontal)));

        SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() > 2200;
        SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() < 10;

        if (gamepad1.x && !SlideSafetyHeight){
            SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() > 2200;
            deliverySlides.SlidesBothPower(1);
        }else if (gamepad1.a && !SlideSafetyBottom) {
            SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() < 10;
            deliverySlides.SlidesBothPower(-1);
        }else{
            deliverySlides.SlidesBothPower(0.0005);
        }

        if (gamepad1.dpad_up){
            Pivot_Target = 460;
        }

        if (gamepad1.dpad_down){
            Pivot_Target = 0;
        }

        if (gamepad1.right_trigger > 0){
            deliverySlides.DeliverySlides(0, -0.6);
        }
        if (gamepad1.b){
            slidey.Intake.setPower(-1);
        }else if (gamepad1.y) {
            slidey.Intake.setPower(0);
        }

//        if(gamepad1.dpad_left){
//            slidey.Plunger.setPosition(0);
//        }else if (gamepad1.dpad_right){
//            slidey.Plunger.setPosition(1);
//        } else if (gamepad1.back) {
//            slidey.Plunger.setPosition(0.6);
//        } else if (gamepad1.start) {
//            slidey.Plunger.setPosition(0.65);
//        }

        if(gamepad1.dpad_left){
            slidey.Plunger.setPosition(-1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            slidey.Plunger.setPosition(0.5);
        }else if (gamepad1.dpad_right){
            slidey.Plunger.setPosition(1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            slidey.Plunger.setPosition(0.5);
        }

        Top_Pivot_Position();

        telemetry.addData("left slide", deliverySlides.Left_Slide.getCurrentPosition());
        telemetry.addData("right slide", deliverySlides.Right_Slide.getCurrentPosition());
        telemetry.addData("height", SlideSafetyHeight);
        telemetry.addData("bottom", SlideSafetyBottom);
        telemetry.addData("pivot pos", slidey.Pivot.getCurrentPosition());
        telemetry.addData("pivot target", Pivot_Target);

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        slidey.init(hardwareMap);

        Slide_Power = new PIDFController(pivot_p, pivot_i, pivot_d, 0);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void Top_Pivot_Position(){

        slidey.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = slidey.Pivot.getCurrentPosition();

        double Top_Pivot_PID = slidey.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.2;

        slidey.Pivot.setPower(Top_Pivot_PID);

    }

    public void CheckSlidePos(){
        SlidePower = Slide_Power.calculate(deliverySlides.Left_Slide.getCurrentPosition(), SlidesTarget);
        deliverySlides.SlidesBothPower(SlidePower);
    }

    public void calculateSlidePowerUp(){
        SlidesTarget = deliverySlides.Left_Slide.getCurrentPosition() + 30;
        SlidePower = Slide_Power.calculate(SlidesTarget);
        SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() > 2200;
        deliverySlides.SlidesBothPower(SlidePower);
    }

    public void calculateSlidePowerDown(){
        SlidesTarget = deliverySlides.Left_Slide.getCurrentPosition() - 30;
        SlidePower = Slide_Power.calculate(SlidesTarget);
        SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() < 10;
        deliverySlides.SlidesBothPower(SlidePower);
    }

}
