package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Collection;

import java.util.List;

@TeleOp
public class Prototype_Teleop_Double_Deposit extends OpMode {

    Drivetrain drive = new Drivetrain();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    Delivery delivery = new Delivery();

    PIDFController Slide_Power;

    Servo LeftClaw;

    Servo RightClaw;
    Servo Intake_Servo;
    Servo Plane_Launcher;

    DistanceSensor left_Pixel;
    ColorSensor right_Pixel;

    int SlidesTarget = 0;

    double SlidePower;

    int Pivot_Target = 0;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    double throttle = 0.6;

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    boolean reverseIntake = false;

    boolean autodeposit = false;
    int RightPixelColThresh = 700;
    double IntakeServopos = 0.3;
    int buttondelaytime = 300;

    ElapsedTime runtime = new ElapsedTime();

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

        /**Delivery Slide Code*/

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

        /**Top pivot code*/

        if (gamepad1.dpad_up){
            Pivot_Target = 2070;
        }

        if (gamepad1.dpad_down){
            Pivot_Target = 0;
        }

        if (gamepad1.right_trigger > 0){
            deliverySlides.DeliverySlides(0, -0.6);
        }
        /**Pixel Sensor Conditions*/

        if (left_Pixel.getDistance(DistanceUnit.MM) < 23 && autodeposit){
            RightClaw.setPosition(0.5);                     //Lol I think I did the hardware wrong
        }

        if ((right_Pixel.blue() > RightPixelColThresh || right_Pixel.red() > RightPixelColThresh || right_Pixel.green() > RightPixelColThresh) && autodeposit){
            LeftClaw.setPosition(1);                        //Lol I think I did the hardware wrong
        }

        /**Intake Toggle*/

        if (collection.Intake.getPower() < 0 && left_Pixel.getDistance(DistanceUnit.MM) < 23 && (right_Pixel.blue() > RightPixelColThresh || right_Pixel.red() > RightPixelColThresh || right_Pixel.green() > RightPixelColThresh)){
            collection.Intake.setPower(0.4);
            reverseIntake = true;
            runtime.reset();
        }

        if (reverseIntake && (runtime.seconds() > 3)){
            reverseIntake = false;
            collection.Intake.setPower(0);
        }

        if (gamepad1.right_bumper && collection.Intake.getPower() >= 0 && (runtime.milliseconds()) > buttondelaytime){
            collection.Intake.setPower(-0.5);
            autodeposit = true;
            runtime.reset();
        }else if (gamepad1.right_bumper && collection.Intake.getPower() < 0 && (runtime.milliseconds()) > buttondelaytime) {
            collection.Intake.setPower(0);
            autodeposit = false;
            runtime.reset();
        }

        /**Deposit Code*/

        if(gamepad1.dpad_left && LeftClaw.getPosition() == 0.5){
            LeftClaw.setPosition(1);
            autodeposit = false;
        }else if (gamepad1.dpad_left && LeftClaw.getPosition() == 1){
            LeftClaw.setPosition(0.5);
            autodeposit = false;
        }

        if(gamepad1.dpad_right && RightClaw.getPosition() == 1){
            RightClaw.setPosition(0.5);
            autodeposit = false;
        }else if (gamepad1.dpad_right && RightClaw.getPosition() == 0.5){
            RightClaw.setPosition(1);
            autodeposit = false;
        }

        if(gamepad1.back){
            RightClaw.setPosition(0.5);
            LeftClaw.setPosition(1);
            autodeposit = false;
        }else if (gamepad1.start){
            RightClaw.setPosition(1);
            LeftClaw.setPosition(0.5);
            autodeposit = false;
        }

        /**Intake servo*/

        if(gamepad1.y && IntakeServopos < 0.6 && (runtime.milliseconds()) > buttondelaytime) {
            IntakeServopos = IntakeServopos + 0.05;
            runtime.reset();
        }
//        else if(gamepad1.y && IntakeServopos < 0.8 && (runtime.milliseconds()) > buttondelaytime) {
//            IntakeServopos = IntakeServopos + 0.1;
//            runtime.reset();
//        }
        else if (gamepad1.y && (runtime.milliseconds()) > buttondelaytime){
            IntakeServopos = 0.3;
            runtime.reset();
        }


        Intake_Servo.setPosition(IntakeServopos);

        /**Plane Launcher*/

        if(gamepad1.b && Plane_Launcher.getPosition() < 0.5 && (runtime.milliseconds()) > buttondelaytime) {
            Plane_Launcher.setPosition(1);
            runtime.reset();
        }else if (gamepad1.b && (runtime.milliseconds()) > buttondelaytime){
            Plane_Launcher.setPosition(0);
            runtime.reset();
        }

        /**Call all PID's and telemetry code*/

        Top_Pivot_Position_With_Feedforward();

        TelemetryMap();

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collection.init(hardwareMap);

        delivery.init(hardwareMap);

        left_Pixel = hardwareMap.get(DistanceSensor.class, "left_pixel_sensor");

        right_Pixel = hardwareMap.get(ColorSensor.class, "right_pixel_sensor");

        LeftClaw = hardwareMap.get(Servo.class, "left_claw");

        RightClaw = hardwareMap.get(Servo.class, "right_claw");

        Intake_Servo = hardwareMap.get(Servo.class, "intake_servo");

        Plane_Launcher = hardwareMap.get(Servo.class, "plane_launcher");

        Slide_Power = new PIDFController(pivot_p, pivot_i, pivot_d, 0);

        currentGamepad1 = new Gamepad();

        previousGamepad1 = new Gamepad();

        runtime.reset();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Intake_Servo.setPosition(0.7);
        Plane_Launcher.setPosition(1);
    }

    public void Top_Pivot_Position(){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.3;

        delivery.Pivot.setPower(Top_Pivot_PID);

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

    void TelemetryMap(){
        telemetry.addData("left slide", deliverySlides.Left_Slide.getCurrentPosition());
        telemetry.addData("right slide", deliverySlides.Right_Slide.getCurrentPosition());
        telemetry.addData("Left_Pixel_Sensor", left_Pixel.getDistance(DistanceUnit.MM));
        telemetry.addData("Right_Pixel_Sensor Blue", right_Pixel.blue());
        telemetry.addData("Right_Pixel_Sensor Red", right_Pixel.red());
        telemetry.addData("Right_Pixel_Sensor Green", right_Pixel.green());
        telemetry.addData("pivot pos", delivery.Pivot.getCurrentPosition());
        telemetry.addData("Intake servo pos", Intake_Servo.getPosition());
        telemetry.addData("pivot target", Pivot_Target);
        telemetry.update();
    }

}
