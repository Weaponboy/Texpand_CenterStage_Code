package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop.Sprint_Teleops.SprintTwo;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;
import static org.opencv.core.Core.inRange;

import android.widget.Switch;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Collection;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Odometry;

import java.util.List;

@TeleOp
public class Sprint_2_teleop_new extends OpMode {

    Drivetrain drive = new Drivetrain();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    Delivery delivery = new Delivery();

    Odometry odometry = new Odometry(0,0,0);

    PIDFController Slide_Power;

    Servo DepositServoRotate;
    Servo clawRight;
    Servo clawLeft;
    Servo DepositPivot;
    Servo TestFeetech;
    Servo IntakeServo;

    int SlidesTarget = 0;

    double SlidePower;

    int Pivot_Target = 0;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    double throttle = 0.6;

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    boolean reverseIntake = false;
    double SlideMinDeposit = -900;
    boolean autodeposit = false;
    int RightPixelColThresh = 700;
    double IntakeServopos = 0.3;
    int buttondelaytime = 300;

    ElapsedTime runtime = new ElapsedTime();

    boolean pivotDelivery = false;
    boolean pivotCollection = false;
    boolean pivotMoving = false;

    boolean pivotSafePoint = false;
    boolean slidesCollect = false;
    boolean slidesMoving = false;

    boolean retracting = false;
    boolean extending = false;

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
        double horizontal = gamepad1.right_stick_x * 1.5;
        double pivot = gamepad1.left_stick_x;

        drive.RF.setPower(throttle * (-pivot + (vertical - horizontal)));
        drive.RB.setPower((throttle * 1.15) * (-pivot + (vertical + horizontal)));
        drive.LF.setPower(throttle * (pivot + (vertical + horizontal)));
        drive.LB.setPower((throttle * 1.15) * (pivot + (vertical - horizontal)));

        /**Delivery Slide Code*/

        SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() < -2200;
        SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() > -5;


        if (gamepad1.x && !SlideSafetyHeight) {
            SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() < -2200;
            deliverySlides.SlidesBothPower(-0.3);
        }else if (gamepad1.a && !SlideSafetyBottom) {
            SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() > -5;
            deliverySlides.SlidesBothPower(0.3);
        }else if (!slidesMoving){
            deliverySlides.SlidesBothPower(0.0005);
        }

        if (Math.abs(deliverySlides.Left_Slide.getVelocity()) < 20){
            slidesMoving = false;
        }

        if (gamepad1.start){
            clawLeft.setPosition(0.6);
            clawRight.setPosition(0.4);
        }

        if (gamepad1.back){
            clawLeft.setPosition(0);
            clawRight.setPosition(1);
        }

        if (gamepad1.dpad_right && IntakeServo.getPosition() == 0){
            IntakeServo.setPosition(0.5);
        }else if (gamepad1.dpad_right && IntakeServo.getPosition() == 0.5){
            IntakeServo.setPosition(0);
        }

        if(gamepad1.dpad_up){

            clawLeft.setPosition(0);
            clawRight.setPosition(1);

            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Pivot_Target = 220;

            while (delivery.Pivot.getCurrentPosition() > 210){
                Top_Pivot_Position();
            }

            DepositServoRotate.setPosition(1);

        }

        if (gamepad1.dpad_down){
            Pivot_Target = -820;

            while (delivery.Pivot.getCurrentPosition() > -800){
                Top_Pivot_Position();
            }

            DepositPivot.setPosition(0.6);

            DepositServoRotate.setPosition(0.5);
        }

        if(gamepad1.right_trigger > 0){

            clawLeft.setPosition(0);
            clawRight.setPosition(1);

            DepositServoRotate.setPosition(1);

            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Pivot_Target = 140;

            while (delivery.Pivot.getCurrentPosition() < 130){
                Top_Pivot_Position();
            }

            DepositPivot.setPosition(1);

        }

        if (gamepad1.dpad_left){
            DepositPivot.setPosition(1);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Pivot_Target = 0;

            while (delivery.Pivot.getCurrentPosition() > 0){
                Top_Pivot_Position();
            }

            DepositPivot.setPosition(1);

            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            DepositServoRotate.setPosition(0.5);

            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            clawLeft.setPosition(0.6);
            clawRight.setPosition(0.4);

        }

        /**Top pivot code*/

        /**Intake Toggle*/

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && collection.Intake.getPower() == 0) {
            collection.Intake.setPower(1);
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            collection.Intake.setPower(0);
        }

        if (currentGamepad1.y && !previousGamepad1.y){
            collection.Intake.setPower(-1);
        }

        /**Call all PID's and telemetry code*/

        Top_Pivot_Position();

        TelemetryMap();

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collection.init(hardwareMap);

        delivery.init(hardwareMap);

        odometry.init(hardwareMap);

        DepositServoRotate = hardwareMap.get(Servo.class, "DepositRotate");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        DepositPivot = hardwareMap.get(Servo.class, "DepositPivot");
        TestFeetech = hardwareMap.get(Servo.class, "fitec");
        IntakeServo = hardwareMap.get(Servo.class, "intake_servo");


        Slide_Power = new PIDFController(pivot_p, pivot_i, pivot_d, 0);

        currentGamepad1 = new Gamepad();

        previousGamepad1 = new Gamepad();

        runtime.reset();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        clawLeft.setPosition(0.6);
        clawRight.setPosition(0.4);
        DepositServoRotate.setPosition(0.5);
        DepositPivot.setPosition(1);

        TestFeetech.setPosition(0);

        IntakeServo.setPosition(0);

    }

    public void Top_Pivot_Position(){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.6;

        delivery.Pivot.setPower(Top_Pivot_PID);

    }

    public void Top_Pivot_Position_With_Feedforward(){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.2;

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
        telemetry.addData("Deposit Rotate", delivery.Pivot.getVelocity());
        telemetry.addData("pivot pos", delivery.Pivot.getCurrentPosition());
        telemetry.addData("pivot target", Pivot_Target);
        telemetry.update();
    }

}
