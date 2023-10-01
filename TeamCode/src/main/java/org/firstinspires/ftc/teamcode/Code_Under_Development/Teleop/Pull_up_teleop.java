package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Intake_And_Pivot;

import java.util.List;

@TeleOp
public class Pull_up_teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Intake_And_Pivot slidey = new Intake_And_Pivot();

    PIDFController Slide_Power;

    Servo LeftClaw;

    Servo RightClaw;

    int Counter = 0;

    int Pivot_Target = 0;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    @Override
    public void loop() {

        if (gamepad1.x){
            //pull up 1
            letDown();
            pullUp();
            //pull up 2
            letDown();
            pullUp();
            //pull up 3
            letDown();
            pullUp();
            //pull up 4
            letDown();
            pullUp();
            //pull up 5
            letDown();
            pullUp();
        }

        Slide_Position();

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        slidey.init(hardwareMap);

        LeftClaw = hardwareMap.get(Servo.class, "left_claw");

        RightClaw = hardwareMap.get(Servo.class, "right_claw");

        Slide_Power = new PIDFController(pivot_p, pivot_i, pivot_d, 0);

        currentGamepad1 = new Gamepad();

        previousGamepad1 = new Gamepad();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void Slide_Position(){

        slidey.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = deliverySlides.Left_Slide.getCurrentPosition();

        double Top_Pivot_PID = slidey.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.5;

        deliverySlides.SlidesBothPower(Top_Pivot_PID);

    }

    public void Slide_Position_With_Feedforward(){

        slidey.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = slidey.Pivot.getCurrentPosition();

        double Top_Pivot_PID = slidey.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.8;

        double pivot_f = 0.1;

        double ticks_in_degrees = 560 / 180.0;

        double Pivot_FF = Math.cos(Math.toRadians(Pivot_Target / ticks_in_degrees)) * pivot_f;

        double Pivot_Power = Top_Pivot_PID + Pivot_FF;

        slidey.Pivot.setPower(Pivot_Power);

    }

    public void letDown(){
        Pivot_Target = 800;
        while(deliverySlides.Left_Slide.getCurrentPosition() < 780){
            Slide_Position();
        }
        Counter = 0;
        while (Counter < 10){
            Counter++;
            Slide_Position();
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void pullUp(){
        Pivot_Target = 0;
        while(deliverySlides.Left_Slide.getCurrentPosition() > 20){
            Slide_Position();
        }
        Counter = 0;
        while (Counter < 10){
            Counter++;
            Slide_Position();
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

}
