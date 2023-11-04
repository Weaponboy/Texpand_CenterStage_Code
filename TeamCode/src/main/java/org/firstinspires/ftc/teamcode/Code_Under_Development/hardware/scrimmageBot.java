package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class scrimmageBot extends OpMode {

    DcMotor RF;

    DcMotor LF;

    DcMotor Pivot2;

    Servo Pivot;

    public Gamepad currentGamepad1;

    public Gamepad previousGamepad1;

    ColorSensor right_Pixel;
    float[] hsvval = new float[3];

    private static final double MIN_HUE = 0; // Minimum hue value for the target color
    private static final double MAX_HUE = 360; // Maximum hue value for the target color
    private static final double MIN_SATURATION = 0; // Minimum saturation value for the target color
    private static final double MAX_SATURATION = 100; // Maximum saturation value for the target color
    private static final double MIN_VALUE = 0; // Minimum value (brightness) for the target color
    private static final double MAX_VALUE = 100; // Maximum value (brightness) for the target color
    boolean isTargetColorDetected = false;

    @Override
    public void init() {
    RF = hardwareMap.get(DcMotor.class,"RF");
    LF = hardwareMap.get(DcMotor.class,"LF");
    Pivot2 = hardwareMap.get(DcMotor.class,"Pivot2");
    Pivot = hardwareMap.get(Servo.class,"Pivot");

    RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    currentGamepad1 = new Gamepad();
    previousGamepad1 = new Gamepad();
    right_Pixel = hardwareMap.get(ColorSensor.class, "right_Pixel");

    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        double vertical = gamepad1.right_stick_y*1.2;
        double pivot = -gamepad1.left_stick_x*1.2;

        double denominator = Math.max(Math.abs(vertical) + Math.abs(pivot), 1);

        if (currentGamepad1.a && !previousGamepad1.a && Pivot.getPosition() < 1) {
            Pivot.setPosition(1);
        } else if (currentGamepad1.a && !previousGamepad1.a && Pivot.getPosition() > 0.9) {
            Pivot.setPosition(0);
        }

        if (gamepad1.left_bumper){
            pivot = pivot * 0.3;
        }

        RF.setPower(0.6*((vertical - pivot)/denominator));
        LF.setPower(0.6*((vertical + pivot)/denominator));

        telemetry.addData("PivotPosition", Pivot2.getCurrentPosition());
        telemetry.update();

        if (gamepad1.x) {
            Pivot2.setTargetPosition(0);
            Pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot2.setPower(1);
        }

        if (gamepad1.b && Pivot2.getCurrentPosition() < 1400) {
            Pivot2.setTargetPosition(1400);
            Pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot2.setPower(1);
        } else if (gamepad1.b && Pivot2.getCurrentPosition() > 1400) {
            Pivot2.setTargetPosition(1400);
            Pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot2.setPower(-1);
        }

        if (gamepad1.right_trigger > 0 && Pivot2.getCurrentPosition() > 1300){
            if(Pivot2.getCurrentPosition() < 1480){
                Pivot2.setTargetPosition(Pivot2.getCurrentPosition()+5);
            }
            Pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot2.setPower(0.5);
        }

        if (gamepad1.y) {
            Pivot2.setTargetPosition(971);
            Pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (Pivot2.getCurrentPosition()>971) {
                Pivot2.setPower(-1);
            }
            if (Pivot2.getCurrentPosition()<971) {
                Pivot2.setPower(1);
            }
        }
        int argb_val = right_Pixel.argb();

        Color.colorToHSV(argb_val, hsvval);

        float hue = hsvval[0];
        float saturation = hsvval[1];
        float value = hsvval[2];

        if (hue >= MIN_HUE && hue <= MAX_HUE && saturation >= MIN_SATURATION && saturation <= MAX_SATURATION && value >= MIN_VALUE && value <= MAX_VALUE) {
            isTargetColorDetected = true;
        }
        telemetry.addData("colorDetected", isTargetColorDetected);
        telemetry.addData("hsv", hsvval);

    }
}

