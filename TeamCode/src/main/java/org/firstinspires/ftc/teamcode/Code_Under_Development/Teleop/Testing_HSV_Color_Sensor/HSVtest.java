package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop.Testing_HSV_Color_Sensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Arrays;

@TeleOp
public class HSVtest extends OpMode {
    ColorSensor right_Pixel;
    float[] hsvval = new float[3];

    private static final double MIN_HUE = 0.001; // Minimum hue value for the target color
    private static final double MAX_HUE = 360; // Maximum hue value for the target color
    private static final double MIN_SATURATION = 0.01; // Minimum saturation value for the target color
    private static final double MAX_SATURATION = 100; // Maximum saturation value for the target color
    private static final double MIN_VALUE = 0.0001; // Minimum value (brightness) for the target color
    private static final double MAX_VALUE = 100; // Maximum value (brightness) for the target color

    private static final double MIN_HUE_PURPLE = 185 ; // Minimum value (brightness) for the target color
    private static final double MAX_HUE_PURPLE = 250; // Maximum value (brightness) for the target color

    private static final double MIN_HUE_YELLOW = 60 ; // Minimum value (brightness) for the target color
    private static final double MAX_HUE_YELLOW = 95; // Maximum value (brightness) for the target color

    private static final double MIN_HUE_GREEN = 120 ; // Minimum value (brightness) for the target color
    private static final double MAX_HUE_GREEN = 140; // Maximum value (brightness) for the target color

    private static final double MIN_HUE_WHITE = 147 ; // Minimum value (brightness) for the target color
    private static final double MAX_HUE_WHITE = 180; // Maximum value (brightness) for the target color

    boolean purple = false;
    boolean yellow = false;
    boolean green = false;
    boolean white = false;
    boolean isTargetColorDetected = false;



    @Override
    public void init() {
        right_Pixel = hardwareMap.get(ColorSensor.class, "right_Pixel");
    }

    @Override
    public void loop() {

        int argb_val = right_Pixel.argb();

        Color.colorToHSV(argb_val, hsvval);

        float hue = hsvval[0];
        float saturation = hsvval[1];
        float value = hsvval[2];

        if (hue >= MIN_HUE && hue <= MAX_HUE && saturation >= MIN_SATURATION && saturation <= MAX_SATURATION && value >= MIN_VALUE && value <= MAX_VALUE) {
            isTargetColorDetected = true;
        }
        else{
            isTargetColorDetected = false;
        }
        if (hue >= MIN_HUE_PURPLE && hue <= MAX_HUE_PURPLE && saturation >= MIN_SATURATION && saturation <= MAX_SATURATION && value >= MIN_VALUE && value <= MAX_VALUE) {
            purple = true;
        }
        else{
            purple = false;
        }
        if (hue >= MIN_HUE_YELLOW && hue <= MAX_HUE_YELLOW && saturation >= MIN_SATURATION && saturation <= MAX_SATURATION && value >= MIN_VALUE && value <= MAX_VALUE) {
            yellow = true;
        }
        else{
            yellow = false;
        }
        if (hue >= MIN_HUE_GREEN && hue <= MAX_HUE_GREEN && saturation >= MIN_SATURATION && saturation <= MAX_SATURATION && value >= MIN_VALUE && value <= MAX_VALUE) {
            green = true;
        }
        else{
            green = false;
        }
        if (hue >= MIN_HUE_WHITE && hue <= MAX_HUE_WHITE && saturation >= MIN_SATURATION && saturation <= MAX_SATURATION && value >= MIN_VALUE && value <= MAX_VALUE) {
            white = true;
        }
        else{
            white = false;
        }
        telemetry.addData("colorDetected", isTargetColorDetected);
        telemetry.addData("hsv", Arrays.toString(hsvval));
        telemetry.addData("purple", purple);
        telemetry.addData("yellow",yellow );
        telemetry.addData("green",green);
        telemetry.addData("white",white);
        telemetry.update();

    }
}
