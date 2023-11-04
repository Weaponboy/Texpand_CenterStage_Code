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

    private static final double MIN_HUE = 0.01; // Minimum hue value for the target color
    private static final double MAX_HUE = 360; // Maximum hue value for the target color
    private static final double MIN_SATURATION = 0.01; // Minimum saturation value for the target color
    private static final double MAX_SATURATION = 100; // Maximum saturation value for the target color
    private static final double MIN_VALUE = 0.01; // Minimum value (brightness) for the target color
    private static final double MAX_VALUE = 100; // Maximum value (brightness) for the target color
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
        telemetry.addData("colorDetected", isTargetColorDetected);
        telemetry.addData("hsv", Arrays.toString(hsvval));
        telemetry.update();

    }
}
