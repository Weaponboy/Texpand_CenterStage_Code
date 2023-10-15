package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop.Odometry_Integration;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class HSVtest extends OpMode {
    ColorSensor right_Pixel;
    float[] hsvval = new float[3];

    private static final float MIN_HUE = 100; // Minimum hue value for the target color
    private static final float MAX_HUE = 150; // Maximum hue value for the target color
    private static final float MIN_SATURATION = 0.5f; // Minimum saturation value for the target color
    private static final float MAX_SATURATION = 1.0f; // Maximum saturation value for the target color
    private static final float MIN_VALUE = 0.5f; // Minimum value (brightness) for the target color
    private static final float MAX_VALUE = 1.0f; // Maximum value (brightness) for the target color
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

        if (hue >= MIN_HUE && hue <= MAX_HUE &&
                saturation >= MIN_SATURATION && saturation <= MAX_SATURATION &&
                value >= MIN_VALUE && value <= MAX_VALUE) {
            isTargetColorDetected = true;
        }
    }
}
