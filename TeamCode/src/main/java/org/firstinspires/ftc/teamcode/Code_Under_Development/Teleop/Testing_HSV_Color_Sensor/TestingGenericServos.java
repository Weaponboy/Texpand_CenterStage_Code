package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop.Testing_HSV_Color_Sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TestingGenericServos extends OpMode {

    ServoImplEx smallBoi;

    @Override
    public void init() {

        smallBoi = hardwareMap.get(ServoImplEx.class, "smallBoi");

        smallBoi.setPwmRange(new PwmControl.PwmRange(600, 2400));

    }

    @Override
    public void loop() {

        if (gamepad1.a && smallBoi.getPosition() == 0){
            smallBoi.setPosition(1);
        }else if(gamepad1.a && smallBoi.getPosition() == 1){
            smallBoi.setPosition(0);
        }
    }
}
