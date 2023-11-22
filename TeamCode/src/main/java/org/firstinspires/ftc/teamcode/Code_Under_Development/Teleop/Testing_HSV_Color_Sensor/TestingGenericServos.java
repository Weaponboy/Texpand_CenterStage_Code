package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop.Testing_HSV_Color_Sensor;

import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class TestingGenericServos extends OpMode {

    ServoImplEx pivot1;
    ServoImplEx pivot2;

    @Override
    public void init() {

        pivot1 = hardwareMap.get(ServoImplEx.class, "pivot1");
        pivot2 = hardwareMap.get(ServoImplEx.class, "pivot2");

        pivot2.setDirection(Servo.Direction.REVERSE);

        pivot1.setPwmRange(new PwmControl.PwmRange(600, 2400));
        pivot2.setPwmRange(new PwmControl.PwmRange(600, 2400));

        previousGamepad1 = new Gamepad();

        currentGamepad1 = new Gamepad();

    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.a && !previousGamepad1.a && pivot2.getPosition() == 0){
            pivot2.setPosition(1);
            pivot1.setPosition(1);
        }else if(currentGamepad1.a && !previousGamepad1.a && pivot2.getPosition() == 1){
            pivot2.setPosition(0);
            pivot1.setPosition(0);
        }
    }
}
