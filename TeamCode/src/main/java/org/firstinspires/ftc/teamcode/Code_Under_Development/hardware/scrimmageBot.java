package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public abstract class scrimmageBot extends OpMode {

    DcMotor LF;
    DcMotor RF;

    Servo Pivot;

    HardwareMap hardwareMap;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;




    public void init(HardwareMap Hmap) {

    }

    @Override
    public void loop() {
        if (gamepad1.a && Pivot.getPosition()<1);
        Pivot.setPosition(1);

           if (gamepad1.a && Pivot.getPosition()>0.9);
           Pivot.setPosition(0);

           if (gamepad1.left_stick_y<0);
           LF.setPower(1);

        if (gamepad1.left_stick_y<0);
        RF.setPower(1);

        if (gamepad1.left_stick_y>0);
        LF.setPower(-1);

        if (gamepad1.left_stick_y>0);
        RF.setPower(-1);

        if (gamepad1.left_stick_x<0);
        LF.setPower(1);

        if (gamepad1.left_stick_x<0);
        RF.setPower(0);

        if (gamepad1.left_stick_x>0);
        LF.setPower(0);

        if (gamepad1.left_stick_x>0);
        RF.setPower(1);



        double vertical = -gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x*1.5;
        double pivot = gamepad1.left_stick_x;;

        RF.setPower((-pivot + (vertical - horizontal)));
        LF.setPower((pivot + (vertical + horizontal)));





    }
}

