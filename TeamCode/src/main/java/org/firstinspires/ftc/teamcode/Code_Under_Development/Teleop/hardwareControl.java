package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.slide_d;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.slide_i;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.slide_p;

public class hardwareControl extends OpMode {

    public DcMotorEx LF;
    public DcMotorEx LB;
    public DcMotorEx RF;
    public DcMotorEx RB;

    public PIDFController Slide_Power;

    public DcMotorEx Left_Slide;
    public DcMotorEx Right_Slide;

    public DcMotorEx Intake;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;


    HardwareMap harwareMap;


    public void init(HardwareMap Hmap) {


    }

    @Override
    public void init() {


        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RF = harwareMap.get(DcMotorEx.class, "RF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);


        Left_Slide = harwareMap.get(DcMotorEx.class, "Left_Slide");
        Right_Slide = hardwareMap.get(DcMotorEx.class, "Right_Slide");

        Right_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide_Power = new PIDFController(slide_p, slide_i, slide_d, 0);

        Intake = harwareMap.get(DcMotorEx.class, "intake");


        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        double vertical = -gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x*1.5;
        double pivot = gamepad1.left_stick_x;

        RF.setPower((-pivot + (vertical - horizontal)));
        RB.setPower((1.15)*(-pivot + (vertical + horizontal)));
        LF.setPower((pivot + (vertical + horizontal)));
        LB.setPower((1.15)*(pivot + (vertical - horizontal)));


        if (gamepad1.a) {
            Intake.setPower(1);


        } else if (gamepad1.a) {
            Intake.setPower(0);
        }

        if (gamepad1.b) {
            Left_Slide.setPower(1);

        } else if (gamepad1.b) {
            Left_Slide.setPower(0);
        }
        if (gamepad1.b) {
            Right_Slide.setPower(1);

        } else if (gamepad1.b) {
            Right_Slide.setPower(0);

        }


    }
}

