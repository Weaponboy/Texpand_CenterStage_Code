package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.slide_d;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.slide_i;
import static org.firstinspires.ftc.teamcode.Code_Under_Development.Constants_and_Setpoints.Constants.slide_p;

public class hardwareControl {

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

    public void init(HardwareMap Hmap){

        harwareMap = Hmap;

        LF = Hmap.get(DcMotorEx.class, "LF");
        LB = Hmap.get(DcMotorEx.class, "LB");
        RF = Hmap.get(DcMotorEx.class, "RF");
        RB = Hmap.get(DcMotorEx.class, "RB");

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


        Left_Slide = Hmap.get(DcMotorEx.class, "Left_Slide");
        Right_Slide = Hmap.get(DcMotorEx.class, "Right_Slide");

        Right_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide_Power = new PIDFController(slide_p, slide_i, slide_d, 0);

        Intake = Hmap.get(DcMotorEx.class, "intake");



        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




    }
}
