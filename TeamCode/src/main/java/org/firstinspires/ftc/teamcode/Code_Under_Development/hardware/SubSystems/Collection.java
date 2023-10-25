package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collection {


    public DcMotorEx Intake;


    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    HardwareMap hmap;

    public void init(HardwareMap hardwareMap){
        
        hmap = hardwareMap;



        Intake = hardwareMap.get(DcMotorEx.class, "Intake");



        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
