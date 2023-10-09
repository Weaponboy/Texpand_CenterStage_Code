package org.firstinspires.ftc.teamcode.Code_Under_Development.hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Delivery {
    public DcMotorEx Pivot;
    public PIDFController pivot_controllers;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    HardwareMap hmap;

    public void init(HardwareMap hardwareMap){
        hmap = hardwareMap;

        pivot_controllers = new PIDFController(pivot_p, pivot_i, pivot_d, 0.05);

        pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0.05);

        Pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        Pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
