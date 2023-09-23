package org.firstinspires.ftc.teamcode.Code_Under_Development.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.Odometry;

public class odometry_Calibration extends OpMode {

    Odometry odo = new Odometry();

    @Override
    public void init() {
        odo.init(hardwareMap);
    }

    @Override
    public void loop() {
        odo.update();
        telemetry.addData("left pod", odo.currentLeftPod);
        telemetry.addData("right pod", odo.currentRightPod);
        telemetry.addData("center pod", odo.currentCenterPod);
        telemetry.update();

    }

}
