package org.firstinspires.ftc.teamcode.Code_Under_Development.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Code_Under_Development.hardware.SubSystems.Drivetrain;

@Autonomous
public class Scrimmage_Bot_Auto extends LinearOpMode {

    DcMotor Left_Drive;
    DcMotor Right_Drive;

    static final double     COUNTS_PER_MOTOR_REV    = 960;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCHES   = 9;
    static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public BNO055IMU imu = null;

    @Override
    public void runOpMode() throws InterruptedException {

        Left_Drive = hardwareMap.get(DcMotor.class, "left_drive");
        Right_Drive = hardwareMap.get(DcMotor.class, "right_drive");

        Left_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        waitForStart();

        //start code

        turnToHeadingWithImu(imu, 90, this);

        encoderDrive(0.4, 20);

    }

    public void encoderDrive(double speed, double TargetCm) {

        Left_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = Left_Drive.getCurrentPosition() + (int)(TargetCm * COUNTS_PER_CM);
        newRightTarget = Right_Drive.getCurrentPosition() + (int)(TargetCm * COUNTS_PER_CM);

        Left_Drive.setTargetPosition(newLeftTarget);
        Right_Drive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        Left_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Left_Drive.setPower(speed);
        Right_Drive.setPower(speed);

        while (Left_Drive.isBusy() && Right_Drive.isBusy()) {
            //do nothing
        }

        // Stop all motion
        Left_Drive.setPower(0);
        Right_Drive.setPower(0);

        // Turn off RUN_TO_POSITION
        Left_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnToHeadingWithImu(BNO055IMU imu, double targetHeading, LinearOpMode opMode){

        double headingError = targetHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (opMode.opModeIsActive() && (Math.abs(headingError) > 0.8)) {

            if (headingError > 180){
                headingError -= 360;
            }else if (headingError <= -180){
                headingError += 360;
            }

            // Determine required steering to keep on heading
            double turnSpeed = Range.clip(headingError * 0.016, -0.4, 0.4);

            Left_Drive.setPower(turnSpeed);
            Right_Drive.setPower(-turnSpeed);

        }

        Left_Drive.setPower(0);
        Right_Drive.setPower(0);

    }
    
}
