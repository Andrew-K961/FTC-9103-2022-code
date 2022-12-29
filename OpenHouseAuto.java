package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Open house")
@Disabled
public class OpenHouseAuto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    Servo servoLeft;
    Servo servoRight;
    double y = 0;
    double x = 0;
    double rx = 0;
    double botHeading;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.dcMotor.get("BackLeft");
        backRight = hardwareMap.dcMotor.get("BackRight");
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        double ogBotHeading = -imu.getAngularOrientation().firstAngle;
        for (int i = 0; i < 400; i++) {
            botHeading = -imu.getAngularOrientation().firstAngle;
            if (i < 101) {
                y = 0;
                x = -0.5;
                rx = 0;
            } else if (i < 118){
                y = .25;
                x = 0;
                rx = 0;
            }
            /* else if( i > 170 && Math.abs(ogBotHeading + imu.getAngularOrientation().firstAngle) < 1.2) {
                    y = 0;
                    x = 0;
                    rx = 0.7;
                } else if (i > 250 && i < 330){
                    y = -.5;
                    x = 0;
                    rx = 0;
                } else {
                    y = 0;
                    x = 0;
                    rx = 0;
                }*/

            telemetry.addData("gyro:", botHeading);
            mecanumDrive();
            telemetry.update();
            if (isStopRequested()) return;
        }
    }

    public void mecanumDrive() {
        double rotX = x * Math.cos(botHeading) + y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * -Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}