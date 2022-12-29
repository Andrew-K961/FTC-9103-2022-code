package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "One Driver")
public class OneDriver extends OpMode {

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor lift;
    Servo servoLeft;
    Servo servoRight;
    BNO055IMU imu;

    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorBackRight = hardwareMap.dcMotor.get("BackRight");
        lift = hardwareMap.dcMotor.get("test");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.REVERSE);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double y = gamepad2.left_stick_y *.35;
        double x = gamepad2.left_stick_x *.35;
        double rx = gamepad2.right_stick_x * .35;

        double botHeading = -imu.getAngularOrientation().firstAngle;

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

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        if (gamepad2.left_bumper){
            lift.setPower(-1);
        }
        else if (gamepad2.right_bumper){
            lift.setPower(.7);
        }
        else if (gamepad2.right_trigger > 0.1){
            lift.setPower(.35);
        }
        else if (gamepad2.left_trigger > 0.1){
            lift.setPower(-.45);
        }
        else {
            lift.setPower(0.12);
        }

        if(gamepad2.a) {
            servoLeft.setPosition(.7);
            servoRight.setPosition(.75);
        }
        else if(gamepad2.x) {
            servoLeft.setPosition(0);
            servoRight.setPosition(0);
        }
    }

    @Override
    public void stop() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        lift.setPower(0);
    }
}
