package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "TEST")
@Disabled
public class FieldCentricMecanumTeleOp extends LinearOpMode {

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor lift;
    double position = 0;
    Servo servoLeft;
    Servo servoRight;
    boolean rtriggerDown = false;
    boolean ltriggerDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        //boolean buttonDown = false;
        //boolean clawOpen = false;

        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorBackRight = hardwareMap.dcMotor.get("BackRight");
        lift = hardwareMap.dcMotor.get("liftMotor");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        waitForStart();
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.REVERSE);
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y *.5; // Remember, this is reversed!
            double x = gamepad1.left_stick_x *.5; //TODO Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * .4;

            // Read inverse IMU heading, as the IMU heading is CW positive
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

        }

        if(gamepad1.x) {
            openClaw();
        }
        else if(gamepad1.a) {
            closeClaw();
        }

        if (gamepad1.right_bumper){
            lift.setPower(.25);
            rtriggerDown = true;
        } else {
            lift.setPower(0);
            rtriggerDown =false;
        }

        if (gamepad1.left_bumper && !gamepad1.right_bumper){
            lift.setPower(-.25);
            ltriggerDown = true;
        } else {
            lift.setPower(0);
            ltriggerDown =false;
        }
    }

    public void openClaw () {
        position = 0;
        servoLeft.setPosition(position);
        servoRight.setPosition(position);
    }

    public void closeClaw () {
        position = 1;
        servoLeft.setPosition(position);
        servoRight.setPosition(position);
    }
}