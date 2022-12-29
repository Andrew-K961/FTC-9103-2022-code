package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lift test")
public class LiftTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lift = hardwareMap.dcMotor.get("test");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper){
                lift.setPower(1);
            }
            else if (gamepad1.left_bumper){
                lift.setPower(-.9);
            }
            else if (gamepad1.a){
                lift.setPower(.8);
            }
            else if (gamepad1.b){
                lift.setPower(.5);
            } else {
                lift.setPower(0);
            }
        }
    }
}
