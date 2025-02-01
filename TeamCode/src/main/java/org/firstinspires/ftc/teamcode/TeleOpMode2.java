package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpMode2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("back_right_drive");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.2
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Choose a speed factor between 0 and 1 to slow down the robot.
        double speedFactor = 0.6;  // 50% of full speed

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Scale the joystick inputs to slow down movement
            double y = -gamepad1.left_stick_y * speedFactor; // Forward/backward is scaled
            double x = gamepad1.left_stick_x * speedFactor * 1.1; // Strafing (with adjustment)
            double rx = gamepad1.right_stick_x * speedFactor; // Rotation is scaled

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
