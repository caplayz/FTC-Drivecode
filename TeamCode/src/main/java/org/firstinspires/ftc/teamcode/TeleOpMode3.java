package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.Util;

@TeleOp
public class TeleOpMode3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        util core = new util();
        core.init(hardwareMap);
        // --- Define variables ---
        double armBasePower;
        double handPower = 0.3;
        double baseSpeedFactor = 0.6;
        double speedFactor;
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- Drive Movement ---
            // Precision mode
            speedFactor = gamepad1.left_bumper ? 0.2 : baseSpeedFactor;
            // Scale the joystick inputs to slow down movement
            double y = -gamepad1.left_stick_y * speedFactor; // Forward/backward is scaled
            double x = -gamepad1.left_stick_x * speedFactor * 1.1; // Strafing (with adjustment)
            double rx = -gamepad1.right_stick_x * speedFactor; // Rotation is scaled

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            core.frontLeftDrive.setPower(frontLeftPower);
            core.backLeftDrive.setPower(backLeftPower);
            core.frontRightDrive.setPower(frontRightPower);
            core.backRightDrive.setPower(backRightPower);

            // --- Arm Control ---
            // Use D-pad up/down to control the arm base motor.
            if (gamepad1.dpad_up) {
                armBasePower = 0.5;
            } else if (gamepad1.dpad_down) {
                armBasePower = -0.5;
            } else {
                armBasePower = 0;
            }
            // Apply a scaling factor to the arm power.
            armBasePower *= 0.75;
            core.armBaseMotor.setPower(armBasePower);

            // --- Hand (Servo) Control ---
            // Use right trigger to move the hand one way and right bumper to move it the opposite way.
            if (gamepad1.right_trigger > 0) {
                core.handServo.setPower(handPower);
            } else if (gamepad1.right_bumper) {
                core.handServo.setPower(-handPower);
            }

            // --- Telemetry ---
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Motors", String.format("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower));
            telemetry.addData("Movement Speed Factor", speedFactor);
            telemetry.addData("Arm Base Power", armBasePower);
            telemetry.addData("Hand Servo Power", core.handServo.getPower());
            telemetry.addData("Precision Mode", gamepad1.left_bumper ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
