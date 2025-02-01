package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * TeleOp mode with new slower drive code and without filters.
 * Retains arm, precision mode, camera, and servo controls.
 */
@TeleOp(name = "Mecanum Drive", group = "Iterative OpMode")
public class TeleOpMode extends OpMode {
    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    util core = new util();  // This holds your hardware (motors, servos, etc.)

    // Arm and hand variables
    double armBasePower = 0;
    double handPower = 0.3;

    // Camera and vision variable (target color toggle)
    static boolean targetBlue = true; // Default to blue target
    private OpenCvCamera camera;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        core.init(hardwareMap);

        // --- Camera Setup ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "cam_1"), cameraMonitorViewId);
        camera.setPipeline(new GamePiecePipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Error opening camera: " + errorCode);
            }
        });
        telemetry.addData("Camera", "Opened and streaming");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * Main loop: new slower drive code combined with arm, servo, and camera controls.
     */
    @Override
    public void loop() {
        // --- New Slow Drive Code ---
        // Base slow speed (50% power). Left bumper (precision mode) reduces it further.
        double baseSpeedFactor = 0.5;
        double speedFactor = gamepad1.left_bumper ? 0.2 : baseSpeedFactor;

        // Scale joystick inputs by speedFactor.
        double y  = -gamepad1.left_stick_y * speedFactor;        // Forward/backward (invert Y)
        double x  = gamepad1.left_stick_x * speedFactor * 1.1;       // Strafing (with slight adjustment)
        double rx = gamepad1.right_stick_x * speedFactor;           // Rotation

        // Normalize motor power values so no value exceeds 1.0.
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        // Send drive powers to the motors.
        core.frontLeftDrive.setPower(frontLeftPower);
        core.frontRightDrive.setPower(frontRightPower);
        core.backLeftDrive.setPower(backLeftPower);
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
        } else {
            core.handServo.setPower(0);
        }

        // --- Target Color Toggle ---
        // Toggle target color with 'B' button.
        if (gamepad1.b) {
            targetBlue = !targetBlue; // Toggle between blue and red
            // A brief pause for debounce.
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                // Handle exception if needed.
            }
        }

        // --- Telemetry ---
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", String.format("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower));
        telemetry.addData("Arm Base Power", armBasePower);
        telemetry.addData("Hand Servo Power", core.handServo.getPower());
        telemetry.addData("Precision Mode", gamepad1.left_bumper ? "ON" : "OFF");
        telemetry.addData("Target Color", targetBlue ? "Blue" : "Red");
        telemetry.update();
    }

    // --- Vision Pipeline (unchanged) ---
    static class GamePiecePipeline extends OpenCvPipeline {
        static volatile double centerX = 160; // Initialize to frame center

        @Override
        public Mat processFrame(Mat input) {
            Scalar lowerBound, upperBound;
            if (targetBlue) {
                lowerBound = new Scalar(100, 150, 50);
                upperBound = new Scalar(130, 255, 255);
            } else {
                lowerBound = new Scalar(0, 150, 50);
                upperBound = new Scalar(10, 255, 255);
            }

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Mat mask = new Mat();

            if (targetBlue) {
                Core.inRange(hsv, lowerBound, upperBound, mask);
            } else {
                Mat lowerRed = new Mat();
                Mat upperRed = new Mat();
                Core.inRange(hsv, new Scalar(0, 150, 50), new Scalar(10, 255, 255), lowerRed);
                Core.inRange(hsv, new Scalar(170, 150, 50), new Scalar(180, 255, 255), upperRed);
                Core.addWeighted(lowerRed, 1.0, upperRed, 1.0, 0.0, mask);
                lowerRed.release();
                upperRed.release();
            }

            Mat morph = new Mat();
            Imgproc.erode(mask, morph, new Mat(), new Point(-1, -1), 2);
            Imgproc.dilate(morph, morph, new Mat(), new Point(-1, -1), 2);
            Rect boundingRect = Imgproc.boundingRect(morph);

            if (boundingRect.width > 0 && boundingRect.height > 0) {
                Point topLeft = boundingRect.tl();
                Point bottomRight = boundingRect.br();
                centerX = (topLeft.x + bottomRight.x) / 2;
                Imgproc.rectangle(input, topLeft, bottomRight, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(centerX, (topLeft.y + bottomRight.y) / 2), 5, new Scalar(255, 0, 0), -1);
            }

            mask.release();
            hsv.release();
            morph.release();
            return input;
        }

        public static double getCurrentCenterX() {
            return centerX;
        }
    }
}
