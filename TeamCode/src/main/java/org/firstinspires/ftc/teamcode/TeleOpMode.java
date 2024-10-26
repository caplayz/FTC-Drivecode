package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Code for TeleOp mode, where you use the controller
 * @see util
 * @see OpMode
 * @see TeleOp
 */
@TeleOp(name="Mecanum Drive", group="Iterative OpMode")
public class TeleOpMode extends OpMode
{
    //TODO : fix joint / arm
    //

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    util core = new util();
    private double movementSpeedMultiplier = 1.0;
    private double armBasePower = 0;
    private double jointOnePower = 0;

    private boolean alignMode = false;
    private static boolean targetBlue = true; // Default to blue target
    private OpenCvCamera camera;
    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        core.init(hardwareMap);
        
        // Camera setup
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
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
// Toggle alignment mode with 'A' button and set color target
        if (gamepad1.a) {
            alignMode = true;
        } else {
            alignMode = false;
        }

        if (gamepad1.b) {
            targetBlue = !targetBlue; // Toggle between blue and red
        }

        if (alignMode) {
            // Vision alignment mode
            double centerX = GamePiecePipeline.centerX;
            double frameCenter = 160; // Half of the 320px frame width
            double error = frameCenter - centerX;

            if (Math.abs(error) > 10) { // Allowable error for alignment
                double alignmentPower = Range.clip(error * 0.01, -0.3, 0.3);
                frontLeftPower = -alignmentPower;
                frontRightPower = alignmentPower;
                backLeftPower = -alignmentPower;
                backRightPower = alignmentPower;
            } else {
                frontLeftPower = 0;
                frontRightPower = 0;
                backLeftPower = 0;
                backRightPower = 0;
                telemetry.addData("Alignment", "Aligned!");
            }
        } else {
            // Standard drive mode
            movementSpeedMultiplier = gamepad1.left_bumper ? 0.5 : 1;

            double drive = -gamepad1.left_stick_y * movementSpeedMultiplier;
            double strafe = gamepad1.left_stick_x * 0.5 * movementSpeedMultiplier; // Reduce strafing speed
            double turn = gamepad1.right_stick_x * movementSpeedMultiplier;

            frontLeftPower = drive + strafe + turn;
            frontRightPower = drive - strafe - turn;
            backLeftPower = drive - strafe + turn;
            backRightPower = drive + strafe - turn;

            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);
        }


        // Send calculated power to wheels
        core.frontLeftDrive.setPower(frontLeftPower);
        core.frontRightDrive.setPower(frontRightPower);
        core.backLeftDrive.setPower(backLeftPower);
        core.backRightDrive.setPower(backRightPower);

        // Control the arm using the D-pad up and down buttons
        if (gamepad1.dpad_up) {
            armBasePower = 0.5;
        } else if (gamepad1.dpad_down) {
            armBasePower = -0.5;
        } else {
            armBasePower = 0;
        }

        armBasePower *= 0.05;

        // Send calculated power to arm motors
        core.armBaseMotor.setPower(armBasePower);
        core.jointOneMotor.setPower(jointOnePower);
        // Send calculated power to arm motor
        core.armBaseMotor.setPower(armBasePower);

        // Show the elapsed game time and wheel power.
        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Arm Base Power", armBasePower);
        telemetry.addData("Joint One Power", jointOnePower);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Alignment Mode", alignMode ? "ON" : "OFF");
        telemetry.addData("Target Color", targetBlue ? "Blue" : "Red");
       
        updateTelemetry(telemetry);
    }

    static class GamePiecePipeline extends OpenCvPipeline {
        static volatile double centerX = 0;

        @Override
        public Mat processFrame(Mat input) {
            Scalar lowerBound, upperBound;

            // Define color ranges for detection
            if (targetBlue) {
                lowerBound = new Scalar(100, 100, 100); // Blue range, adjust values
                upperBound = new Scalar(130, 255, 255);
            } else {
                lowerBound = new Scalar(0, 100, 100);   // Red range, adjust values
                upperBound = new Scalar(10, 255, 255);
            }

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Mat mask = new Mat();
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // Get bounding rectangle
            Rect boundingRect = Imgproc.boundingRect(mask);
            Point topLeft = boundingRect.tl();
            Point bottomRight = boundingRect.br();

            // Calculate the center X of the bounding box
            centerX = (topLeft.x + bottomRight.x) / 2;

            // Draw rectangle and center point for visualization
            Imgproc.rectangle(input, topLeft, bottomRight, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(centerX, topLeft.y), 5, new Scalar(255, 0, 0), -1);

            mask.release();
            hsv.release();

            return input;
        }
    }
}