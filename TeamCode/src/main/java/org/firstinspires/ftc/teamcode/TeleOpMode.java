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
 * Code for TeleOp mode, where you use the controller
 * @see util
 * @see OpMode
 * @see TeleOp
 */
@TeleOp(name="Mecanum Drive", group="Iterative OpMode")
public class TeleOpMode extends OpMode
{
    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    util core = new util();
    double movementSpeedMultiplier = 1.0;
    double armBasePower = 0;
    double handPower = 0.3;
    //double jointOnePower = 0;

    boolean alignMode = false;
    static boolean targetBlue = true; // Default to blue target
    private OpenCvCamera camera;

    // Proportional control constants
    private static final double ROTATION_KP = 0.005; // Adjust as needed
    //private static final double STRAFE_KP = 0.01;   // Adjust as needed

    // Maximum power limits to prevent overcorrection
    private static final double MAX_ROTATION_POWER = 0.3;
    private static final double MAX_STRAFE_POWER = 0.3;

    // Dead Zone Thresholds
    private static final double CENTER_X_THRESHOLD = 10.0; // Pixels
    //private static final double ANGLE_THRESHOLD = 2.0;     // Degrees

    // PID Controllers
    private PIDController strafePID;
    private PIDController turnPID;

    // Filters
    private LowPassFilter centerXFilter;

    // Rate Limiters
    private RateLimiter frontLeftLimiter = new RateLimiter(0.05);
    private RateLimiter frontRightLimiter = new RateLimiter(0.05);
    private RateLimiter backLeftLimiter = new RateLimiter(0.05);
    private RateLimiter backRightLimiter = new RateLimiter(0.05);
    


    // Timing for PID calculations
    private ElapsedTime pidTimer = new ElapsedTime();

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        core.init(hardwareMap);

        // Initialize PID Controllers with tuned constants
        strafePID = new PIDController(0.01, 0.000, 0.001);
        strafePID.setOutputLimits(-MAX_STRAFE_POWER, MAX_STRAFE_POWER);

        turnPID = new PIDController(ROTATION_KP, 0.000, 0.001);
        turnPID.setOutputLimits(-MAX_ROTATION_POWER, MAX_ROTATION_POWER);

        strafePID.setSetpoint(160); // Center of the frame
        turnPID.setSetpoint(0);     // Desired angle deviation

        // Initialize Filters
        centerXFilter = new LowPassFilter(0.5); // Alpha = 0.5
        centerXFilter.reset(160); // Initialize to frame center

        // Initialize PID Timer
        pidTimer.reset();

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
        pidTimer.reset();
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

        // Toggle alignment mode with 'A' button
        alignMode = gamepad1.a;

        // Toggle target color with 'B' button
        if (gamepad1.b) {
            targetBlue = !targetBlue; // Toggle between blue and red
            strafePID.reset();
            turnPID.reset();
            centerXFilter.reset(160);
            try {
                Thread.sleep(200); // 200 ms debounce
            } catch (InterruptedException e) {
                // Handle exception
            }
        }

        if (alignMode) {
            // Vision alignment mode
            double rawCenterX = GamePiecePipeline.getCurrentCenterX();
            double filteredCenterX = centerXFilter.filter(rawCenterX);

            double currentTime = pidTimer.seconds();
            double dt = currentTime;
            pidTimer.reset();

            // Compute PID outputs
            double strafePower = strafePID.compute(filteredCenterX, dt);

            // Assuming no angle error available; set turnPower to 0
            double turnPower = 0;

            // Apply dead zones
            if (Math.abs(strafePID.setpoint - filteredCenterX) <= CENTER_X_THRESHOLD) {
                strafePower = 0;
            }
            /*
            if (Math.abs(turnPower) <= ANGLE_THRESHOLD) {
                turnPower = 0;
            }
            */


            // Combine strafing and turning powers
            frontLeftPower = -strafePower - turnPower;
            frontRightPower = strafePower - turnPower;
            backLeftPower = -strafePower + turnPower;
            backRightPower = strafePower + turnPower;

            // Apply rate limiting
            frontLeftPower = frontLeftLimiter.calculate(frontLeftPower);
            frontRightPower = frontRightLimiter.calculate(frontRightPower);
            backLeftPower = backLeftLimiter.calculate(backLeftPower);
            backRightPower = backRightLimiter.calculate(backRightPower);

            telemetry.addData("Alignment", "Aligning...");
            telemetry.addData("Raw Center X", rawCenterX);
            telemetry.addData("Filtered Center X", filteredCenterX);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.addData("Turn Power", turnPower);
        } else {
            // Standard drive mode
            movementSpeedMultiplier = gamepad1.left_bumper ? 0.2 : 1.0;

            double drive = gamepad1.left_stick_y * movementSpeedMultiplier;
            double strafe = gamepad1.left_stick_x * movementSpeedMultiplier * 0; // Reduce strafing speed
            double turn = -gamepad1.right_stick_x * movementSpeedMultiplier;

            frontLeftPower = drive + strafe + turn;
            frontRightPower = drive - strafe - turn;
            backLeftPower = drive - strafe + turn;
            backRightPower = drive + strafe - turn;

            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);
        }

        // Send calculated (and rate-limited) power to wheels
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

        
        //move hand (right trigger and bumper)i
        if (gamepad1.right_trigger>0) {
            telemetry.addData("right trigger",true);
            telemetry.addData("right bumper",false);
            core.handServo.setPower(handPower);
        }
        else if(gamepad1.right_bumper){
            telemetry.addData("right trigger",false);
            telemetry.addData("right bumper",true);
            core.handServo.setPower(-handPower);
        }
        else{
            telemetry.addData("right bumper",false);
            telemetry.addData("right trigger",false);
            core.handServo.setPower(0);
        }

        armBasePower *= 0.75;

        // Send calculated power to arm motors
        core.armBaseMotor.setPower(armBasePower);
        //core.jointOneMotor.setPower(jointOnePower);

        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", String.format("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower));
        telemetry.addData("Arm Base Power", armBasePower);
        telemetry.addData("hand servo power", core.handServo.getPower());
        //telemetry.addData("Joint One Power", jointOnePower);
        telemetry.addData("Alignment Mode", alignMode ? "ON" : "OFF");
        telemetry.addData("Target Color", targetBlue ? "Blue" : "Red");

        telemetry.update();
    }

    /**
     * Applies a dead zone to the given power value.
     * If the absolute value is below the threshold, returns 0.
     *
     * @param power The motor power to adjust.
     * @param threshold The dead zone threshold.
     * @return The adjusted motor power.
     */
    private double applyDeadZone(double power, double threshold) {
        if (Math.abs(power) < threshold) {
            return 0;
        }
        return power;
    }

    static class GamePiecePipeline extends OpenCvPipeline {
        static volatile double centerX = 160; // Initialize to frame center

        @Override
        public Mat processFrame(Mat input) {
            Scalar lowerBound, upperBound;

            // Define color ranges for detection
            if (targetBlue) {
                lowerBound = new Scalar(100, 150, 50); // Adjusted blue range
                upperBound = new Scalar(130, 255, 255);
            } else {
                // Red color handling with two ranges
                // Implemented above in the enhanced CV processing
                lowerBound = new Scalar(0, 150, 50);   // Lower range for red
                upperBound = new Scalar(10, 255, 255); // Upper hue range will require separate handling
            }

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat mask = new Mat();

            if (targetBlue) {
                Core.inRange(hsv, lowerBound, upperBound, mask);
            } else {
                // Handle red color which spans two hue ranges
                Mat lowerRed = new Mat();
                Mat upperRed = new Mat();
                Core.inRange(hsv, new Scalar(0, 150, 50), new Scalar(10, 255, 255), lowerRed);
                Core.inRange(hsv, new Scalar(170, 150, 50), new Scalar(180, 255, 255), upperRed);
                Core.addWeighted(lowerRed, 1.0, upperRed, 1.0, 0.0, mask);
                lowerRed.release();
                upperRed.release();
            }

            // Apply morphological operations to reduce noise
            Mat morph = new Mat();
            Imgproc.erode(mask, morph, new Mat(), new Point(-1, -1), 2);
            Imgproc.dilate(morph, morph, new Mat(), new Point(-1, -1), 2);

            // Get bounding rectangle
            Rect boundingRect = Imgproc.boundingRect(morph);

            // Check if any object is detected
            if (boundingRect.width > 0 && boundingRect.height > 0) {
                Point topLeft = boundingRect.tl();
                Point bottomRight = boundingRect.br();

                // Calculate the center X of the bounding box
                centerX = (topLeft.x + bottomRight.x) / 2;

                // Draw rectangle and center point for visualization
                Imgproc.rectangle(input, topLeft, bottomRight, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(centerX, (topLeft.y + bottomRight.y) / 2), 5, new Scalar(255, 0, 0), -1);
            } else {
                // No object detected; optionally keep previous centerX or set to frame center
                // centerX = 160; // Uncomment if desired
            }

            // Release resources
            mask.release();
            hsv.release();
            morph.release();

            return input;
        }

        public static double getCurrentCenterX() {
            return centerX;
        }
    }

    // PID Controller Class
    public class PIDController {
        private double kp, ki, kd;
        private double setpoint;

        private double integral;
        private double previousError;

        private double outputMin = -1.0;
        private double outputMax = 1.0;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.setpoint = 0;
            this.integral = 0;
            this.previousError = 0;
        }

        public void setSetpoint(double setpoint) {
            this.setpoint = setpoint;
        }

        public double compute(double measurement, double dt) {
            double error = setpoint - measurement;
            integral += error * dt;
            double derivative = (error - previousError) / dt;

            double output = kp * error + ki * integral + kd * derivative;

            // Clamp output
            output = Math.max(outputMin, Math.min(output, outputMax));

            previousError = error;
            return output;
        }

        public void reset() {
            integral = 0;
            previousError = 0;
        }

        // Optional: Set output limits
        public void setOutputLimits(double min, double max) {
            this.outputMin = min;
            this.outputMax = max;
        }
    }

    // Low-Pass Filter Class
    public class LowPassFilter {
        private double alpha;
        private double filteredValue;

        public LowPassFilter(double alpha) {
            this.alpha = alpha;
            this.filteredValue = 0;
        }

        public double filter(double value) {
            filteredValue = alpha * value + (1 - alpha) * filteredValue;
            return filteredValue;
        }

        public void reset(double value) {
            filteredValue = value;
        }

        public double get() {
            return filteredValue;
        }
    }

    // Rate Limiter Class
    public class RateLimiter {
        private double maxDelta;
        private double lastValue;

        public RateLimiter(double maxDelta) {
            this.maxDelta = maxDelta;
            this.lastValue = 0;
        }

        public double calculate(double target) {
            double delta = target - lastValue;
            if (delta > maxDelta) {
                delta = maxDelta;
            } else if (delta < -maxDelta) {
                delta = -maxDelta;
            }
            lastValue += delta;
            return lastValue;
        }

        public void reset(double value) {
            lastValue = value;
        }
    }
}
