package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous()
public class AutonomousWithoutAprilTag extends LinearOpMode {

    @Override
    public void runOpMode() {
        util core = new util();
        core.init(hardwareMap);
        telemetry.addLine("Waiting for start");
        telemetry.addLine("Make sure you stop after because I don't make the wheels stop in this code");
        updateTelemetry(telemetry);
        waitForStart();
        while(opModeIsActive()){
            core.frontLeftDrive.setPower(0.2);
            core.frontRightDrive.setPower(0.2);
            core.backLeftDrive.setPower(0.2);
            core.backRightDrive.setPower(0.2);

        }
    }
}
