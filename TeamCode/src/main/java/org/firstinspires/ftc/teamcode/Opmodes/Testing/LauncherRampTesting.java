package org.firstinspires.ftc.teamcode.Opmodes.Testing;

// Written by Tobin - 11/6

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

// uncomment Config to use FTC dashboard
//@Config
//comment disabled to be able to run this test code
@Disabled

// Used for testing the functionality of the launcher ramp
@TeleOp
public class LauncherRampTesting extends BaseTeleOp {

    Launcher launcher;

    public static double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Init launcher
        launcher = new Launcher(hardwareMap, telemetry);
        // We don't want the flywheel running right now
        launcher.setSpeed(0);

        waitForStart();
        while (opModeIsActive()){

            // Use buttons for each preset and press triangle to push the FTC dashboard tuning value to the mechanism.
            if (gamepad1.squareWasPressed()){
                launcher.retractRamp();
            }
            if (gamepad1.circleWasPressed()){
                launcher.fullyExtendRamp();
            }
            if (gamepad1.crossWasPressed()){
                launcher.halfExtendRamp();
            }
            if (gamepad1.triangleWasPressed()){
                launcher.setLaunchAngle(angle);
            }
            // update the launcher speed control so it does feel neglected.
            launcher.updateSpeedControl();
        }
    }
}
