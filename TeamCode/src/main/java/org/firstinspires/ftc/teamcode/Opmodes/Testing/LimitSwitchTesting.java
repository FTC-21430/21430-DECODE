package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@Config
@TeleOp
public class LimitSwitchTesting extends BaseTeleOp {

    public static String configAddress = "intakeLimitSwitchOne";

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false,false);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("spindexerSwitch", robot.spindexer.getIntakeSwitch());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }

    }
}
