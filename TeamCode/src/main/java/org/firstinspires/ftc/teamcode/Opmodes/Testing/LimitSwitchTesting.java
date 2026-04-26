package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@Config
@TeleOp
@Disabled

public class LimitSwitchTesting extends BaseTeleOp {

    public static String configAddress = "intakeLimitSwitchOne";

    private DigitalChannel LiftLimitSwitch1;
    private DigitalChannel LiftLimitSwitch2;

    @Override
    public void runOpMode() throws InterruptedException {
        LiftLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch1");
        LiftLimitSwitch2 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch2");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("spindexerSwitch", robot.spindexer.getIntakeSwitch());
            telemetry.addData("liftSwitch1", LiftLimitSwitch1.getState());
            telemetry.addData("liftSwitch2", LiftLimitSwitch2.getState());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }

    }
}
