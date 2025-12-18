package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

public class LimitSwitchTesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,false);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("spindexerSwitch", robot.spindexer.getIntakeSwitch());
            telemetry.update();
        }
    }
}
