package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

public class SpindexerTesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,false);
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.circleWasPressed()){
                robot.spindexer.moveToNextIndex();
            }
            if (gamepad1.triangleWasPressed()){
                robot.spindexer.recalibrateSpindexerPosition();
            }
            if (gamepad1.squareWasPressed()){
                robot.spindexer.eject();
            }
            robot.updateRobot(false,false,false);
            robot.bulkSensorBucket.clearCache();
        }
    }
}
