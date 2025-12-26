package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@Config
@TeleOp
public class SpindexerTesting extends BaseTeleOp {
    public static double pos = 0;
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
            if (gamepad1.crossWasPressed()){
                robot.spindexer.setSpindexerPos(pos);
            }

            if (gamepad1.rightBumperWasPressed()){
                robot.spindexer.eject();
            }
            if (gamepad2.crossWasPressed()){
                robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
            }
            if (gamepad2.circleWasPressed()){
                robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.INTAKE);
            }
            if (gamepad2.triangleWasPressed()){
                robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.LAUNCH);
            }
            robot.updateRobot(false,false,false);
            robot.operatorStateMachine.updateStateMachine();
            robot.bulkSensorBucket.clearCache();
            telemetry.update();
        }
    }
}
