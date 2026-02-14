package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@Config
@TeleOp
@Disabled
public class SpindexerTesting extends BaseTeleOp {
    public static double pos = 0;
    private Spindexer spindexer;
    @Override

    public void runOpMode() throws InterruptedException {
//        initialize(true,false);
        spindexer = new Spindexer(hardwareMap,telemetry,true,false);
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.circleWasPressed()){
                spindexer.moveToNextIndex();
            }
            if (gamepad1.triangleWasPressed()){
                spindexer.recalibrateSpindexerPosition();
            }
            if (gamepad1.squareWasPressed()){
                spindexer.eject();
            }
            if (gamepad1.crossWasPressed()){
                spindexer.setSpindexerPos(pos);
            }

            if (gamepad1.rightBumperWasPressed()){
                spindexer.eject();
            }
            if (gamepad2.crossWasPressed()){
                spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
            }
            if (gamepad2.circleWasPressed()){
                spindexer.setIndexOffset(Spindexer.INDEX_TYPE.INTAKE);
            }
            if (gamepad2.triangleWasPressed()){
                spindexer.setIndexOffset(Spindexer.INDEX_TYPE.LAUNCH);
            }
            spindexer.updateSpindexer();
            telemetry.update();
        }
    }
}
