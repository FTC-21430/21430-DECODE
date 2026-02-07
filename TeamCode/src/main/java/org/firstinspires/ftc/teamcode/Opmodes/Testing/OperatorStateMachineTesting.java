package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class OperatorStateMachineTesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false,false);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.triangleWasPressed()){
                robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
            }
            if (gamepad1.circleWasPressed()){
                robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
            }
            if (gamepad1.crossWasPressed()){
                robot.operatorStateMachine.moveToState(OperatorStateMachine.State.IDLE);
            }
            if (gamepad1.dpadDownWasPressed()){
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
            }
            if (gamepad1.dpadLeftWasPressed()){
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
            }
            if (gamepad1.dpadRightWasPressed()){
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
            }
            robot.operatorStateMachine.updateStateMachine();
            telemetry.update();
            robot.updateRobot(false,false,false);
            robot.bulkSensorBucket.clearCache();
        }
    }
}
