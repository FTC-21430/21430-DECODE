package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

// This Autonomous is for when the robot starts in the far zone
@Autonomous
public class BlueDolphin extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true);
        robot.setAlliance("red");
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        while (opModeInInit()){
            int tempID = robot.aprilTags.getMotifID();
            if (tempID != 0) motifId = tempID;
            telemetry.addData("CurrentMotif", motifId);
            telemetry.update();
        }
        robot.setAlliance("red");
        //This is the starting location of the robot
        robot.odometry.overridePosition(64,-13,0);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.chill(false,0.2);

        robot.autoMoveTo(58,-13,15,4);
        // launch preloads
        robot.aimAtGoal();
        autonomousLaunching(motifId);

        //move off the line
        robot.autoMoveTo(40,-58,0,2);
        //end auto
    }
}
