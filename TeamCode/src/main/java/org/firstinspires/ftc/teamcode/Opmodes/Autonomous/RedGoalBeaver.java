package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

@Autonomous
public class RedGoalBeaver extends BaseAuto {

    private int motifId = 21;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true);

        while (opModeInInit()){
            int tempId = robot.aprilTags.getMotifID();
            if (tempId != 0){
                motifId = tempId;
            }
            telemetry.addData("motif", motifId);
            telemetry.update();
        }
        robot.odometry.overridePosition(-58,44,-175);
        robot.launchFrom("close");

//        robot.autoMoveTo(-39,16,35,2);
        robot.driveTrain.setDrivePower(-0.4, 0, 0, 0);

//        robot.chill(false,1);
        robot.driveTrain.setDrivePower(0, 0, 0, 0);

        switch (motifId){
            case 21:
                robot.spindexer.moveToNextIndex();
                for (int i = 0; i < 3; i++){

                    robot.chill(false,1.5);
                    robot.spindexer.eject();
                    robot.chill(false,1.5);
                    robot.spindexer.moveToNextIndex();
                }
                break;
            case 22:
                for (int i = 0; i < 4; i++){

                    robot.chill(false,1);
                    robot.spindexer.eject();
                    robot.chill(false,1.5);
                    robot.spindexer.moveToNextIndex();
                }
                break;
            case 23:
                robot.spindexer.moveToNextIndex();
                robot.spindexer.moveToNextIndex();
                robot.chill(false,2);
                for (int i = 0; i < 3; i++){

                    robot.chill(false,1);
                    robot.spindexer.eject();
                    robot.chill(false,1.5);
                    robot.spindexer.moveToNextIndex();
                }
                break;
        }
//        robot.autoMoveTo(-58,14,180,2);
    }
}
