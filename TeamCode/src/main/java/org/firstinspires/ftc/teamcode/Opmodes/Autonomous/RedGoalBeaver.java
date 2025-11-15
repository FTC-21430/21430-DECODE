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
        }
        robot.odometry.overridePosition(-51,51,110);

        robot.launchFrom("close");

        robot.autoMoveTo(-32,20,45,2);

        switch (motifId){
            case 21:
                robot.spindexer.moveToNextIndex();
                for (int i = 0; i < 3; i++){

                    while (!robot.spindexer.isAtRest()){
                        robot.chill(true,0.01);
                    }
                    robot.spindexer.eject();
                    robot.chill(true,0.3);
                    robot.spindexer.moveToNextIndex();
                }
                break;
            case 22:
                for (int i = 0; i < 3; i++){

                    while (!robot.spindexer.isAtRest()){
                        robot.chill(true,0.01);
                    }
                    robot.spindexer.eject();
                    robot.chill(true,0.3);
                    robot.spindexer.moveToNextIndex();
                }
                break;
            case 23:
                robot.spindexer.moveToNextIndex();
                robot.spindexer.moveToNextIndex();
                robot.chill(true,0.8);
                for (int i = 0; i < 3; i++){

                    while (!robot.spindexer.isAtRest()){
                        robot.chill(true,0.01);
                    }
                    robot.spindexer.eject();
                    robot.chill(true,0.3);
                    robot.spindexer.moveToNextIndex();
                }
                break;
        }
        robot.autoMoveTo(-58,14,0,2);


    }
}
