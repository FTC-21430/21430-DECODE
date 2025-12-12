package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class RedBeaver extends BaseAuto {
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
            //This is the starting location of the robot
            robot.autoMoveTo(-29,13,-495,2);

            //This is the position that the robot moves to to shoot the first three balls
            robot.autoMoveTo(-23,24,-135,2);
            robot.launchFrom("close");

            //The robot moves to the place to intake the balls
            robot.autoMoveTo(-22,47,-450,2);
            robot.aprilTags.getMotifID();
            robot.intake.setIntakePower(1);

            //The robot moves to bump the gate
            robot.autoMoveTo(-28,12,-495,2);
            robot.chill(true,1);





            robot.autoMoveTo(2,47,-450,2);

            robot.autoMoveTo(-26,11,-495,2);

            robot.autoMoveTo(25,47,-450,2);

            robot.autoMoveTo(-23,10,-495,2);

            robot.autoMoveTo(-32,54,-450,2);
            


        }
    }
}
