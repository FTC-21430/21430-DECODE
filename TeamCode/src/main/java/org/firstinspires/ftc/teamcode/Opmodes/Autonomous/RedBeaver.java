package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class RedBeaver extends BaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true);
        robot.odometry.recalibrateIMU();

        waitForStart();
        //TODO; tune starting positon for actual robot
        //This is the starting location of the robot
        robot.odometry.overridePosition(-51,52,-45);

        

        //This is the position that the robot moves to to shoot the first three balls
        robot.autoMoveTo(-34,13,-45,2);
        //TODO: Figure out the shooting code with actual robot
        robot.launchFrom("close");


        //The robot moves to the place to intake the balls
        robot.intake.setIntakePower(1);
        detectMotifWhileMoveTo(-23,44,10,2);
        detectMotifWhileMoveTo(-5,46,90,2);

        //The robot bumps the gate
        detectMotifWhileMoveTo(-2,51,90,2);
        detectMotifWhileMoveTo(-2,55,90,2);
        robot.chill(true, 1);

        //The robot moves to the launch zone and it launches the three balls
        robot.autoMoveTo(2, 47, -450, 2);
        robot.launchFrom("mid");
        switch (motifId) {
            case 21:
                robot.spindexer.moveToNextIndex();
                for (int i = 0; i < 3; i++) {

                    robot.chill(false, 1.5);
                    robot.spindexer.eject();
                    robot.chill(false, 1.5);
                    robot.spindexer.moveToNextIndex();
                }
                break;
            case 22:
                for (int i = 0; i < 4; i++) {

                    robot.chill(false, 1);
                    robot.spindexer.eject();
                    robot.chill(false, 1.5);
                    robot.spindexer.moveToNextIndex();
                }
                break;
            case 23:
                robot.spindexer.moveToNextIndex();
                robot.spindexer.moveToNextIndex();
                robot.chill(false, 2);
                for (int i = 0; i < 3; i++) {

                    robot.chill(false, 1);
                    robot.spindexer.eject();
                    robot.chill(false, 1.5);
                    robot.spindexer.moveToNextIndex();
                }

                robot.autoMoveTo(-26, 11, -495, 2);

                robot.autoMoveTo(25, 47, -450, 2);

                robot.autoMoveTo(-23, 10, -495, 2);

                robot.autoMoveTo(-32, 54, -450, 2);


        }
    }
    }
