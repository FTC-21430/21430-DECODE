package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;

@Autonomous
public class PathFollowingTesting extends OpMode {
    @Override
    public void init() {
        ElapsedTime runtime = new ElapsedTime();
        PathFollowing p = new PathFollowing(0,0,0,0,0,0,runtime);
        p.setTargetPosition(10,10);
        p.setFollowSpeed(3);
    }
    public void loop(){

    }
}
