package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Firmware.Robot;
import org.firstinspires.ftc.teamcode.Robot.Robot;

// This class will have the autonomous functions applicable to every auto. All autos extend BaseAuto.
@Config
abstract public class BaseAuto extends org.firstinspires.ftc.teamcode.Opmodes.GeneralOpMode {
    public void setAutoSpeedSlow(){
        Robot robot = null;
        robot.setRobotSpeed(Robot.Speed.SLOW);
    }
    public void setAutoSpeedFast(){
        robot.setRobotSpeed(Robot.Speed.FAST);
    }
    
}
