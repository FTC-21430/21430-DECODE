package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LED {
    //Classes
    private Servo led = null;
    private ElapsedTime elapsedTime = null;
    private double lastTime;
    private double allianceColor;
    //Disco
    private boolean discoParty = false;
    //Colors
    public double color = 0.277;
    private final double RED = 0.28;
    private final double ORANGE = 0.333;
    private final double YELLOW = 0.388;
    private final double GREEN = 0.472;
    private final double BLUE = 0.611;
    public static double discoRate = 0.2;
    public LED(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "LED");
        this.elapsedTime = new ElapsedTime();
    }
    public void setLed(int numbOfArtifacts){
        if (discoParty) return;
        switch (numbOfArtifacts){
            case 0:
                setLedColor(RED);
                break;
            case 1:
                setLedColor(ORANGE);
                break;
            case 2:
                setLedColor(GREEN);
                break;
            default:
                setLedColor(RED);
        }

    }
    public void setLedColor(double color){
        led.setPosition(color);
    }
    public void update(){
        double deltaTime = elapsedTime.seconds()-lastTime;
        lastTime=elapsedTime.seconds();
        if (discoParty){
            color += discoRate*deltaTime;
            if (color>0.724) {
                color = 0.27;
            }
            setLedColor(color);
        }
    }
    public void discoParty(){
        discoParty = true;
        setLedColor(color);
    }
    public void setLedAlliance(){
        setLedColor(allianceColor);
    }
    public void setAllianceColor(String alliance){
        if (discoParty) return;
        if (alliance.equals("red")){
            allianceColor =RED;
        } else {
            allianceColor =BLUE;
        }
    }

}

