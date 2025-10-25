package org.firstinspires.ftc.teamcode.Firmware.Systems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

// Enable FTC Dashboard variable support
@Config
public class ColorSensorFirmware {

    private Telemetry telemetry = null;
    private NormalizedColorSensor sensor = null;

    private float[] hsvValues = new float[3];

//    TODO: Remove public static term currently in use for FTC Dashboard tuning - Tobin 10/18/2025

    public static float[] noneValues = new float[3];
    public static float[] purpleValues = new float[3];
    public static float[] greenValues = new float[3];

    public enum COLORS{
        NONE,
        PURPLE,
        GREEN,
    }
    public ColorSensorFirmware(HardwareMap hardwareMap, Telemetry telemetry, String configName, float sensorGain){
        sensor = hardwareMap.get(NormalizedColorSensor.class, configName);
        this.telemetry = telemetry;
        sensor.setGain(sensorGain);
    }



    public COLORS getDetectedColor(){
        updateSensor();
        ArrayList<Double> distances = getColorDistances();
        int lowestIndex = 0;
        double lowestDistance = 0;
        for (int i = 0; i < 3; i++){
            if (i==0){
                lowestIndex = i;
                lowestDistance = distances.get(i);
            }else{
                if (Math.abs(distances.get(i)) < lowestDistance){
                    lowestIndex = i;
                    lowestDistance = Math.abs(distances.get(i));
                }
            }
            telemetry.addData("distance " + i,distances.get(i));
        }

        switch (lowestIndex){
            case 0:
                return COLORS.NONE;
            case 1:
                return COLORS.PURPLE;
            case 2:
                return COLORS.GREEN;
        }
//        If index is out of range
        return COLORS.NONE;
    }
    public float[] getRawData(){
        updateSensor();
        return hsvValues;
    }

    public void setGain(float gain){
        sensor.setGain(gain);
    }

    private void updateSensor(){
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
    }

    private ArrayList<Double> getColorDistances(){
        ArrayList<Double> returnValues = new ArrayList<Double>();

//        Start with none colors
        double noneDistance = 0;
        noneDistance = hsvValues[0] - noneValues[0];
        returnValues.add(noneDistance);

        double purpleDistance = 0;
        purpleDistance = hsvValues[0] - purpleValues[0];
        returnValues.add(purpleDistance);

        double greenDistance = 0;
        greenDistance = hsvValues[0] - greenValues[0];
        returnValues.add(greenDistance);

        return returnValues;
    }
}
