package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    public ColorRangeSensor colorSensor;
    public void init(){
        ///this.opMode = opMode;
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");
    }
    public void test(){
        telemetry.addData("DISTANCE :: ", colorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("COLOR :: ", colorSensor.getNormalizedColors());
        telemetry.update();
    }
}
