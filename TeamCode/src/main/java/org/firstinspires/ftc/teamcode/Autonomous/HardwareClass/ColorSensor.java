package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor{
    public ColorRangeSensor colorSensor;
    public DcMotor intake;

    public void init(){
        ///this.opMode = opMode;
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");
        intake = hardwareMap.get(DcMotor.class, "intake");
    }
    public void test(){
        telemetry.addData("DISTANCE :: ", colorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("COLOR :: ", colorSensor.getNormalizedColors());
        telemetry.update();
    }

    public void checkColor(double sec){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        if (colorSensor.blue() > 150 && colorSensor.red() < 150 && colorSensor.getDistance(DistanceUnit.CM) < 4){
            intake.setPower(0);

        } else if (colorSensor.getDistance(DistanceUnit.CM) < 4 && colorSensor.blue() < 150 && colorSensor.red() > 150){
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

}
