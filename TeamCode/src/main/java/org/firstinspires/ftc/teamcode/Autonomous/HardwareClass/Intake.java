package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;
import java.util.*;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public ColorRangeSensor colorSensor;
    public DcMotor intake;

    public void intake(LinearOpMode opMode){
        colorSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "colorSensor");
        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
    }
    public void checkColor(double sec){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (colorSensor.getDistance(DistanceUnit.CM) > 4 && time.seconds() < sec){
            intake.setPower(1);
            if (colorSensor.blue() > 150 && colorSensor.red() < 150 && colorSensor.getDistance(DistanceUnit.CM) < 4){
                intake.setPower(0);
                break;
            } else if (colorSensor.getDistance(DistanceUnit.CM) < 4){
                intake.setPower(-1);
            }
        }
    }

    public void backIntake(double sec){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        intake.setPower(-1);
    }
    public void inIntake(){
        intake.setPower(1);
    }
    public void reverseIntake(double sec){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        intake.setPower(-1);
    }
}
