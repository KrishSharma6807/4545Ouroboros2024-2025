package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;
import java.util.*;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public ColorRangeSensor colorSensor;
    public DcMotor intake;

    public Servo intakeTiltLeft;
    public Servo intakeTiltRight;

    public void intake(LinearOpMode opMode){
        colorSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "colorSensor");
        intake = opMode.hardwareMap.get(DcMotor.class, "intake");

        intakeTiltLeft = opMode.hardwareMap.get(Servo.class, "intakeTiltLeft");
        intakeTiltRight = opMode.hardwareMap.get(Servo.class, "intakeTiltRight");
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
