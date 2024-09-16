package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Movement {

    HardwareMap robot;
    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;

    public Movement(HardwareMap hardwareMap){

        this.robot = hardwareMap;

        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void motorPower(double p1, double p2, double p3, double p4) {

        fl.setPower(p1);
        bl.setPower(p2);
        br.setPower(p3);
        fr.setPower(p4);

    }
    public void kill() {

        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);

    }
}