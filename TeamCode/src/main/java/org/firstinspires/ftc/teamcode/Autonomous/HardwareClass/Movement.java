package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Movement {

    HardwareMap robot;
    public DcMotor bl;
    public DcMotor br;
    public DcMotor fl;
    public DcMotor fr;

    public Movement() {

        bl = hardwareMap.get(DcMotor.class , "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class,"fr");

    }

}
