package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class practiceAuto extends LinearOpMode {
    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;

    br = hardwareMap.get(DcMotor.class, "br");
    fr = hardwareMap.get(DcMotor.class, "fr");
    fl = hardwareMap.get(DcMotor.class, "fl");
    bl = hardwareMap.get(DcMotor.class, "bl");

    @Override
}
