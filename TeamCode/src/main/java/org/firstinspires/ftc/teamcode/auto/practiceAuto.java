package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class practiceAuto extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    public DcMotor br;

    public DcMotor bl;

    public DcMotor fr;

    public DcMotor fl;

    @Override
    public void runOpMode(){
        telemetry.addData("Initialized", 1);
        telemetry.update();
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);



        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            while (timer.seconds() < 5) {
                bl.setPower(-.5);
                br.setPower(-.5);
                fr.setPower(-.5);
                fl.setPower(-.5);
            }
            bl.setPower(0);
            br.setPower(0);
            fr.setPower(0);
            fl.setPower(0);
            resetRuntime();
        }

    }
}