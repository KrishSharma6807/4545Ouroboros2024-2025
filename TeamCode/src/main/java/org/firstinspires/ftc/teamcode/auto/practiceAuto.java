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
public abstract class practiceAuto extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    public DcMotor br;

    public DcMotor bl;

    public DcMotor fr;

    public DcMotor fl;

    @Override
    public void runOpMode() throws InterruptedException {
        timer.reset();
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("here", 1);
        telemetry.update();

        waitForStart();
        if (!isStopRequested()) {
            while (opModeIsActive() && timer.seconds() < 3) {
                bl.setPower(1);
                br.setPower(1);
                fr.setPower(1);
                fl.setPower(1);
            }
            bl.setPower(0);
            br.setPower(0);
            fr.setPower(0);
            fl.setPower(0);
            resetRuntime();
        }

    }
}