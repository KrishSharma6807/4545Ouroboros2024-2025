package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.TeleLib;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ThreadTeleOpV2 extends ThreadTeleLibV2 {

    private ElapsedTime loopTimer;
    double loopTime = 0.0;

    @Override
    public void init() {
        super.init();
        loopTimer = new ElapsedTime();
    }

    @Override
    public void loop(){
        //loopTimer.reset();

        double loop = System.nanoTime();

        telemetry.addData("frequencyinHZ ", 1000000000 / (loop - loopTime));
        telemetry.addData("loopTimeinMS", (loop - loopTime) / 1000000);

        update();
        telem();
        ArcadeDrive();
        intake();
        horizSlides();
        intakeTilt();
        arm();
        claw();


        verticalSlides();

        loopTime = loop;
    }

    @Override
    public void stop(){
        super.stop();
    }
}