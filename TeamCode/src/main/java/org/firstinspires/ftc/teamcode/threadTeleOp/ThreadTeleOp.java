package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.teleop.TeleLib;
//@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ThreadTeleOp extends ThreadTeleLib {

    private ElapsedTime loopTimer;

    @Override
    public void init() {
        super.init();
        loopTimer = new ElapsedTime();
    }

    @Override
    public void loop(){
        //loopTimer.reset();
        telem();
        ArcadeDrive();
        intake();
        horizSlides();
        intakeTilt();
        arm();
        claw();


        verticalSlides();

//        telemetry.addData("Cycle Time (ms)", loopTimer.milliseconds());
//        telemetry.update();
        //Claw();
    }

    @Override
    public void stop(){
        super.stop();
    }
}