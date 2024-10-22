package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.acmerobotics.dashboard.config.Config;

//import org.firstinspires.ftc.teamcode.teleop.TeleLib;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ThreadTeleOp extends ThreadTeleLib {
    @Override
    public void loop(){
        ArcadeDrive();
        intake();
        horizSlides();
        intakeTilt();
        //verticalSlides();
        //Claw();
    }

    @Override
    public void stop(){
        stop();
    }
}