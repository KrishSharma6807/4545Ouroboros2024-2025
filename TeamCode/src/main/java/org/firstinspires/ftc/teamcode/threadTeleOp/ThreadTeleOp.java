package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.teleop.TeleLib;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ThreadTeleOp extends ThreadTeleLib {



    @Override
    public void loop(){
        telem();
        ArcadeDrive();
        intake();
        horizSlides();
        intakeTilt();
        arm();
        claw();


        verticalSlides();
        //Claw();
    }

    @Override
    public void stop(){
        stop();
    }
}