package org.firstinspires.ftc.teamcode.threadTeleOp;

//import org.firstinspires.ftc.teamcode.teleop.TeleLib;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ThreadTeleOp extends ThreadTeleLib {
    @Override
    public void loop(){
        ArcadeDrive();
        intake();
        horizSlides();
        //Claw();
    }

    @Override
    public void stop(){
        stop();
    }
}