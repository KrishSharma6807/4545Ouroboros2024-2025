package org.firstinspires.ftc.teamcode.teleop;

public class TeleOp extends TeleLib{
    @Override
    public void loop(){
        ArcadeDrive();
        horizSlides();
        intake();
        Claw();
    }

    @Override
    public void kill(){
        kill();
    }
}
