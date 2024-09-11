package org.firstinspires.ftc.teamcode.teleop;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends TeleLib{
    @Override
    public void loop(){
        ArcadeDrive();
        //horizSlides();
        //intake();
        //Claw();
    }

    @Override
    public void kill(){
        kill();
    }
}
