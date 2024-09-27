package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Teleop.ThreadTeleLib;

@TeleOp

public class ThreadTeleOp extends ThreadTeleLib {
    @Override
    public void loop() {
        // add your threads here
    }
    @Override
    public void kill(){
        kill();
    }
}