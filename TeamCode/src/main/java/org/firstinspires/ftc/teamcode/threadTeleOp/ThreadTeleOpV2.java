package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.TeleLib;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ThreadTeleOpV2 extends ThreadTeleLibV2 {
    private boolean wasBumperPressed = false;

    private ElapsedTime loopTimer;
    private gameMode mode;
    double loopTime = 0.0;
    public enum gameMode {
        SAMPLE,
        SPECIMEN
    };
    @Override
    public void init() {
        super.init();
        loopTimer = new ElapsedTime();
        mode = gameMode.SPECIMEN;
    }

    @Override
    public void loop(){
        //loopTimer.reset();

        double loop = System.nanoTime();

        telemetry.addData("frequencyinHZ ", 1000000000 / (loop - loopTime));
        telemetry.addData("loopTimeinMS", (loop - loopTime) / 1000000);
        telemetry.addData("mode", mode);

        update();
        telem();
        ArcadeDrive();
        intake();
        horizSlides();
        intakeTilt();
        claw();
        //wrist();


        boolean isBumperPressed = gamepad2.right_bumper && gamepad2.left_bumper;

        if (isBumperPressed && !wasBumperPressed) { // Transition from unpressed to pressed
            if (mode == gameMode.SAMPLE) {
                mode = gameMode.SPECIMEN;
            } else if (mode == gameMode.SPECIMEN) {
                mode = gameMode.SAMPLE;
            }
        }

        // Update bumper state
        wasBumperPressed = isBumperPressed;

        // Handle mode-specific actions
        switch (mode) {
            case SAMPLE:
                verticalSlidesSample();
                armSample();
                break;
            case SPECIMEN:
                verticalSlidesSpecimen();
                armSpecimen();
                break;
        }

        loopTime = loop;
    }

    @Override
    public void stop(){
        super.stop();
    }
}