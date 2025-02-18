package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ThreadTeleOpBLUE extends ThreadTeleLibBLUE {
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
        //wrist();
        disableArm();



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
                clawSample();
                break;
            case SPECIMEN:
                verticalSlidesSpecimen();
                armSpecimen();
                clawSpecimen();
                break;
        }

        loopTime = loop;
    }

    @Override
    public void stop(){
        super.stop();
    }
}