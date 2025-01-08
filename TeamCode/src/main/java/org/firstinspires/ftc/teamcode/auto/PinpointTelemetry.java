//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.dashboard.DashboardCore;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.auto.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import java.util.Locale;
//
//public class PinpointTelemetry extends LinearOpMode {
//
//    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
//
//    double oldTime = 0;
//
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize the hardware variables. Note that the strings used here must correspond
//        // to the names assigned during the robot configuration step on the DS or RC devices.
//
//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
//
//        /*
//        Set the odometry pod positions relative to the point that the odometry computer tracks around.
//        The X pod offset refers to how far sideways from the tracking point the
//        X (forward) odometry pod is. Left of the center is a positive number,
//        right of center is a negative number. the Y pod offset refers to how far forwards from
//        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
//        backwards is a negative number.
//         */
//        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
//
//        /*
//        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
//        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
//        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
//        number of ticks per mm of your odometry pod.
//         */
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        //odo.setEncoderResolution(13.26291192);
//
//
//        /*
//        Set the direction that each of the two odometry pods count. The X (forward) pod should
//        increase when you move the robot forward. And the Y (strafe) pod should increase when
//        you move the robot to the left.
//         */
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//
//        /*
//        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
//        The IMU will automatically calibrate when first powered on, but recalibrating before running
//        the robot is a good idea to ensure that the calibration is "good".
//        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
//        This is recommended before you run your autonomous, as a bad initial calibration can cause
//        an incorrect starting value for x, y, and heading.
//         */
//        //odo.recalibrateIMU();
//        odo.resetPosAndIMU();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", odo.getXOffset());
//        telemetry.addData("Y offset", odo.getYOffset());
//        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
//        telemetry.addData("Device Scalar", odo.getYawScalar());
//        telemetry.update();
//
//        // Wait for the game to start (driver presses START)
//        waitForStart();
//        resetRuntime();
//
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            /*
//            Request a bulk update from the Pinpoint odometry computer. This checks almost all outputs
//            from the device in a single I2C read.
//             */
//            odo.update();
//
//
//            if (gamepad1.a){
//                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
//            }
//
//            if (gamepad1.b){
//                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
//            }
//
//            /*
//            This code prints the loop frequency of the REV Control Hub. This frequency is effected
//            by I2C reads/writes. So it's good to keep an eye on. This code calculates the amount
//            of time each cycle takes and finds the frequency (number of updates per second) from
//            that cycle time.
//             */
//            double newTime = getRuntime();
//            double loopTime = newTime-oldTime;
//            double frequency = 1/loopTime;
//            oldTime = newTime;
//
//
//            /*
//            gets the current Position (x & y in inches, and heading in radians) of the robot, and prints it.
//             */
//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//
//
//            /*
//            gets the current Velocity (x & y in inches/sec and heading in radians/sec) and prints it.
//             */
//            Pose2D vel = odo.getVelocity();
//
//            telemetry.addData("X Encoder:", odo.getEncoderX()); //gets the raw data from the X encoder
//            telemetry.addData("Y Encoder:",odo.getEncoderY()); //gets the raw data from the Y encoder
//            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
//
//            /*
//            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
//            READY: the device is working as normal
//            CALIBRATING: the device is calibrating and outputs are put on hold
//            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
//            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
//            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
//            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
//            */
//            telemetry.addData("Status", odo.getDeviceStatus());
//
//            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
//            telemetry.update();
//        }
//    }
//    public void drawPose(Pose pose, String color, double radius) {
//
//        fieldOverlay.setStroke(color);
//
//        fieldOverlay.strokeCircle(pose.x, pose.y, radius);
//
//        fieldOverlay.strokeLine(pose.x, pose.y, pose.x + Math.cos(pose.heading) * radius, pose.y + Math.sin(pose.heading) * radius);
//
//    }
//}
