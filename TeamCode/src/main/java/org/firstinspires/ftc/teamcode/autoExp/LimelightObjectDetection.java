//package org.firstinspires.ftc.teamcode.autoExp;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//
//@TeleOp(name = "Limelight Object Detection", group = "TeleOp")
//public class LimelightObjectDetection extends LinearOpMode {
//
//    // Limelight NetworkTable entries
//    private NetworkTable limelightTable;
//    private NetworkTableEntry tx, ty, ta, tv;
//
//    @Override
//    public void runOpMode() {
//        // Initialize Limelight NetworkTables
//        initializeLimelight();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Get Limelight data
//            double xOffset = tx.getDouble(0.0);  // Horizontal offset from crosshair to target (-27 to 27 degrees)
//            double yOffset = ty.getDouble(0.0);  // Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
//            double targetArea = ta.getDouble(0.0); // Target area (% of image)
//            double targetVisible = tv.getDouble(0.0); // Whether the target is visible (0 or 1)
//
//            if (targetVisible == 1.0) {
//                telemetry.addData("Target", "Visible");
//                telemetry.addData("X Offset", xOffset);
//                telemetry.addData("Y Offset", yOffset);
//                telemetry.addData("Target Area", targetArea);
//            } else {
//                telemetry.addData("Target", "Not Visible");
//            }
//
//            telemetry.update();
//
//            // Use the xOffset, yOffset, and targetArea to control the robot
//            alignToTarget(xOffset, yOffset, targetArea);
//        }
//    }
//
//    private void initializeLimelight() {
//        NetworkTableInstance inst = NetworkTableInstance.getDefault();
//        limelightTable = inst.getTable("limelight");
//        tx = limelightTable.getEntry("tx");
//        ty = limelightTable.getEntry("ty");
//        ta = limelightTable.getEntry("ta");
//        tv = limelightTable.getEntry("tv");
//    }
//
//    private void alignToTarget(double xOffset, double yOffset, double targetArea) {
//        // Simple alignment logic (can be replaced with PID control or other algorithms)
//        double alignmentThreshold = 1.0; // Tolerance for alignment
//
//        if (Math.abs(xOffset) > alignmentThreshold) {
//            if (xOffset > 0) {
//                // Rotate right to reduce xOffset
//                telemetry.addData("Alignment", "Rotating Right");
//            } else {
//                // Rotate left to reduce xOffset
//                telemetry.addData("Alignment", "Rotating Left");
//            }
//        } else {
//            telemetry.addData("Alignment", "Aligned Horizontally");
//        }
//
//        // Example: Adjust drive power based on distance
//        if (targetArea < 10.0) {
//            telemetry.addData("Alignment", "Moving Closer");
//        } else if (targetArea > 20.0) {
//            telemetry.addData("Alignment", "Backing Away");
//        } else {
//            telemetry.addData("Alignment", "Proper Distance");
//        }
//    }
//}
