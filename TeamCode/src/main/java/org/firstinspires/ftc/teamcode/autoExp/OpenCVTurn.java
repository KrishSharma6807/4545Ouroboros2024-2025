package org.firstinspires.ftc.teamcode.autoExp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpenCV Object Tracking")
@Config
public class OpenCVTurn extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;
    boolean objectDetected = false;

    // Camera parameters
    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;
    private static final double CAMERA_CENTER_X = CAMERA_WIDTH / 2.0;

    // PID control parameters
    public static double TURN_P = 0.002;  // Proportional control constant for turning
    public static double TURN_D = 0.0001; // Derivative control for dampening
    public static double FACING_TOLERANCE = 15;  // Tolerance in pixels

    // Time tracking
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0;
    private double currentError = 0;

    // Robot control
    private PackagedP2P robotDrive;

    @Override
    public void runOpMode() {
        // Initialize the camera
        initOpenCV();

        // Initialize the drive system
        robotDrive = new PackagedP2P(hardwareMap);

        // Set up telemetry dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        pidTimer.reset();

        while (opModeIsActive()) {
            // Get current pose for tracking
            Pose2d currentPose = robotDrive.getPoseEstimate();

            // Check if we've detected an object
            if (objectDetected) {
                // Calculate how far off-center the object is (positive = object is to the right)
                currentError = cX - CAMERA_CENTER_X;

                // Calculate derivative term
                double deltaTime = pidTimer.seconds();
                double errorRate = (currentError - lastError) / deltaTime;
                pidTimer.reset();

                // If the object is within our tolerance, we're already facing it
                if (Math.abs(currentError) <= FACING_TOLERANCE) {
                    telemetry.addData("Status", "Facing object");
                    // Stop the robot
                    robotDrive.stopMotors();
                } else {
                    // Calculate the rotation amount based on a simple PD controller
                    // We negate the error because if object is to the right (positive error),
                    // we need to rotate clockwise (negative heading change)
                    double turnAdjustment = (-currentError * TURN_P) - (errorRate * TURN_D);

                    // Set the target position with our current position but adjusted heading
                    // Make sure to work with the current heading from odometry
                    double currentHeading = currentPose.heading.toDouble();
                    double targetHeading = currentHeading + turnAdjustment;

                    // Set the target to our current position but with adjusted heading
                    robotDrive.setTarget(
                            currentPose.position.x,
                            currentPose.position.y,
                            targetHeading
                    );

                    // Use runToPosition to execute the movement
                    telemetry.addData("Status", "Turning to face object");
                    telemetry.addData("Current Heading", Math.toDegrees(currentHeading));
                    telemetry.addData("Target Heading", Math.toDegrees(targetHeading));
                    telemetry.addData("Turn Adjustment", turnAdjustment);

                    // Run a single iteration of movement
                    robotDrive.runToPositionIterative();
                }

                // Save error for next derivative calculation
                lastError = currentError;

            } else {
                telemetry.addData("Status", "No object detected");
                // Stop the robot if no object is detected
                robotDrive.stopMotors();
            }

            // Display object information
            telemetry.addData("Object Detected", objectDetected);
            telemetry.addData("Object Position", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Object Width", width + " pixels");
            telemetry.addData("Error from center", currentError);
            telemetry.update();

            // Short delay to prevent CPU overload
            sleep(10);
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {
        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        // Thresholds for detection
        public static final double MIN_CONTOUR_AREA = 200; // Minimum area to consider a contour

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            // Reset detection flag
            objectDetected = false;

            if (largestContour != null) {
                double area = Imgproc.contourArea(largestContour);

                // Only consider it detected if it's large enough
                if (area >= MIN_CONTOUR_AREA) {
                    // Set detection flag
                    objectDetected = true;

                    // Draw a red outline around the largest detected object
                    Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

                    // Calculate the width of the bounding box
                    Rect boundingRect = Imgproc.boundingRect(largestContour);
                    width = boundingRect.width;

                    // Calculate the centroid of the largest contour
                    Moments moments = Imgproc.moments(largestContour);
                    cX = moments.get_m10() / moments.get_m00();
                    cY = moments.get_m01() / moments.get_m00();

                    // Draw visual feedback on the camera feed
                    // Display the width
                    String widthLabel = "Width: " + (int) width + " pixels";
                    Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                    // Display coordinates
                    String label = "(" + (int) cX + ", " + (int) cY + ")";
                    Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                    // Draw a circle at the centroid
                    Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

                    // Draw a line from the center of the screen to the object
                    Imgproc.line(input, new Point(CAMERA_WIDTH / 2, CAMERA_HEIGHT / 2),
                            new Point(cX, cY), new Scalar(255, 0, 255), 2);

                    // Calculate and display the error from center
                    double errorX = cX - CAMERA_CENTER_X;
                    String errorLabel = "Error X: " + String.format("%.1f", errorX);
                    Imgproc.putText(input, errorLabel, new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 2);

                    // Draw horizontal line at screen center for reference
                    Imgproc.line(input, new Point(0, CAMERA_HEIGHT / 2),
                            new Point(CAMERA_WIDTH, CAMERA_HEIGHT / 2), new Scalar(128, 128, 128), 1);

                    // Draw vertical line at screen center for reference
                    Imgproc.line(input, new Point(CAMERA_WIDTH / 2, 0),
                            new Point(CAMERA_WIDTH / 2, CAMERA_HEIGHT), new Scalar(128, 128, 128), 1);
                }
            }

            // Clean up
            yellowMask.release();
            hierarchy.release();

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // HSV range for yellow objects - adjust these values based on your specific object
            // These values are for detecting pink/magenta objects
            Scalar lowerYellow = new Scalar(150, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);

            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            hsvFrame.release();
            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
    }
}