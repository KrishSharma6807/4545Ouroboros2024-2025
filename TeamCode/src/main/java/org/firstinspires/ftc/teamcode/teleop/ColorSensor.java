package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

// Add these imports for FTC Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Color Sensor TeleOp", group = "Demo")
@Config
public class ColorSensor extends LinearOpMode {
    private NormalizedColorSensor colorSensor;
    private FtcDashboard dashboard;
    public static double gain = 4;

    public Servo intakeTiltLeft;
    public Servo intakeTiltRight;

    public static double intakeTiltLeftUp = .4;
    public static double intakeTiltRightUp = .02;

    public static double intakeTiltLeftDown = .2;
    public static double intakeTiltRightDown = .37;
    @Override
    public void runOpMode() {
        // Initialize the sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain((float)gain); // Optional: Adjust gain
        intakeTiltLeft = hardwareMap.get(Servo.class, "intakeTiltLeft");
        intakeTiltRight = hardwareMap.get(Servo.class, "intakeTiltRight");
        // Initialize FTC Dashboard telemetry
        dashboard = FtcDashboard.getInstance();

        waitForStart();
        intakeTiltLeft.setPosition(intakeTiltLeftDown);
        intakeTiltRight.setPosition(intakeTiltRightDown);
        while (opModeIsActive()) {
            // Read sensor data
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsv = new float[3];
            Color.colorToHSV(colors.toColor(), hsv);
            String detectedColor = classifyColor(hsv[0], hsv[1], hsv[2]);

            // Send to Driver Station
            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Hue", "%.1f", hsv[0]);
            telemetry.addData("Sat", "%.2f", hsv[1]);
            telemetry.addData("Val", "%.2f", hsv[2]);
            telemetry.update();

            // Send to FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Color", detectedColor);
            packet.put("Hue", hsv[0]);
            packet.put("Saturation", hsv[1]);
            packet.put("Value", hsv[2]);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private String classifyColor(float hue, float saturation, float value) {
        // (Same classification logic as before)
        if (saturation < 0.3) return "unknown";
        if ((hue >= 0 && hue <= 64) || (hue >= 330 && hue <= 360)) return "red";
        else if (hue >= 200 && hue <= 260) return "blue";
        else if (hue >= 65 && hue <= 100) return "yellow";
        return "unknown";
    }
}