package org.firstinspires.ftc.teamcode.robotControl;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
public class CustomPID {
    public Timer timer = new Timer();

    public double previousError = 0;

    public double integralSum = 0;

    public double derivative = 0;

    public double kP = 0;

    public double kI = 0;

    public double kD = 0;

    public double kF = 0;

    public boolean reset = false;

    public double lowPass = 0;
    public double a = 0.1;

    public CustomPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double calculatePower(double desiredPos, double currentPos){
        double internalTimer = getTimeRan();

        lowPass = a * currentPos + (1 - a) * lowPass;
        double error = desiredPos - lowPass;
        integralSum =  error * timer.currentTime();
        derivative = (error - previousError) / internalTimer;

        double power = kP * error + kI * integralSum + kD * derivative + kF;
        return power;
    }

    public double getTimeRan() {
        if (!reset) {
            reset = true;
            timer.reset();
        }
        double dt = timer.currentTime();
        timer.reset();
        return dt;
    }
}
