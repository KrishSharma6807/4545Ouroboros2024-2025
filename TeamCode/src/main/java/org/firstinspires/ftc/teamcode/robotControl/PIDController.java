package org.firstinspires.ftc.teamcode.robotControl;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double kP, kI, kD, feedforward, correction = 0, integralSum = 0, maxIntSum = 0;
    public boolean wasIntegralReset = false;
    private int integralLength;

    private Derivative derivative = new Derivative(3);
    private RingBuffer<Double> integralBuffer;



    public PIDController(int integralLength){ this(0,0,0, integralLength); }

    public PIDController(){ this(0, 0, 0, 0); }

    public PIDController(double proportional, double integral, double derivative, int integralLength) {
        kP = proportional; kI = integral; kD = derivative; this.integralLength = integralLength;
        integralBuffer = new RingBuffer<Double>(integralLength, 0.0);
    }


    public void setConstants(double kP, double kI, double kD){
        this.kP = kP; this.kI = kI; this.kD = kD;
    }

    public void setFComponent(double fComponent){
        this.feedforward = fComponent;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double update(double error){
        derivative.update(System.currentTimeMillis() / 1000.0, error);

        if(!wasIntegralReset) {
            integralSum += error;
            if (integralLength != 0) integralSum -= integralBuffer.getValue(error);
        }

        double pComponent = error * kP;
        double iComponent = Range.clip(integralSum * kI, -maxIntSum, maxIntSum);
        double dComponent = (derivative.getDerivative() * kD);

        wasIntegralReset = false;
        return correction = pComponent + iComponent + dComponent + feedforward;
    }

    public double getCorrection() {
        return correction;
    }



    public void resetDerivative(){ derivative.resetX(); }

    public void setMaxIntSum(double maxIntSum){
        this.maxIntSum = maxIntSum;
    }

    public void setIntegralSum(double integralSum){
        this.integralSum = integralSum;
        wasIntegralReset = true;
    }
}
