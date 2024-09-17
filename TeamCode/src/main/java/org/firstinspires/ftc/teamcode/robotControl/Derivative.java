package org.firstinspires.ftc.teamcode.robotControl;

public  class Derivative {
    RingBuffer<Double> xRing;
    RingBuffer<Double> yRing;
    double derivative = 0;

    public Derivative(int length){
        xRing = new RingBuffer<>(length,0.0);
        yRing = new RingBuffer<>(length,0.0);
    }

    public void update(double x, double y){
        double deltaX = x - xRing.getValue(x);
        double deltaY = y - yRing.getValue(y);
        derivative = deltaY/deltaX;
    }

    public double getDerivative(){
        return derivative;
    }

    public void resetX(){
        xRing.overwriteValues(0.0);
    }

    public void resetY(){
        yRing.overwriteValues(0.0);
    }
}