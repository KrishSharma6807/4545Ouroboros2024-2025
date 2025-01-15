package org.firstinspires.ftc.teamcode.Geom;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.*;

public class Pose {

    public double heading;
    public double x;
    public double y;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = AngleUnit.normalizeRadians(heading);
    }

    public Pose() {
        this(0, 0, 0);
    }

    public void setPose(Pose inp) {
        this.x = inp.x;
        this.y = inp.y;
        this.heading = inp.heading;
    }
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.3f", x, y, heading);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public Pose clone() {
        return new Pose(x, y, heading);
    }
}
