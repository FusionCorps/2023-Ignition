package frc.robot.math;

import static java.lang.Math.*;

public class Tilt {
    public static double calculate(double pitch, double roll) {
        return toDegrees(acos(cos(toRadians(pitch)) * cos(toRadians(roll))));
    }
}
