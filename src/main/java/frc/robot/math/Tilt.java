package frc.robot.math;

import static java.lang.Math.*;

public class Tilt {
    public static double calculate(double yaw, double pitch, double roll) {
        double sign = 1;
        if (sin(toRadians(yaw)) * roll < 1 || cos(toRadians(pitch)) * pitch < 1)
            sign = -1;
        return sign * toDegrees(acos(cos(toRadians(pitch)) * cos(toRadians(roll))));
    }
}
