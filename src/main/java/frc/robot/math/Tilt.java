package frc.robot.math;

import static java.lang.Math.*;
// In theory, this should calculate tilt of robot on charging station.
// Needs testing.
public class Tilt {
    public static double calculate(double pitch, double roll) {
        double y = pow(-1, floor(roll / 180));
        if (pitch == 90 || pitch == 270) {
            y = 0;
        } else if (roll == 90 || roll == 270) {
            y *= sqrt(1 / (pow(tan(roll), 2) + 1));
        } else {
            y *= sqrt(1 / (pow(tan(pitch), 2) + 1 / (pow(tan(roll), 2) +  1)));
        }
        return asin(y);
    }
}
