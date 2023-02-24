package frc.robot.math;

import static java.lang.Math.*;

public class Tilt {
    public static final double YAW_DANGER_ZONE_MARGIN = 45;
    public static double calculate(double yaw, double pitch, double roll) {
        double sign;

        yaw = (yaw + 360) % 360;

        // danger zone for roll
        if (((yaw <= YAW_DANGER_ZONE_MARGIN && yaw >= 0) || (yaw <= 360 && yaw >= 360 - YAW_DANGER_ZONE_MARGIN)) || (yaw <= 180 + YAW_DANGER_ZONE_MARGIN && yaw >= 180 - YAW_DANGER_ZONE_MARGIN)) {
            sign = copySign(1, roll * cos(toRadians(yaw)));
        }
        // danger zone for pitch
        else if((yaw <= 90 + YAW_DANGER_ZONE_MARGIN && yaw >= 90 - YAW_DANGER_ZONE_MARGIN) || (yaw <= 270 + YAW_DANGER_ZONE_MARGIN && yaw >= 270 - YAW_DANGER_ZONE_MARGIN)) {
            sign = copySign(1, pitch * sin(toRadians(yaw)));
        }
        // just default to roll smh
        else {
            sign = copySign(1, roll * cos(toRadians(yaw)));
        }

        /*double sign = 1;
        if (sin(toRadians(yaw)) * roll < 1 || cos(toRadians(pitch)) * pitch < 1)
            sign = -1;*/
        return sign * toDegrees(acos(cos(toRadians(pitch)) * cos(toRadians(roll))));
    }
}
