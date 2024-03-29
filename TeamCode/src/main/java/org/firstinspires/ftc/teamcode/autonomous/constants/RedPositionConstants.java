package org.firstinspires.ftc.teamcode.autonomous.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class RedPositionConstants {
    public static Pose2d NEAR_START_POSE = new Pose2d(14.885, -62.61615, Math.toRadians(90));
    public static double[] NEAR_START_COORDS = {14.885, -62.61615};
    public static double[] FAR_START_COORDS = {-38.485, -62.61615};
    public static double START_HEADING = Math.toRadians(90);

    public static double[] PURPLE_RIGHT_NEAR = {22.65, -38.5, 90};
    public static double[] PURPLE_RIGHT_FAR = {-46.5, -44, 155};
    public static double[] PURPLE_CENTER_NEAR = {15.2, -35.5};
    public static double[] PURPLE_CENTER_FAR = {-35.4, -36.3};
    public static double[] PURPLE_LEFT_NEAR = {12.5, -38, 155};
    public static double[] PURPLE_LEFT_FAR = {12.8, -37, 90};
    public static double PURPLE_RIGHT_HEADING = Math.toRadians(90);

    public static double[] BACKDROP_LEFT_COORDS = {50, -29.3};
    public static double[] BACKDROP_CENTER_COORDS = {50, -33.6}; // 36.2
    public static double[] BACKDROP_RIGHT_COORDS = {50, -41.5};

    public static double[] RIGGING_UP_COORDS = {5, -59 + 0.3};
    public static double[] RIGGING_DOWN_COORDS = {-40, -59 + 0.3};

    public static double[] DOOR_UP_COORDS = {24, -8};
    public static double[] DOOR_DOWN_COORDS = {-40, -8};
    public static double[] FIRST_PIXEL_STACK_COORDS = {-60.5, -35.3};
    public static double[] SECOND_PIXEL_STACK_COORDS = {-60.5, -23.3};
    public static double SECOND_PIXEL_STACK_BROKE = 4;
    public static double[] THIRD_PIXEL_STACK_COORDS = {-57.8, -11.5};
    public static double[] END_NEAR = {45, -55, 300};
}
