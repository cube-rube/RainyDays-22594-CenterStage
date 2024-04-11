package org.firstinspires.ftc.teamcode.autonomous.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class BluePositionConstants {
    public static Pose2d NEAR_START_POSE = new Pose2d(14.885, 62.61615, Math.toRadians(270));
    public static double[] NEAR_START_COORDS = {14.885, 62.61615};
    public static double[] FAR_START_COORDS = {-38.485, 62.61615};
    public static double START_HEADING = Math.toRadians(270);

    public static double[] PURPLE_LEFT_NEAR = {21.6, 40, 270};
    public static double[] PURPLE_LEFT_FAR = {-46.5, 44, 335};
    public static double[] PURPLE_CENTER_NEAR = {15.2, 35};
    public static double[] PURPLE_CENTER_FAR = {-35.4, 36.3};
    public static double[] PURPLE_RIGHT_NEAR = {10.6, 36, 205};
    public static double[] PURPLE_RIGHT_FAR = {11.6, 37, 205};
    public static double PURPLE_RIGHT_HEADING = Math.toRadians(270);

    public static double[] BACKDROP_RIGHT_COORDS = {50.2, 29.3};
    public static double[] BACKDROP_CENTER_COORDS = {50.2, 36.2}; // 33.2
    public static double[] BACKDROP_LEFT_COORDS = {50.2, 40};

    public static double[] RIGGING_UP_COORDS = {5, 59 - 2.5};
    public static double[] RIGGING_DOWN_COORDS = {-45, 59 - 2.5};

    public static double[] DOOR_UP_COORDS = {24, 8};
    public static double[] DOOR_DOWN_COORDS = {-40, 8};
    public static double[] FIRST_PIXEL_STACK_COORDS = {-57.8, 35.3};
    public static double[] SECOND_PIXEL_STACK_COORDS = {-57.8, 23.3};
    public static double SECOND_PIXEL_STACK_BROKE = 4;
    public static double[] THIRD_PIXEL_STACK_COORDS = {-57.8, 11.5};
    public static double[] END_NEAR = {45, 55, 40};
    public static double[] END_FAR = {45, 55 - 43, 0};
}
