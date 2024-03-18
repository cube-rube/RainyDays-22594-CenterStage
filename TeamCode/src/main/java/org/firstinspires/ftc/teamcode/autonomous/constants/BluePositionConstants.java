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

    public static Vector2d PURPLE_LEFT_VECTOR = new Vector2d(11, 37);
    public static double[] PURPLE_LEFT_NEAR = {22.65, 44, 90};
    public static double[] PURPLE_LEFT_FAR = {-46.5, 44, 335};
    public static double[] PURPLE_CENTER_NEAR = {14.2, 37};
    public static double[] PURPLE_CENTER_FAR = {-35.4, 36.3};
    public static Vector2d PURPLE_RIGHT_VECTOR = new Vector2d(23.2, 44);
    public static double[] PURPLE_RIGHT_NEAR = {12.8, 37, 205};
    public static double PURPLE_RIGHT_HEADING = Math.toRadians(270);



    public static Vector2d BACKDROP_RIGHT_VECTOR = new Vector2d(49.5, 29);
    public static double[] BACKDROP_RIGHT_COORDS = {49.8, 29.3};
    public static Vector2d BACKDROP_CENTER_VECTOR = new Vector2d(48.9, 35);
    public static double[] BACKDROP_CENTER_COORDS = {49.8, 34.8};
    public static Vector2d BACKDROP_LEFT_VECTOR = new Vector2d(49.6, 42);
    public static double[] BACKDROP_LEFT_COORDS = {49.8, 41.5};

    public static Vector2d DIFF_VECTOR = new Vector2d(0, 0.5);

    public static Vector2d RIGGING_UP_VECTOR = new Vector2d(5, 59);
    public static Vector2d RIGGING_DOWN_VECTOR = new Vector2d(-30, 59);

    public static Vector2d DOOR_UP_VECTOR = new Vector2d(24, 7);
    public static double[] DOOR_UP_COORDS = {24, 8};
    public static Vector2d DOOR_DOWN_VECTOR = new Vector2d(-40, 7);
    public static double[] DOOR_DOWN_COORDS = {-40, 8};
    public static Vector2d PIXEL_STACK_VECTOR = new Vector2d(-58.3,  35);
    public static double[] FIRST_PIXEL_STACK_COORDS = {-60.5, 35.3};
    public static double[] SECOND_PIXEL_STACK_COORDS = {-60.5, 23.3};
    public static double SECOND_PIXEL_STACK_BROKE = 4;
    public static double[] THIRD_PIXEL_STACK_COORDS = {-59.1, 11.5};
    public static double[] END_NEAR = {47, 55, 40};
}
