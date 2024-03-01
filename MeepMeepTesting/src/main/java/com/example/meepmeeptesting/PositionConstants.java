package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PositionConstants {
    public static Pose2d NEAR_START_POSE = new Pose2d(14.885, -62.61615, Math.toRadians(90));
    public static double[] NEAR_START_COORDS = {14.885, -62.61615};
    public static double[] FAR_START_COORDS = {-38.485, -62.61615};
    public static double START_HEADING = Math.toRadians(90);

    public static Vector2d PURPLE_LEFT_VECTOR = new Vector2d(11, -37);
    public static double PURPLE_LEFT_HEADING = Math.toRadians(155);
    public static double[] PURPLE_CENTER_NEAR = {13, -37};
    public static double[] PURPLE_CENTER_FAR = {-35.4, -37};
    public static Vector2d PURPLE_RIGHT_VECTOR = new Vector2d(23.2, -44);
    public static double PURPLE_RIGHT_HEADING = Math.toRadians(90);



    public static Vector2d BACKDROP_LEFT_VECTOR = new Vector2d(49.5, -29);
    public static Vector2d BACKDROP_CENTER_VECTOR = new Vector2d(48.9, -35);
    public static double[] BACKDROP_CENTER_COORDS = {48.9, -35};
    public static Vector2d BACKDROP_CENTER_LEFT_VECTOR = new Vector2d(48.9, -35);
    public static Vector2d BACKDROP_CENTER_RIGHT_VECTOR = new Vector2d(48.9, -37.5);
    public static Vector2d BACKDROP_RIGHT_VECTOR = new Vector2d(49.6, -42);
    public static double[] BACKDROP_RIGHT_COORDS = {48.9, -42};

    public static Vector2d DIFF_VECTOR = new Vector2d(0, -0.5);

    public static Vector2d RIGGING_UP_VECTOR = new Vector2d(5, -59);
    public static Vector2d RIGGING_DOWN_VECTOR = new Vector2d(-30, -59);

    public static Vector2d DOOR_UP_VECTOR = new Vector2d(24, -5);
    public static double[] DOOR_UP_COORDS = {24, -8};
    public static Vector2d DOOR_DOWN_VECTOR = new Vector2d(-40, -5);
    public static double[] DOOR_DOWN_COORDS = {-40, -8};
    public static Vector2d PIXEL_STACK_VECTOR = new Vector2d(-58.3,  -35);
    public static double[] FIRST_PIXEL_STACK_COORDS = {-60.5, -35.3};
    public static double[] SECOND_PIXEL_STACK_COORDS = {-60.5, -23.3};
    public static double[] THIRD_PIXEL_STACK_COORDS = {-60.5, -11.5};
}
