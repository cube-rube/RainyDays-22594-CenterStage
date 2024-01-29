package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class RedPositionConstants {
    public static Pose2d NEAR_START_POSE = new Pose2d(15, -63, Math.toRadians(90));

    public static Vector2d PURPLE_RIGHT_VECTOR = new Vector2d(9, -33);
    public static double PURPLE_RIGHT_HEADING = Math.toRadians(150);
    public static Vector2d PURPLE_CENTER_VECTOR = new Vector2d(13, -33);
    public static Vector2d PURPLE_LEFT_VECTOR = new Vector2d(20, -38);
    public static double PURPLE_LEFT_HEADING = Math.toRadians(290);

    public static Vector2d BACKDROP_RIGHT_VECTOR = new Vector2d(50, -25);
    public static Vector2d BACKDROP_CENTER_VECTOR = new Vector2d(50, -34);
    public static Vector2d BACKDROP_LEFT_VECTOR = new Vector2d(50, -42);

    public static Vector2d RIGGING_UP_VECTOR = new Vector2d(5, -60);
    public static Vector2d RIGGING_DOWN_VECTOR = new Vector2d(-30, -60);
    public static Vector2d PIXEL_STACK_VECTOR = new Vector2d(-58,  -35);
}
