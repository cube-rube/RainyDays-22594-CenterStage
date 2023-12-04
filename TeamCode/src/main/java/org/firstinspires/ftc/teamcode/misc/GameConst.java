package org.firstinspires.ftc.teamcode.misc;


import com.acmerobotics.dashboard.config.Config;

@Config
public class GameConst {
    public enum StartPos {
        NEAR_BACKDROP,
        NOT_NEAR_BACKDROP
    }
    public static StartPos STARTPOS = StartPos.NEAR_BACKDROP; // 0 - near backdrop, 1 not near
    public enum AllianceColor {
        RED,
        BLUE
    }
    public static AllianceColor ALLIANCECOLOR = AllianceColor.RED; // 1 - red, 2 - blue
}
