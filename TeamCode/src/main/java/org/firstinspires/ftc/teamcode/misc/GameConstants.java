package org.firstinspires.ftc.teamcode.misc;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.modules.vision.AllianceColor;

@Config
public class GameConstants {
    public enum StartPos {
        NEAR_BACKDROP,
        NOT_NEAR_BACKDROP
    }
    public static StartPos STARTPOS = StartPos.NEAR_BACKDROP; // 0 - near backdrop, 1 not near
    public static AllianceColor ALLIANCECOLOR = AllianceColor.BLUE; // 1 - red, 2 - blue
}
