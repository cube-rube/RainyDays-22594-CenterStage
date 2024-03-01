package org.firstinspires.ftc.teamcode.misc;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.modules.vision.AllianceColor;

@Config
public class GameConstants {
    public enum StartPos {
        NEAR,
        FAR
    }
    public static StartPos STARTPOS = StartPos.NEAR;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.RED;
}
