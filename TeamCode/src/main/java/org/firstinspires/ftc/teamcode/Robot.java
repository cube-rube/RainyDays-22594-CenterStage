package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.modules.Scorer;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.PullUp;

public class Robot {
    public final LinearOpMode linearOpMode;
    private FtcDashboard dashboard;
    private final BasicDrive basicDrive;
    private final Scorer scorer;
    private final Intake intake;
    private final Shooter shooter;
    private final Lift lift;
    private final PullUp pullUp;

    public Robot(LinearOpMode linearOpMode) {
        dashboard = FtcDashboard.getInstance();
        this.linearOpMode = linearOpMode;
        basicDrive = new BasicDrive(linearOpMode, dashboard);

        intake = new Intake(linearOpMode);
        shooter = new Shooter(linearOpMode);
        lift = new Lift(linearOpMode, dashboard);
        pullUp = new PullUp(linearOpMode, dashboard);
        scorer = new Scorer(linearOpMode, lift);
    }

    public void operatorControl() {
        while (linearOpMode.opModeIsActive()) {
            scorer.opControl();
            intake.opControlSensor();
            lift.opControl();
            pullUp.opControlPID();

            timersReset();
        }
    }

    public void timersReset() {
        lift.timer.reset();
        pullUp.timer.reset();
    }
}
