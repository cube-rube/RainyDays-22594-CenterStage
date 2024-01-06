package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.modules.Deploy;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Launch;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.PullUp;

public class Robot {
    public final LinearOpMode linearOpMode;
    private FtcDashboard dashboard;
    private final BasicDrive basicDrive;
    private final Deploy deploy;
    private final Intake intake;
    private final Launch launch;
    private final Lift lift;
    private final PullUp pullUp;

    public Robot(LinearOpMode linearOpMode) {
        dashboard = FtcDashboard.getInstance();
        this.linearOpMode = linearOpMode;
        basicDrive = new BasicDrive(linearOpMode, dashboard);
        deploy = new Deploy(linearOpMode);
        intake = new Intake(linearOpMode);
        launch = new Launch(linearOpMode);
        lift = new Lift(linearOpMode, dashboard);
        pullUp = new PullUp(linearOpMode, dashboard);
    }

    public void operatorControl() {
        while (linearOpMode.opModeIsActive()) {
            deploy.opControl();
            intake.opControl();
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
