package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OperatorDrive;
import org.firstinspires.ftc.teamcode.modules.PullUp;
import org.firstinspires.ftc.teamcode.modules.Scorer;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Lift;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {
    public OperatorDrive operatorDrive;
    public Lift lift;
    public Intake intake;
    public Scorer scorer;
    public PullUp pullUp;

    public Shooter shooter;
    //public guro gyroscope;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();
        lift.timer.reset();

        shooter.start_shooter_pos(); //может все сломать

        pullUp.timer.reset();
        operatorDrive.runtimeReset();


        while (opModeIsActive()) {
            operatorDrive.driveFieldCentric();
            operatorDrive.telemetry();
            lift.opControlPos();
            intake.opControlSensor();
            scorer.opControl();
            pullUp.opControlPos();
            shooter.tele();

            telemetry.addData("Runtime", runtime.toString());

            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();
        MultipleTelemetry mulTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        operatorDrive = new OperatorDrive(this, dashboard);
        lift = new Lift(this, dashboard);
        intake = new Intake(this);
        scorer = new Scorer(this, lift);
        pullUp = new PullUp(this, dashboard);
        shooter = new Shooter(this);

        telemetry.update();
    }
}
