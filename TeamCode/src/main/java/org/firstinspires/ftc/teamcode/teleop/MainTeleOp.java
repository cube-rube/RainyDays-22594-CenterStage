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
            operatorDrive.drive();
            operatorDrive.telemetry();
            lift.opControl();
            switch (intake.intakeState) {
                case STOP:
                    intake.opControlSensor();
                    break;
                case EJECT:
                    intake.ejectOp();
                    break;
            }
            scorer.opControl();
            pullUp.opControlPos();
            shooter.tele();

            switch (scorer.intakeState) {
                case EJECT:
                    intake.intakeState = Intake.IntakeState.EJECT;
                    break;
                case STOP:
                    intake.intakeState = Intake.IntakeState.STOP;
                    break;
            }

            if (scorer.rotationBeamState == Scorer.RotationState.DEPLOY) {
                operatorDrive.driveState = OperatorDrive.DriveState.BACKDROP;
            } else if (gamepad1.x) {
                operatorDrive.driveState = OperatorDrive.DriveState.PULLUP;
            } else {
                operatorDrive.driveState = OperatorDrive.DriveState.FIELD;
            }

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
