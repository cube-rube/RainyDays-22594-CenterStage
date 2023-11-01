package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.drive.BasicDriveNoEncoders;
import org.firstinspires.ftc.teamcode.modules.Claw;
import org.firstinspires.ftc.teamcode.modules.Deploy;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Launch;
import org.firstinspires.ftc.teamcode.modules.Lift;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    private BasicDriveNoEncoders basicDrive;
    private Lift lift;
    private Intake intake;
    private Deploy deploy;
    private Claw claw;
    private Launch launch;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();

        while (opModeIsActive()) {
            basicDrive.tele();
            intake.tele();
            lift.easyTele();
            deploy.easyTele();
            claw.tele();

            telemetry.update();
        }
    }

    private void initRobot() {
        basicDrive = new BasicDriveNoEncoders(this, runtime);
        lift = new Lift(this);
        intake = new Intake(this);
        deploy = new Deploy(this);
        claw = new Claw(this);
        launch = new Launch(this);

        telemetry.update();
    }
}
