package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.modules.Deploy;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    private BasicDrive basicDrive;
    private Lift lift;
    private Intake intake;
    private Deploy deploy;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();

        while (opModeIsActive()) {
            basicDrive.testing();
            lift.testing();
            intake.testing();
            deploy.testing();

            telemetry.update();
        }
    }

    private void initRobot() {
        basicDrive = new BasicDrive(this, runtime);
        lift = new Lift(this);
        intake = new Intake(this);
        deploy = new Deploy(this);

        telemetry.update();
    }
}
