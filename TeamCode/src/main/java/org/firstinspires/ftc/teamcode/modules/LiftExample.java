package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftExample {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private Gamepad gamepad1;
    private DigitalChannel digitalTouch;

    /*


 digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
 digitalTouch.setMode(DigitalChannel.Mode.INPUT);

     */

    private double kF = 0;
    private double kP = 0.01;
    private double error;
    private double intakePos = 0;
    private double lowPos = -560;
    private double middlePos = -1090;
    private double nwZero = 0;
    private double highPos = -1650;
    private double groundPos = -300;
    private DcMotorEx motor1, motor2;
    private boolean candonwz = false;
    private int LLPT = 1;
    private int LiftPosOnG = -1;
    private double tempPowMot = 0;
    private double takeKon = 120;
    public double lifPoseForIntake = 0;

    public enum State {
        BYPASS,
        INTAKE,
        GROUND,
        LOW,
        MIDDLE,
        HIGH,
        NWZERO,
        CANDON,
        //POSITION,
    }
    public State state = State.BYPASS;

    public LiftExample(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad1 = linearOpMode.gamepad2;


        motor1 = hardwareMap.get(DcMotorEx.class, "liftmotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        digitalTouch = hardwareMap.get(DigitalChannel.class, "Gercone_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("", "Lift initialized!");
    }

    public void AutoMotorSetP(double d){
        motor1.setPower(d);
        motor2.setPower(d);
    }

    private int znakdbl( double a){
        if (a > 0)
            return 1;
        else if (a < 0)
            return -1;
        else
            return 0;
    }
    public void easyTele(){
        motor1.setPower(-gamepad1.left_stick_y);
        motor2.setPower(-gamepad1.left_stick_y);
    }

    public void updateWithst(){
        if (gamepad1.y) {
            LiftPosOnG = -1;
        }
        else if (LiftPosOnG >= 0) {
            double needHight = nwZero;

            if (LiftPosOnG == 1) needHight += takeKon;
            else if (LiftPosOnG == 2) needHight -= lowPos;
            else if (LiftPosOnG == 3) needHight -= middlePos;
            else if (LiftPosOnG == 4) needHight -= highPos;
            if (motor1.getCurrentPosition() < needHight - 40) tempPowMot = 1;
            else if (motor1.getCurrentPosition() > needHight + 40) tempPowMot = -1;
            else {
                LiftPosOnG = -2;
                tempPowMot = 0.2;
            }
            //state = State.POSITION;
        }
        telemetry.addData("", "LiftPose: " + LiftPosOnG);
        telemetry.addData("", "LLPT: " + LLPT);
        if (gamepad1.dpad_down){
            state = State.NWZERO;
        }
        else if (gamepad1.dpad_up){
            LiftPosOnG = 4;
            //state = State.POSITION;
            //state = State.HIGH;
        }
        else if (gamepad1.b){
            DoZeroLift_DigitalSensor();
        }
        else if (gamepad1.dpad_left){
            LiftPosOnG = 2;
            //state = State.POSITION;
            //state = State.LOW;
        }
        else if (gamepad1.dpad_right){
            LiftPosOnG = 3;
            //state = State.POSITION;
            //state = State.MIDDLE;
        }
        else if(gamepad1.right_trigger > 0.5){
            //state = State.BYPASS;
            motor1.setPower(1);
            motor2.setPower(1);
            state = State.CANDON;
        }
        else if(gamepad1.left_trigger > 0.5){
            //state = State.GROUND;
            motor1.setPower(-1);
            motor2.setPower(-1);
            state = State.CANDON;
        }
        /*else if(gamepad1.x){
            state = State.CANDON;
        }*/
        else if (gamepad1.a){
            LiftPosOnG = LLPT;
        }
        else{
            state = State.BYPASS;
        }
        switch (state) {
            case BYPASS:
                if (gamepad1.left_stick_y != 0){
                    LiftPosOnG = -1;
                    if(gamepad1.left_stick_y > 0 && motor1.getCurrentPosition() < nwZero){
                        LiftPosOnG = -1;
                        motor1.setPower(0);
                        motor2.setPower(0);
                    }
                    else {
                        motor1.setPower(-gamepad1.left_stick_y);
                        motor2.setPower(-gamepad1.left_stick_y);
                    }
                }
                else if (LiftPosOnG != -1){
                    motor1.setPower(tempPowMot);
                    motor2.setPower(tempPowMot);
                }
                else {
                    motor1.setPower(0);
                    motor2.setPower(0);
                }

                /*if (Math.abs(gamepad1.left_stick_x) > 0.7 && candonwz) {
                    motor1.setPower(znakdbl(gamepad1.left_stick_x) * 0.25);
                    motor2.setPower(znakdbl(gamepad1.left_stick_x) * 0.25);
                }*/
                break;
            /*case POSITION:

                break;*/
            case INTAKE:
                error = intakePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case NWZERO:
                nwZero = motor1.getCurrentPosition();
                break;
            case CANDON:
                if (candonwz)
                    candonwz = false;
                else
                    candonwz = true;
                break;
            case GROUND:
                error = groundPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case LOW:
                error = lowPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case MIDDLE:
                error = middlePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case HIGH:
                error = highPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
        }
        lifPoseForIntake = motor1.getCurrentPosition();
        telemetry.addData("gamepad", gamepad1.left_stick_y);
        telemetry.addData("error", error);
        telemetry.addData("lift",motor1.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.update();


    }

    public void Pos2() {
        while (motor1.getCurrentPosition() > 200) {
            motor1.setPower(-0.3);
            motor2.setPower(-0.3);
        }
        motor1.setPower(0.1);
        motor2.setPower(0.1);
    }

    public int PositionTeleop1(double posit, int i)
    {
        if (i == 1){
            double y = 0.3;
            if (motor1.getCurrentPosition() < posit) y = -1;
            while (motor1.getCurrentPosition() * y > posit * y){
                AutoMotorSetP( -1 * y);}
            AutoMotorSetP(0.1);
            return 0;
        }
        int tempPowM1 = 0;
        if (motor1.getCurrentPosition() < posit - 10) tempPowM1 = 1;
        else if (motor1.getCurrentPosition() > posit + 10) tempPowM1 = -1;
        motor1.setPower(tempPowM1);
        motor2.setPower(tempPowM1);
        return tempPowM1;
    }


    public void update(){

        if (gamepad1.y){
            state = State.NWZERO;
        }
        /*else if (gamepad2.dpad_up){
            state = State.HIGH;
        }
        else if (gamepad2.dpad_left){
            state = State.LOW;
        }
        else if (gamepad2.dpad_right){
            state = State.MIDDLE;
        }*/
        else if(gamepad1.left_bumper){
            state = State.BYPASS;
        }
        /*else if(gamepad2.left_bumper){
            state = State.GROUND;
        }*/
        else if(gamepad1.x){
            state = State.CANDON;
        }
        else{
            state = State.BYPASS;
        }

        switch (state) {
            case BYPASS:
               if(gamepad1.right_stick_y >= 0 && motor1.getCurrentPosition() < nwZero){
                    motor1.setPower(0);
                    motor2.setPower(0);
                }
                else if(gamepad1.right_stick_y < 0){
                   motor1.setPower(-gamepad1.right_stick_y * 0.75);
                   motor2.setPower(-gamepad1.right_stick_y * 0.75);
               }
                else{
                    motor1.setPower(-gamepad1.right_stick_y);
                    motor2.setPower(-gamepad1.right_stick_y);
                }
                if (Math.abs(gamepad1.right_stick_x) > 0.7 && candonwz) {
                    motor1.setPower(znakdbl(gamepad1.right_stick_x) * 0.25);
                    motor2.setPower(znakdbl(gamepad1.right_stick_x) * 0.25);
                }
                /*if(gamepad2.left_stick_y == 0){
                    motor1.setPower(0.25);
                    motor2.setPower(0.25);
                }*/
                break;
            case INTAKE:
                error = intakePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case NWZERO:
                nwZero = motor1.getCurrentPosition();
                break;
            case CANDON:
                if (candonwz)
                    candonwz = false;
                else
                    candonwz = true;
                break;
            case GROUND:
                error = groundPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case LOW:
                error = lowPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case MIDDLE:
                error = middlePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case HIGH:
                error = highPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
        }
        /*if (digitalTouch.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }*/
        telemetry.addData("gamepad", gamepad1.left_stick_y);
        telemetry.addData("error", error);
        telemetry.addData("lift",motor1.getCurrentPosition());
        telemetry.addData("state", state);
        //telemetry.update();


    }


    public void DoZeroLift_DigitalSensor(){
        if ((digitalTouch.getState())){
            while ((digitalTouch.getState()) && gamepad1.left_stick_y == 0){
                motor1.setPower(-0.3);
                motor2.setPower(-0.3);
                telemetry.addData("down", -1);
                telemetry.update();
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 300 && gamepad1.left_stick_y == 0){
            motor1.setPower(-0.05);
            motor2.setPower(-0.05);
        }
        t = new ElapsedTime();
        while ((digitalTouch.getState() == false) && gamepad1.left_stick_y == 0 && t.milliseconds() < 700){
            motor1.setPower(0.25);
            motor2.setPower(0.25);
            telemetry.addData("up", 1);
            telemetry.update();
        }
        motor1.setPower(0);
        motor2.setPower(0);
        nwZero = motor1.getCurrentPosition();
    }


    public void updateavto(){
        /*error = lowPos - motor1.getCurrentPosition();
        motor1.setPower(kP * error);
        motor2.setPower(kP * error);*/
        switch (state) {
            case BYPASS:
                if(gamepad1.right_stick_y >= 0 && motor1.getCurrentPosition() > nwZero){
                    motor1.setPower(0);
                    motor2.setPower(0);
                }
                else{
                    motor1.setPower(gamepad1.right_stick_y);
                    motor2.setPower(gamepad1.right_stick_y);
                }
                break;
            case INTAKE:
                error = intakePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case GROUND:
                error = groundPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case LOW:
                error = lowPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case MIDDLE:
                while (motor1.getCurrentPosition() > middlePos){
                    motor1.setPower(-1);
                    motor2.setPower(-1);
                }

                motor1.setPower(0);
                motor2.setPower(0);

                /*
                error = middlePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);*/
                break;
            case HIGH:
                error = highPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
        }
        telemetry.addData("gamepad", gamepad1.left_stick_y);
        telemetry.addData("error", error);
        telemetry.addData("lift",motor1.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.update();


    }

    public void teleop() {

        motor1.setPower(gamepad1.right_stick_y);
       motor2.setPower(gamepad1.right_stick_y);


        /* if (Math.abs(gamepad2.left_stick_y) > 0)
            state = State.TELE;
        else
            state = State.HOLD;

        switch (state) {
            case HOLD:
                motor1.setPower(-kP * motor1.getCurrentPosition() + kF);
                 motor2.setPower(-kP * motor1.getCurrentPosition() + kF);
            case TELE:
                motor1.setPower(gamepad2.left_stick_y + kF);
               motor2.setPower(gamepad2.left_stick_y + kF);

        }*/

        telemetry.addData("lift",motor1.getCurrentPosition());
        telemetry.update();
      /*  if(gamepad2.a) {
            while (gamepad2.a) {
                double power = 1;
                servo3.setPower(power);
                servo2.setPower(-power);
            }
        }
        if(gamepad2.b) {
            while (gamepad2.b) {
                double power = 1;
                servo3.setPower(-power);
                servo2.setPower(power);
            }
        }
        else {
            servo3.setPower(0);
            servo2.setPower(0);
        }*/
    }
    public void Pos0() {
        error = 6000 - motor1.getCurrentPosition();
        motor1.setPower(-kP * error);
        motor2.setPower(-kP * error);

        /*ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 4000) {
            motor1.setPower(1);
            motor2.setPower(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);*/
    }
    public void Pos1() {
        error = 6000 - motor1.getCurrentPosition();
        motor1.setPower(kP * error);
        motor2.setPower(kP * error);

        /*ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 4000) {
            motor1.setPower(1);
            motor2.setPower(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);*/
    }
   /* public void Pos1(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.dpad_down) {
            while (motor1.getCurrentPosition() < 1450) {
                motor1.setPower(1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
    public void Pos2(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.dpad_right) {
            while (motor1.getCurrentPosition() < 2000) {
                motor1.setPower(1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
    // Pos 3 is correct variant
    public void Pos3(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.dpad_up) {
            while (motor1.getCurrentPosition() < 2500) {
                motor1.setPower(1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }

    }*/
}
