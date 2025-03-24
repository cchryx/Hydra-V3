package components;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
public class OuttakeControl {

    public Servo OUTPIVOT_L, OUTPIVOT_R, OUTWRIST, OUTROTATE, OUTCLAW;
    public DcMotor VSLIDES_L, VSLIDES_R;
    public Gamepad gamepad1, gamepad2;
    public Telemetry telemetry;

    public PIDController slidesController;
    public double sP = 0.015, sI = 0, sD = 0.0008, sF = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public int slidesTarget = 0; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public int slidesPosition = 0;
    public double motorPowerSlides;
    public double slidesPID;
    public double sMinSpeed = -1;


    public double
            pivotTarget = Values.OUTPIVOT_INIT,
            wristTarget = Values.OUTWRIST_INIT,
            rotateTarget = Values.OUTROTATE_INIT,
            clawTarget = Values.CLAW_CLOSED; // TODO: RM STATIC AFTER TUNING (http://192.168.43.1:8080/dash)
    public double
            pivotLPosition = 0,
            pivotRPosition = 0,
            wristPosition = 0,
            clawPosition = 0;

    // Constructor to initialize the motors and gamepad
    public OuttakeControl(
            Servo OUTPIVOT_L,
            Servo OUTPIVOT_R,
            Servo OUTROTATE,
            Servo OUTWRIST,
            Servo OUTCLAW,
            DcMotor VSLIDES_L,
            DcMotor VSLIDES_R,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry

    ) {
        this.OUTPIVOT_L = OUTPIVOT_L;
        this.OUTPIVOT_R = OUTPIVOT_R;
        this.OUTROTATE = OUTROTATE;
        this.OUTWRIST = OUTWRIST;
        this.OUTCLAW = OUTCLAW;
        this.VSLIDES_L = VSLIDES_L;
        this.VSLIDES_R = VSLIDES_R;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        OUTPIVOT_R.setDirection(Servo.Direction.REVERSE);
        OUTPIVOT_L.setPosition(pivotTarget + Values.OUTPIVOT_OFFSET);
        OUTPIVOT_R.setPosition(pivotTarget);
        OUTROTATE.setPosition(0.166);
        OUTWRIST.setPosition(wristTarget);
        OUTCLAW.setPosition(clawTarget);

        VSLIDES_L.setZeroPowerBehavior(BRAKE);
        VSLIDES_R.setZeroPowerBehavior(BRAKE);
        VSLIDES_R.setDirection(DcMotor.Direction.REVERSE);

        VSLIDES_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VSLIDES_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VSLIDES_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VSLIDES_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesController = new PIDController(sP, sI, sD);
    }

    public void move() {
        pivotRPosition = OUTPIVOT_R.getPosition();
        pivotLPosition = OUTPIVOT_L.getPosition();
        wristPosition = OUTWRIST.getPosition();
        clawPosition = OUTCLAW.getPosition();
        slidesPosition = (VSLIDES_L.getCurrentPosition() + VSLIDES_R.getCurrentPosition()) / 2;

        //////////
        // PIDs //
        //////////

        // Slides
        slidesController.setPID(sP, sI, sD); // TODO: COMMENT OUT AFTER TUNING (http://192.168.43.1:8080/dash)
        slidesPID = slidesController.calculate(slidesPosition, slidesTarget);
        motorPowerSlides = slidesPID + sF;
        motorPowerSlides = Math.min(1, Math.max(sMinSpeed, motorPowerSlides));
        VSLIDES_L.setPower(motorPowerSlides);
        VSLIDES_R.setPower(motorPowerSlides);

        OUTPIVOT_L.setPosition(pivotTarget + Values.OUTPIVOT_OFFSET);
        OUTPIVOT_R.setPosition(pivotTarget);
        OUTROTATE.setPosition(rotateTarget);
        OUTWRIST.setPosition(wristTarget);
        OUTCLAW.setPosition(clawTarget);
    }

    public void telemetry() {
        telemetry.addData("OUTPIVOT_L", pivotLPosition);
        telemetry.addData("OUTPIVOT_R", pivotRPosition);
        telemetry.addData("slidesPosition", slidesPosition);
        telemetry.addData("slidesTarget", slidesTarget);

    }
}