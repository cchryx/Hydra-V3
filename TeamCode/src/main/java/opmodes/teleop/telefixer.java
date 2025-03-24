package opmodes.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import components.HardwareInitializer;
import components.IntakeControl;
import components.MecanumDTControl;
import components.OuttakeControl;
import components.Values;


@TeleOp(group = "Actual", name = "telefixer")
public class telefixer extends OpMode {
    private HardwareInitializer hardwareInitializer;
    private MecanumDTControl mecanumDrive;
    private OuttakeControl outake;
    private IntakeControl intake;


    // Initialize hardware
    DcMotor FR, FL, BR, BL, VSLIDES_L, VSLIDES_R;
    Servo OUTPIVOT_L, OUTPIVOT_R, OUTROTATE, OUTWRIST, OUTCLAW, INTURRET, INPIVOT, INROTATE, INWRIST, INCLAW, HSLIDES_F, HSLIDES_B;


    private ElapsedTime autoTime = new ElapsedTime();
    private String autoProcess_d = "none", autoProcess_i = "none";
    private int autoStep_d = 0, autoStep_i = 0;
    boolean pChamber = false;
    boolean pBasket = false;
    boolean pHome = false;
    boolean pTransfer = false;
    boolean pSubmersible = false;
    boolean pDropoff = false;
    boolean pFailed = false;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        hardwareInitializer = new HardwareInitializer();
        hardwareInitializer.initHardware(hardwareMap);

        FR = hardwareInitializer.getMotor("FR");
        FL = hardwareInitializer.getMotor("FL");
        BR = hardwareInitializer.getMotor("BR");
        BL = hardwareInitializer.getMotor("BL");
        VSLIDES_L = hardwareInitializer.getMotor("VSLIDESL");
        VSLIDES_R = hardwareInitializer.getMotor("VSLIDESR");
        OUTPIVOT_L = hardwareInitializer.getServo("OUTPIVOTL");
        OUTPIVOT_R = hardwareInitializer.getServo("OUTPIVOTR");
        OUTROTATE = hardwareInitializer.getServo("OUTROTATE");
        OUTWRIST = hardwareInitializer.getServo("OUTWRIST");
        OUTCLAW = hardwareInitializer.getServo("OUTCLAW");
        INTURRET = hardwareInitializer.getServo("INTURRET");
        INPIVOT = hardwareInitializer.getServo("INPIVOT");
        INROTATE = hardwareInitializer.getServo("INROTATE");
        INWRIST = hardwareInitializer.getServo("INWRIST");
        INCLAW = hardwareInitializer.getServo("INCLAW");
        HSLIDES_F = hardwareInitializer.getServo("HSLIDESF");
        HSLIDES_B = hardwareInitializer.getServo("HSLIDESB");


        mecanumDrive = new MecanumDTControl(FR, FL, BR, BL, gamepad1);
        outake = new OuttakeControl(
                OUTPIVOT_L,
                OUTPIVOT_R,
                OUTROTATE,
                OUTWRIST,
                OUTCLAW,
                VSLIDES_L,
                VSLIDES_R,
                gamepad1,
                gamepad2,
                telemetry
        );


        intake = new IntakeControl(
                INTURRET,
                INPIVOT,
                INROTATE,
                INWRIST,
                INCLAW,
                HSLIDES_F,
                HSLIDES_B,
                gamepad1,
                gamepad2,
                telemetry
        );
        mecanumDrive.init();

        OUTPIVOT_R.setDirection(Servo.Direction.REVERSE);
        OUTPIVOT_L.setPosition(0.5 + Values.OUTPIVOT_OFFSET);
        OUTPIVOT_R.setPosition(0.5);
        OUTROTATE.setPosition(0);
        OUTWRIST.setPosition(0.45);
        OUTCLAW.setPosition(1);
        intake.init();
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {
        mecanumDrive.move();

        VSLIDES_L.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        VSLIDES_R.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        outake.telemetry();
        intake.telemetry();

        telemetry.addData("autoStep_d", autoStep_d);
        telemetry.addData("autoProcess_d", autoProcess_d);
        telemetry.addData("autoStep_i", autoStep_i);
        telemetry.addData("autoProcess_i", autoProcess_i);
    }
}