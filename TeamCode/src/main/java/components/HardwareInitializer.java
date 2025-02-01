package components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class HardwareInitializer {

    private Map<String, RevBlinkinLedDriver> ledDrivers = new HashMap<>();
    private Map<String, DcMotor> motors = new HashMap<>();
    private Map<String, Servo> servos = new HashMap<>();


    // Method to initialize all hardware components
    public void initHardware(HardwareMap hardwareMap) {
//        ledDrivers.put("L_Light", hardwareMap.get(RevBlinkinLedDriver.class, "L_Light"));

        motors.put("FR", hardwareMap.get(DcMotor.class, "FR"));
        motors.put("FL", hardwareMap.get(DcMotor.class, "FL"));
        motors.put("BR", hardwareMap.get(DcMotor.class, "BR"));
        motors.put("BL", hardwareMap.get(DcMotor.class, "BL"));

        motors.put("VSLIDESL", hardwareMap.get(DcMotor.class, "VSLIDESL"));
        motors.put("VSLIDESR", hardwareMap.get(DcMotor.class, "VSLIDESR"));

        // OUTTAKE
        servos.put("OUTPIVOTR", hardwareMap.get(Servo.class, "OUTPIVOTR"));
        servos.put("OUTPIVOTL", hardwareMap.get(Servo.class, "OUTPIVOTL"));
        servos.put("OUTCLAW", hardwareMap.get(Servo.class, "OUTCLAW"));
        servos.put("OUTROTATE", hardwareMap.get(Servo.class, "OUTROTATE"));
        servos.put("OUTWRIST", hardwareMap.get(Servo.class, "OUTWRIST"));

        // INTAKE
        servos.put("INTURRET", hardwareMap.get(Servo.class, "INTURRET"));
        servos.put("INPIVOT", hardwareMap.get(Servo.class, "INPIVOT"));
        servos.put("INROTATE", hardwareMap.get(Servo.class, "INROTATE"));
        servos.put("INWRIST", hardwareMap.get(Servo.class, "INWRIST"));
        servos.put("INCLAW", hardwareMap.get(Servo.class, "INCLAW"));
        servos.put("HSLIDESF", hardwareMap.get(Servo.class, "HSLIDESF"));
        servos.put("HSLIDESB", hardwareMap.get(Servo.class, "HSLIDESB"));
    }

    // Getter for the LED lights by name
    public RevBlinkinLedDriver getLedDriver(String name) {
        return ledDrivers.get(name);
    }

    // Getter for the motors by name
    public DcMotor getMotor(String name) {
        return motors.get(name);
    }

    // Getter for servos by name
    public Servo getServo(String name) {
        return servos.get(name);
    }

}