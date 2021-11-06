package frc.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.Robot;
import frc.utilities.Xbox;
//import frc.utilities.SoftwareTimer;
//import frc.utilities.Toggler;


public class ArmPivot extends Thread implements Runnable {

	private TalonSRX pivoter1;
	//private DigitalInput calibrateSwitch;
	//private Toggler manualOverrideTogglerPivot;
	//private SoftwareTimer armTimer;
	Encoder pivotEncoder;

	// Constants
	public final double CARGO_IN_SETPOINT = 0.0;
	public final double DOWN_SETPOINT = -90.0;

	public final double UP_SETPOINT = 20.0;

	//private final double BACKDRIVE_POWER = 0.3;

	//private double STATIC_GAIN = 0.0;// 0.41 max 0.34 min
	//private double PROPORTIONAL_GAIN = 65800e-6;
	//private double INTEGRAL_GAIN = 7600e-6;
	//private double DERIVATIVE_GAIN = -3450e-6;
	//private final double STARTING_ANGLE = DOWN_SETPOINT;
	//private final double SETPOINT_INCREMENT = 5; // deg
	private final double MAX_POWER = 1.0;
	//private final double UPPER_LIMIT = 20;
	//private final double LOWER_LIMIT = -90;
	//private final double DEGREES_PER_PULSE = (-0.2036244521);
	//private final double ANGLE_THRESHOLD = 90; // deg
	//private final double ANGLE_COAST_RATE = 90; // deg/s

	// General parameters
	//private boolean delayInit = true;
	//private boolean manualOverrideIsEngaged = true;
	//private boolean isBackDriven = false;
	//private double encoderOffset = 0;
	//private double estimatedAngle = 90; // deg
	//private double commandedAngle = STARTING_ANGLE; // deg
	private double manualPower = 0;
	//private double staticPower = 0;
	//private double activePower = 0;
	private double commandedPower = 0;

	// PID parameters
	//private boolean initializePID = true;
	//private double angleError = 0; // deg
	//private double angleErrorLast = 0; // deg
	//private double accumulatedError = 0; // deg*s
	//private double rateError = 0; // deg/s
	//private double timeNow;
	//private double timePrevious;
	//private double timeStep;

	public ArmPivot(int pivoterCANPort1){//, int encA, int encB, int limSwitchDIO) {
		pivoter1 = new TalonSRX(pivoterCANPort1);
		//calibrateSwitch = new DigitalInput(limSwitchDIO);

		//manualOverrideTogglerPivot = new Toggler(Xbox.Back);
		//armTimer = new SoftwareTimer();
		//pivotEncoder = new Encoder(encA, encB);

	}

	private void computeManualPower() {
		if (Math.abs(Robot.xboxJoystick.getRawAxis(Xbox.LAxisY)) > 0.2) {
			manualPower = Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) ;
		} else {
			manualPower = 0;
		}
	}


	private void limitCommandedPower() {
		// Limit the range of commanded power
		if (commandedPower > MAX_POWER) {
			commandedPower = MAX_POWER;
		} else if (commandedPower < -MAX_POWER) {
			commandedPower = -MAX_POWER;
		}
	}


	public void performMainProcessing() {
		computeManualPower();
			commandedPower = manualPower;
		
		limitCommandedPower();
		pivoter1.set(ControlMode.PercentOutput, -commandedPower);
	}
}