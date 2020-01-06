package frc.robot;


public class Const {
	// XBoxController
	public static final int DriveControllerPort = 0;
	public static final int OperateControllerPort = 2;
	
	// DriveBaseMotors
	public static final int DriveRightPort = 1;
	public static final int DriveLeftPort = 0;
	
	// LiftMotors
	public static final int LiftMotorPort = 2;

	// ArmSolenoids & Motors
	public static final int ArmSolenoidPort = 0;
	public static final int BarSolenoidPort = 1;
	public static final int RollerMotorPort = 3;

	// Solenoids & Motors for climbing
	public static final int ClimbSolenoidPort = 2;
	public static final int ClimbMotorPort = 4;
	
	// DriveBaseEncoders
	public static final int RightDriveEncoderAPort = 0;
	public static final int RightDriveEncoderBPort = 1;
	public static final int LeftDriveEncoderAPort = 2;
	public static final int LeftDriveEncoderBPort =3;

	// LiftEncoders
	public static final int LiftEncoderAPort = 4;
	public static final int LiftEncoderBPort = 5;

	// Senosors for line trace
	public static final int RightFrontSensorPort = 0;
	public static final int RightBackSensorPort = 0;
	public static final int LeftFrontSensorPort = 0;
	public static final int LeftBackSensorPort = 0;

	// cargo
	public static final int cargoSenosorPort = 6;
	
	/**
		Field Dimension	
		everything in cm
	 */
	private static final double CmPerInch = 2.54;
	private static final double HeightAdjustment = 20;	// 高さの微調整

	// 「カーゴを下に落ちないようにする板」を基準とした高さにする。 
	public static final double RocketFirstCargoHeight = 26.8 * CmPerInch - HeightAdjustment;
	public static final double RocketSecondCargoHeight = 54.8 * CmPerInch - HeightAdjustment;
	public static final double RocketSecondHatchHeight = 46.3 * CmPerInch - 18.3 * CmPerInch + HeightAdjustment;	//Hatch用の棒が18.3inchの高さにあるのでその分マイナスする 
	public static final double ShipCargoHeight = 40 * CmPerInch + HeightAdjustment;
	public static final double HabSecondHeight = 60;	
	public static final double HabThirdHeight = 125;	
	public static final double GroundHeight = 0;
	public static final double LaunchCargoHeight = 20;
	public static final double FinalClimbHeight = 59;

	public static final double HoldPanelHeight = 10;

	/**
		Robot Dimension
	 */
	public static final double DriveEncoderDistancePerPulse = 7.7 * Math.PI / 10.71;
    public static final double LiftMinHeight = 0 ;
	public static final double LiftEncoderDistancePerPulse = -0.1107585212;    // エンコーダ(リフトのモーター)の向きと実際に動く向きが逆なのでマイナスがつく
	
	public static final double LiftPIDTolearnce = 10;	// LiftのPIDOnTargetの許容範囲(cm)

	public static final double KeepLiftHeightSpeed = 0.3;

	public static final double KeepHoldingCargoSpeed =0.3;
	
	// NetworkTable for finding lines
	public static final String LineFindNetworkTable = "";

	public static enum ArmHeight {
		shipCargoHeight(ShipCargoHeight),
		rocketFirstCargoHeight(RocketFirstCargoHeight),
		rocketSecondCargoHeight(RocketSecondCargoHeight),
		panel_1Height(0),
		panel_2Height(RocketSecondHatchHeight);		

		public double height;
		private  ArmHeight(double height) {
			this.height = height;
		}
	}

	// Deadband
	public static final double Deadband = 0.1;

	// Constants for PID control
	public static final double DriveStraightKp = 0;
	public static final double DriveStraightKi = 0;
	public static final double DriveStraightKd = 0;
	public static final double DriveRotateKp = 0;
	public static final double DriveRotateKi = 0;
	public static final double DriveRotateKd = 0;
	public static final double LiftKp = 0.06;	// 適切に動くこと確認済み
	public static final double LiftKi = 0;
	public static final double LiftKd = 0;

	// Constants for limitting acceleration
	public static final double PIDLoopPeriod = 0.05;
	public static final double maxAcceleration = 0.1;

	//Constants for getting distance to the line
	public static final double Theta_Camera_rad = Math.toRadians(0);
	public static final double Theta_Angle_rad = Math.toRadians(0);
	public static final double cameraHeight = 0;
	
}
