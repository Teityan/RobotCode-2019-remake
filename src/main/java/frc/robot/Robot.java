/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.AnalogInput;

public class Robot extends TimedRobot {
    private State state;

    // Controller
    private XboxController driver, operator;

    // Motors
    private Spark driveLeft_1,driveLeft_2, driveRight_1,driveRight_2;
    private Talon liftMotor;
    private VictorSP rollerMotor;
    private VictorSP climbMotor;

    // Encoder, Gyro
    private Encoder rightDriveEncoder, leftDriveEncoder;
    private EncoderGroup driveEncoder;
    private Encoder liftEncoder;
    private ADXRS450_Gyro gyro;

    // Solenoid
    private Solenoid armSolenoid, barSolenoid;
    private Solenoid climbSolenoid;

    // Compressor
    private Compressor compressor;

    // Timer for climb
    private Timer climbTimer, backTimer, autonomousTimer;

    // SubModule
    private Drive drive;
    private Lift lift;
    private Grabber grabber;
    private Climb climb;

    private DigitalInput cargoSensor;

    // ライントレース用のセンサー　
    // 0～3.3V 白線があると電圧が上がる 
    //private AnalogInput rightFrontSensor, 
    //                  rightBackSensor, 
    //                  leftFrontSensor, 
    //                  leftBackSensor;

    // Camera
    //private CameraServer elevatorCamera, frameCamera;

    // NetWorkTable
    private NetworkTable networkTable;
    private double[] voltages;
    private int count=0;
     private static double kDt = 0.02;
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();




    // functions
    private double deadbandProcessing(double value) {
        return Math.abs(value) > Const.Deadband ? value : 0 ;
    }

    @Override
    public void robotInit() {
        // Controller
        /*
        XboxController controller_d = null;
        XboxController controller_o = null;
        for (int i=0;i>5;i++){
            if(new XboxController(i)!=null){
                if(controller_d==null){
                    controller_d = new XboxController(i);
                }else if(controller_o==null){
                    controller_o = new XboxController(i);
                }
            }
        }*/
        driver = new XboxController(Const.DriveControllerPort);
        operator = new XboxController(Const.OperateControllerPort);


        // Motors
        driveRight_1 = new Spark(7);
        driveRight_2 = new Spark(8);
        driveLeft_1 = new Spark(5);
        driveLeft_2 = new Spark(6);
        liftMotor = new Talon(Const.LiftMotorPort);

        rollerMotor = new VictorSP(Const.RollerMotorPort);

        climbMotor = new VictorSP(Const.ClimbMotorPort);

        rightDriveEncoder = new Encoder(Const.RightDriveEncoderAPort, Const.RightDriveEncoderBPort);
        leftDriveEncoder = new Encoder(Const.LeftDriveEncoderAPort, Const.LeftDriveEncoderBPort);
        driveEncoder = new EncoderGroup(rightDriveEncoder, leftDriveEncoder);    // エンコーダをまとめる   
        driveEncoder.setDistancePerPulse(Const.DriveEncoderDistancePerPulse);

        liftEncoder = new Encoder(Const.LiftEncoderAPort, Const.LiftEncoderBPort);
        liftEncoder.setDistancePerPulse(Const.LiftEncoderDistancePerPulse);

        gyro = new ADXRS450_Gyro();

        // Solenoid
        armSolenoid = new Solenoid(Const.ArmSolenoidPort);
        barSolenoid = new Solenoid(Const.BarSolenoidPort);

        climbSolenoid = new Solenoid(Const.ClimbSolenoidPort);

        // Compressor
        compressor = new Compressor();

        // Sensor
        //rightFrontSensor = new AnalogInput(Const.RightFrontSensorPort);
        //rightBackSensor = new AnalogInput(Const.RightBackSensorPort);
        //leftFrontSensor = new AnalogInput(Const.LeftFrontSensorPort);
        //leftBackSensor = new AnalogInput(Const.LeftBackSensorPort);
        //
        cargoSensor = new DigitalInput(Const.cargoSenosorPort);

        // Climb Timer
        climbTimer = new Timer();
        backTimer = new Timer();
        autonomousTimer = new Timer();


        // Camera

        //UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture("ElevatorCamera", 0);
        //UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture("FrameCamera", 1);
        //camera1.setResolution(1280,720);
        //camera2.setResolution(1280,720);

        // State
        state = new State();

        // Submodules
        drive = new Drive(new SpeedControllerGroup(driveLeft_1,driveLeft_2), new SpeedControllerGroup(driveRight_1,driveRight_2), driveEncoder, gyro);
        lift = new Lift(liftMotor, liftEncoder);
        grabber = new Grabber(rollerMotor, barSolenoid, armSolenoid);
        climb = new Climb(climbMotor, climbSolenoid);

    }

    @Override
    public void autonomousInit() {
        autonomousTimer.reset();
        autonomousTimer.start();
    }

    @Override
    public void autonomousPeriodic() {
        teleopPeriodic();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        
        state.stateInit();
        
        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("BateryVoltage",voltage);
      /*Init
         * これからコマンドやコントローラーで変数の値を変えてから代入する。
         * 操作しないときは出力を出したくないため、最初に出力を出さない状態で初期化する。
         */

        /*
         * Joystickやセンサーからの入力を受け取ってstate objectに値を流しこむ
         */
        /********** Drive ***********/
	        if (driver.getBumper(Hand.kLeft)) {
            state.driveState = State.DriveState.kLineTrace;
        } else if (driver.getBumper(Hand.kRight)) {
            state.driveState = State.DriveState.kCloseToLine;
        } 
        
            state.driveState = State.DriveState.kManual;
        /*    state.driveStraightSpeed = deadbandProcessing(-driver.getY(Hand.kLeft)*0.8);    // 入力が直感と逆
            state.driveRotateSpeed = deadbandProcessing(driver.getX(Hand.kRight)*0.8);
            state.climbMotorSpeed = deadbandProcessing(driver.getY(Hand.kRight)*0.8);
            SmartDashboard.putNumber("DriveStraightSpeed",-driver.getY(Hand.kLeft)/0.8);
            SmartDashboard.putNumber("DriverRouteSpeed",driver.getX(Hand.kRight)/0.8);
            SmartDashboard.putNumber("climbMortorSpeed",driver.getY(Hand.kRight)/0.8);
    */
    if(driver.getY(Hand.kLeft)>0.2){
        SmartDashboard.putNumber("in",driver.getY(Hand.kLeft));
    m_goal = new TrapezoidProfile.State(0,5);
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(0.02);
     SmartDashboard.putNumber("m_set",m_setpoint.position);
      SmartDashboard.putNumber("m_feed",m_feedforward.calculate(m_setpoint.velocity) / 12.0);
    // Send setpoint to offboard controller PID
    drive.setSetpoint(m_setpoint.position,
                        m_feedforward.calculate(m_setpoint.velocity) / 12.0);
    }
        if(driver.getBumper(Hand.kLeft)) {
            // 左のバンパーのボタンが押されたら低出力モード
            state.is_lowInputOn = true;
        }

        /********** Lift ***********/
        /**
         * Liftは全てoperatorが操作する
         */
        if(operator.getTriggerAxis(Hand.kLeft) > Const.Deadband){
            // LeftTriggerでCargoをロボットの外に出す
            state.liftSetpoint = Const.LaunchCargoHeight;
            state.is_liftPIDOn = true;
        }else if (operator.getYButton()) {
            // YでRocketの二段目のCargo
            state.liftSetpoint = Const.RocketSecondCargoHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;

            SmartDashboard.putString("Lift","RocketCago2");
        } else if (operator.getAButton()) {
            // AでRocketの一段目のCargo
            state.liftSetpoint = Const.RocketFirstCargoHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;

            SmartDashboard.putString("Lift","RocketCago1");
        } else if (operator.getBButton()) {
            // BでRocketCargo ShipのCargo
            state.liftSetpoint = Const.ShipCargoHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;
            SmartDashboard.putString("Lift","ShipCago");
        } else if (operator.getXButton()) {
            // XでRocketのの２段目のHatch
            state.liftSetpoint = Const.RocketSecondHatchHeight;
            state.is_liftPIDOn = true;
            state.is_lowInputOn = true;

            SmartDashboard.putString("Lift","RocketHatch2");
        } else if(driver.getYButton()) {
            // ドライバーのYボタンでHATCH回収
            state.liftSetpoint = Const.HoldPanelHeight;
            state.is_liftPIDOn = true;
        } else {
            // それ以外の場合は手動操作
            state.liftSpeed = deadbandProcessing(-operator.getY(Hand.kLeft));   // 直感と逆の入力
            state.is_liftPIDOn = false;

            if(lift.getHeight() > 30) {
                state.is_lowInputOn = true;
            }
        }


        state.is_liftPIDOnTarget = lift.is_PIDOnTarget();

        /*********** Grabber ***********/
        /**
         * Cargoを掴む部分
         * driverが操作する
         */

        // T/Fが逆転している
        if(!cargoSensor.get()) {
            state.is_holdingCargo = true;
        } else {
            state.is_holdingCargo = false;
        }

        if (driver.getTriggerAxis(Hand.kRight) > Const.Deadband) {
            // Right TriggerでCargoを掴む
            state.cargoState = State.CargoState.kHold;
        } else if (driver.getTriggerAxis(Hand.kLeft) > Const.Deadband) {
            // Left TriggerでCargoを離す
            state.cargoState = State.CargoState.kRelease;
        } else {
            // 普段はなにもしない
            state.cargoState = State.CargoState.kDoNothing;
        }

        if (driver.getBumper(Hand.kRight)) {
            // RightBumperで掴むところが縮む。
            state.is_toHoldPanel = false;
        } else {
            // 普段は掴むところが広がっている
            state.is_toHoldPanel = true;
        }

        if (operator.getBumper(Hand.kLeft)) {
            // Left BumberでArmをしまう
            state.is_toRetractArm = true;
        } else {
            state.is_toRetractArm = false;
        }

        /**
         * Climb
         *
         * クライムの流れ...
         * StartButton : HABの上に乗るまでの処理
         *    kDoNothing : kLiftUpと同じ。初期化した後はここからkLiftUpに行く。
         *    kLiftUp    : リフトを枠の、止めるところの少し上へ上げる。
         *    kLocking   : ストッパーを出す。ここでリフトが下がらないように維持する。
         *            　   出す命令をしてから即座にリフトを下げると止めるところに引っかからない可能性があるので0.3秒置く
         *    kAdvance   : リフトを下げて枠にひっかけてロボットを持ち上げる。
         *                 ロボットを持ち上げるのには-1.0くらいの入力が必要だが、最初からそれを入れるとアームに負担がかかるので手動でゆっくり入力する。
         *                 DriveBaseと枠についたモーターを動かして前に進み、HABに乗る。
         *
         *
         * BackButton : HABに乗っかった後の処理。本体が上に乗っかった後も枠がストッパーで下に押し付けられてるのでそれを外す。ストッパーがロックされてたら実行。
         *   kDoNothing  : kLiftUpと同じ。初期化した後はここからkLiftUpに行く。
         *   kLiftUp     : リフトを枠の、止めるところの少し上へ上げる。
         *   kUnlocking  : ストッパーを外す。ここでリフトが下がらないように維持する。
         *            　   外す命令をしてから即座にリフトを下げるとストッパーが外れない可能性があるので0.3秒置く。
         *   kLiftDown   : リフトを自重で下げさせる。枠がHABの手前の壁に引っかかってるかもしれないのでdrvierの右のスティックで枠のモーターを動かす。
         *
         *
         */

        if(operator.getStartButton()){
            SmartDashboard.putBoolean("Climb",true);
            // スタートボタンでクライムを始める
            state.is_autoClimbOn = true;
            state.is_lowInputOn = true;
            state.is_toRetractArm = false;

            switch(state.climbSequence) {
                case kDoNothing:
                case kLiftUp:

                    if(operator.getTriggerAxis(Hand.kRight) > Const.Deadband) {
                        // StartとRightTriggerでHABのLEVEL2までリフトを上げる
                        state.liftSetpoint = Const.HabSecondHeight;
                        state.is_liftPIDOn = true;
                    }else if(operator.getBumper(Hand.kRight)) {
                        // StartとRightBumperでHABのLEVEL3までリフトを上げる
                        state.liftSetpoint = Const.HabThirdHeight;
                        state.is_liftPIDOn = true;
                    }else{
                        //コマンドがなかったら抜ける
                        break;
                    }

                    if(state.is_liftPIDOnTarget) {
                        // 届いたらストッパーを出す
                        state.climbSequence = State.ClimbSequence.kLocking;
                        // ストッパーを出す時間を考慮して時間計る
                        climbTimer.reset();
                        climbTimer.start();
                    }
                    break;

                case kLocking:
                    // ストッパーを出す
                    state.is_lockingClimb = true;

                    // リフトが下がらないように維持する
                    state.liftSpeed = Const.KeepLiftHeightSpeed;



                    if(climbTimer.get() > 0.3) {
                        // 時間がたったら前に進む
                        state.climbSequence = State.ClimbSequence.kAdvance;
                        state.is_lockedClimb = true;
                        climbTimer.reset();
                    }
                    break;

                case kAdvance:
                    // スティックで前に進む
                    state.driveStraightSpeed = deadbandProcessing(-driver.getY(Hand.kLeft));

                    state.climbMotorSpeed = -state.driveStraightSpeed;

                    // コントローラー操作にてリフトを下げる。

                    // コンプレッサーを止めてできるだけ負担を減らす。
                    compressor.stop();
                    break;

                default:
                    break;
            }

        } else if(operator.getBackButton() && state.is_lockedClimb) {
            state.is_lowInputOn = true;
            state.is_toRetractArm = false;
            // 十分前に進んで後輪がHABに乗ったら実行
            switch(state.climbSequence) {
                case kDoNothing:
                case kLiftUp:

                    // 乗っかったら後処理
                    // リフトを上げる
                    if(operator.getTriggerAxis(Hand.kRight) > Const.Deadband) {
                        // StartとRightTriggerでHABのLEVEL2までリフトを上げる
                        state.liftSetpoint = Const.HabSecondHeight;
                        state.is_liftPIDOn = true;

                    }else if(operator.getBumper(Hand.kRight)) {
                        // StartとRightBumperでHABのLEVEL3までリフトを上げる
                        state.liftSetpoint = Const.FinalClimbHeight;
                        state.is_liftPIDOn = true;
                        state.climbMotorSpeed = -0.3;
                    }
                    /*
					if(lift.is_PIDOnTarget()){
						// 届いたらストッパーを外す
						state.climbSequence = State.ClimbSequence.kUnlocking;
						climbTimer.reset();
						climbTimer.start();
                    }	*/


                    state.is_liftPIDOn = true;

                    state.is_lockingClimb = false;

                    break;

                case kUnlocking:
                    // ストッパーを外す
                    state.is_lockingClimb = false;

                    // リフトが下がらないように維持する
                    state.liftSpeed = Const.KeepLiftHeightSpeed;

                    state.climbMotorSpeed = -1.0;

                    if(climbTimer.get() > 0.3) {
                        // 時間がたったらリフトを下げる
                        state.climbSequence = State.ClimbSequence.kLiftDown;
                    }
                    break;

                case kLiftDown:
                    // コントロール操作にてリフトを下げる

                    break;

                default:
                    break;
            }

        }else {
            state.is_autoClimbOn = false;
            // 初期化
            state.climbSequence = State.ClimbSequence.kDoNothing;
            compressor.start();
            SmartDashboard.putBoolean("Climb",false);
        }

        if(driver.getStartButton() && driver.getBackButton()) {
            if(!state.is_startCounting) {
                backTimer.reset();
                backTimer.start();
                state.is_startCounting = true;
            }
            if(backTimer.get() < 0.1) {
                state.driveStraightSpeed = -0.4;
                backTimer.reset();
                backTimer.start();
                state.is_startCounting = false;
            }
        } else {
            state.is_startCounting = false;
        }


        /*
         * Stateをapplyする
         */
        drive.applyState(state);
        lift.applyState(state);
        grabber.applyState(state);
        climb.applyState(state);
        if(state.is_lowInputOn){
            SmartDashboard.putBoolean("LowInputOn",true);
        }else{
            SmartDashboard.putBoolean("LowInputOn",false);
        }
    }

    public void robotPeriidic(){
        drive.printVariables();
        lift.printVariables();
        state.printVariables();


        SmartDashboard.putBoolean("cargo", state.is_holdingCargo);
    }
}
