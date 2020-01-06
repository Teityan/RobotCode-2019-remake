/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends DifferentialDrive{

    private enum PIDMode {
		Straight, Rotate, Default
	};

	private double straightOutput, rotateOutput;
	private double preStraightOutput,preRotateOutput;

	private EncoderGroup e_drive;
	private ADXRS450_Gyro g_drive;

	private PIDController straightController, rotateController;
	
	
 
	public Drive(SpeedController leftMotor, SpeedController rightMotor, EncoderGroup e_drive, ADXRS450_Gyro g_drive) {
		super(leftMotor, rightMotor);
		this.e_drive = e_drive;
		this.g_drive = g_drive;

		straightController = new PIDController(Const.DriveStraightKp, Const.DriveStraightKi, Const.DriveStraightKd, e_drive,
				new DrivePIDOutput(PIDMode.Straight));
		rotateController = new PIDController(Const.DriveRotateKp, Const.DriveRotateKi, Const.DriveRotateKd, g_drive,
				new DrivePIDOutput(PIDMode.Rotate));
	}

	public void applyState(State state) {
		
		switch(state.driveState) {
			case kManual:
				PIDDisable();
				setSpeed(state.driveStraightSpeed, state.driveRotateSpeed, state.is_lowInputOn);	
				break;

			case kLineTrace:
				PIDDisable();
				// lineTrace();
				break;

			case kCloseToLine:
				setSetpoint(state.driveStraightSetpoint, state.driveRotateSetpoint);// PID制御
				PIDEnable();
				break;
			
			default:
				break;
		}
	}

	private void setSpeed(double straightSpeed, double rotateSpeed){
		setSpeed(straightSpeed, rotateSpeed, false);
	}

	private void setSpeed(double straightSpeed, double rotateSpeed, boolean is_lowInputOn) {
		//低出力モード
		straightSpeed *= is_lowInputOn ? 0.5 : 1;
		rotateSpeed *= is_lowInputOn ? 0.5 : 1;

		// 加速度制限
		straightOutput = limitAcceleration(preStraightOutput, straightOutput);
		rotateOutput = limitAcceleration(preRotateOutput, rotateOutput);

		preStraightOutput = straightOutput;
		preRotateOutput = rotateOutput;
		
		arcadeDrive(straightSpeed, rotateSpeed);
	}
	

	private void setRelativeStraightSetpoint(double setpoint) {
		setRelativeSetpoint(e_drive.getDistance() + setpoint, g_drive.getAngle());
	}

	private void setRelativeTurnSetpoint(double setpoint) {
		setRelativeSetpoint(e_drive.getDistance(), g_drive.getAngle() + setpoint);
	}

	private void setRelativeSetpoint(double straightSetpoint, double turnSetpoint) {
		straightController.setSetpoint(e_drive.getDistance() + straightSetpoint);
		rotateController.setSetpoint(g_drive.getAngle() + turnSetpoint);
    }
    
    private void setStraightSetpoint(double straightSetpoint) {
        setSetpoint(straightSetpoint, 0);
    }

    private void setTurnSetpoint(double turnSetpoint) {
        setSetpoint(0, turnSetpoint);
    }

    public void setSetpoint(double straightSetpoint, double turnSetpoint) {
        straightController.setSetpoint(straightSetpoint);
		rotateController.setSetpoint(turnSetpoint);
    }


	private void PIDEnable() {
		if (!straightController.isEnabled()) {
			straightController.enable();
		}
		if (!rotateController.isEnabled()) {
			rotateController.enable();
		}
	}

	private void PIDDisable() {
		if (straightController.isEnabled()) {
			straightController.disable();
			straightController.reset();
		}
		if (rotateController.isEnabled()) {
			rotateController.disable();
			rotateController.reset();
		}
	}

	private boolean is_PIDEnabled() {
		return straightController.isEnabled() && rotateController.isEnabled();
	}
		
	public void setStraightP(double p) {
		straightController.setP(p);
	}

	public void setStraightI(double i) {
		straightController.setI(i);
	}

	public void setStraightD(double d) {
		straightController.setD(d);
	}

	public void setRotateP(double p) {
		rotateController.setP(p);
	}

	public void setRotateI(double i) {
		rotateController.setI(i);
	}

	public void setRotateD(double d) {
		rotateController.setD(d);
	}

	private double limitAcceleration(double preOutput, double output) {
		if(preOutput == output) {
			// 0除算回避
			return output;
		}

		double acceleration = (output - preOutput) / Const.PIDLoopPeriod;
		if(acceleration * preOutput < 0 && acceleration * output < 0) {
			// 0へ向かう加速度だったらそのまま
			return output;
		}
		// 制限
		acceleration =  Math.signum(acceleration) * Math.min(Math.abs(acceleration), Const.maxAcceleration);

		output = preOutput + acceleration * Const.PIDLoopPeriod;

		return output;
	}
    
    public void printVariables() {
		SmartDashboard.putNumber("straightOutput", straightOutput);
		SmartDashboard.putNumber("rotateOutput", rotateOutput);
		SmartDashboard.putNumber("e_drive.getDistance()", e_drive.getDistance());
		SmartDashboard.putNumber("g_drive.getAngle()", g_drive.getAngle());

		SmartDashboard.putNumber("straightController.getP()", straightController.getP());
        SmartDashboard.putNumber("straightController.getI()", straightController.getI());
		SmartDashboard.putNumber("straightController.getD()", straightController.getD());
		SmartDashboard.putNumber("rotateController.getP()", rotateController.getP());
        SmartDashboard.putNumber("rotateController.getI()", rotateController.getI());
        SmartDashboard.putNumber("rotateController.getD()", rotateController.getD());
    }

	public class DrivePIDOutput implements PIDOutput {

		PIDMode m_pid = PIDMode.Default;

		DrivePIDOutput(PIDMode m_pid) {
			this.m_pid = m_pid;
		}

		public void pidWrite(double output) {

			switch (m_pid) {
			case Straight:
				straightOutput = output;
				break;

			case Rotate:
				rotateOutput = output;
				break;

			case Default:
			default:
			}

			// Straightは前向きがマイナス
			setSpeed(-straightOutput, rotateOutput);			
		}

	}


}

/*　関数一覧
Drive(SpeedController leftMotor, SpeedController rightMotor, EncoderGroup e_drive, ADXRS450_Gyro g_drive)
	DifferentialDrive要素のSpeedController、PIDSource用のEnocoderGroupとADXRS450_Gyroを受け取り、PIDControllerのインスタンスを作る


applyState(State state)
	stateから情報を受け取り入力する

setSpeed(double straightSpeed, double rotateSpeed)
	arcadeDrive()に代入する
	足元を動かしたくないときはここの処理をコメントアウトすればよい

setRelativeStraightSetpoint(double setpoint)
setRelativeTurnSetpoint(double setpoint)
setRelativeSetpoint(double straightSetpoint, double turnSetpoint)
	Setpointを相対位置で代入する
	 * 本来のsetSetPointでは現在位置からではなくEncoder系を作動させた時の初期位置からの距離、角度を指定する必要がある。
	 * PIDControllerのcalculate()をみるとinputにpidGet()をいれ、setSetpoint()でsetされた目標値との偏差をとっている。
	 * つまり、何も考えずに入れるとEncoder系を作動させた時の初期位置からのsetpointになり、思うように動作しない。
	 * 例えば、1ｍ前進した後に2ｍ後進しようとしてsetSetpoint(-2000)などとすると実際には3ｍ後進してしまう。
	 * だから、setSetpoint(getDistance()(=1000) + setpoint(=-2000))とする。
	 * しかし、少し扱いづらいので"Relative"(=相対)にして見えないところで処理する。
	 
setStraightSetpoint(double straightSetpoint)
setTurnSetpoint(double turnSetpoint)
setSetpoint(double straightSetpoint, double turnSetpoint)
	Setpointを絶対座標で代入する

PIDEnable()
	二つのPIDControllerをenableにする
PIDDisable()
	二つのPIDControllerをdisableにする
is_PIDEnabled()
	二つのPIDControllerがenableかどうか

*/
/*メンバークラス---DrivePIDOutput　PIDOutput継承
  その関数 
	DrivePIDOutput(PIDMode pidmode)
		PIDModeによって後述のpidWrite()の動作が変わる。

	pidWrite(double output)
		outputを受け取り処理して代入する。PIDModeによって代入対象が異なる。
*/


/*
追加で
setStraightP,I,D(double p,i,d);
setTurnP,I,D(double p,i,d);
    ゲインの調整をしやすくするため

isMoving();
	進んだり回ったりしているか
	Encoderのしきい値よく考える。

PIDReset();
	PIDのリセット

*/

/*
	setLineTraceSetpoint(double setpoint);
		引数はなくてもいいかもしれない。
	lineTracePIDEnalble();
		ライントレースする。普通のPID制御をDisableにする。(PIDEnableも修正)
	
*/

/*修正
	lineTraceによるPIDEnable修正, コンストラクタの引数の追加、PIDControllerのインスタンス生成, PIDOutputのEnum及びswitch文内に追加, 
*/
