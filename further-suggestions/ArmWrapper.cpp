#include "WPILib.h"
#include "ArmWrapper.h"

// ----- Constructor -----

// With Object
ArmWrapper::ArmWrapper (SpeedController* lArm, SpeedController* rArm, Encoder* armAngle, DigitalInput* armLimSwitch) {
	m_lArm = lArm;
	m_rArm = rArm;
	m_armAngle = armAngle;
	m_armLimSwitch = armLimSwitch;
	
	// ----- Set Conf -----
	c_distPerPulse = 1.0;
	c_maxPeriod = 1.0;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
	m_armAngle->SetMaxPeriod(c_maxPeriod);
}

// With Channel
ArmWrapper::ArmWrapper(int lArm, int rArm, int armAngle1, int armAngle2, int armLimSwitch) {
	m_lArm = new Talon(lArm);
	m_rArm = new Talon(rArm);
	m_armAngle = new Encoder(armAngle1, armAngle2, true);
	m_armLimSwitch = new DigitalInput(armLimSwitch);
	
	// ----- Set Conf -----
	c_distPerPulse = 1.0;
	c_maxPeriod = 1.0;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
	m_armAngle->SetMaxPeriod(c_maxPeriod);
}

// With Channel and reverse direction
ArmWrapper::ArmWrapper(int lArm, int rArm, int armAngle1, int armAngle2, bool reverse, int armLimSwitch) {
	m_lArm = new Talon(lArm);
	m_rArm = new Talon(rArm);
	m_armAngle = new Encoder(armAngle1, armAngle2, reverse);
	m_armLimSwitch = new DigitalInput(armLimSwitch);
	
	// ----- Set Conf -----
	c_distPerPulse = 1.0;
	c_maxPeriod = 1.0;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
	m_armAngle->SetMaxPeriod(c_maxPeriod);
}

// ----- Control Motors -----
void ArmWrapper::Set (float speed) {
	m_lArm->Set (speed);
	m_rArm->Set (speed);
}

// ----- PID -----
void ArmWrapper::StartPID (float p, float i, float d) {
	PID = new PIDController (p, i, d, m_armAngle, this);
}

void ArmWrapper::SetAngle (float angle) {
	PID->SetSetpoint(angle);
}

void ArmWrapper::PIDEnable () {
	PID->Enable();
}

void ArmWrapper::PIDDisable () {
	PID->Disable();
}

void ArmWrapper::PIDWrite(float output) {
	Set(output);
}

// ----- Control Encoder -----
void ArmWrapper::Reset() {
	m_armAngle->Reset();
}

void ArmWrapper::Start() {
	m_armAngle->Start();
}

void ArmWrapper::Stop () {
	m_armAngle->Stop();
}

// ----- Get Values -----
int ArmWrapper::GetRawAngle () {
	return m_armAngle->Get();
}

double ArmWrapper::GetAngle () {
	return (double)m_armAngle->Get() * c_distPerPulse;
}

double ArmWrapper::GetSpeed() {
	if (m_armAngle->GetDirection()) {
		return c_distPerPulse / m_armAngle->GetPeriod();
	} else {
		return - c_distPerPulse / m_armAngle->GetPeriod();
	}
}

bool ArmWrapper::GetLimSwitch() {
	return m_armLimSwitch->Get();
}

// ----- Conf -----
void ArmWrapper::SetDistPerPulse(double distPerPulse) {
	c_distPerPulse = distPerPulse;
	m_armAngle->SetDistancePerPulse(c_distPerPulse);
}

void ArmWrapper::SetMaxPeriod(double maxPeriod) {
	c_maxPeriod = maxPeriod;
	m_armAngle->SetMaxPeriod(c_maxPeriod);
}

double ArmWrapper::GetDistPerPulse () {
	return c_distPerPulse;
}

double ArmWrapper::GetMaxPeriod () {
	return c_maxPeriod;
}
