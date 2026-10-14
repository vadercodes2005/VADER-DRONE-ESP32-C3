// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Flight control

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

#define PITCHRATE_P 0.05
#define PITCHRATE_I 0.2
#define PITCHRATE_D 0.001
#define PITCHRATE_I_LIM 0.3
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3
#define YAWRATE_I 0.0
#define YAWRATE_D 0.0
#define YAWRATE_I_LIM 0.3
#define ROLL_P 6
#define ROLL_I 0
#define ROLL_D 0
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P 3
#define PITCHRATE_MAX radians(360)
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300)
#define TILT_MAX radians(30)
#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

// --- ALTITUDE HOLD GAINS ---
#define ALT_BUFFER_MM 50.0f // 5 cm deadband buffer
#define ALT_KP 0.0005f      // Proportional gain (distance error)
#define ALT_KD 0.0002f      // Derivative gain (velocity braking)
// ---------------------------

const int RAW = 0, ACRO = 1, STAB = 2, AUTO = 3; // flight modes
int mode = STAB;
bool armed = false;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;
int flightModes[] = {STAB, STAB, STAB}; // map for rc mode switch

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;

// --- DELTA-THROTTLE GLOBALS ---
extern float laser_alt_mm;
extern float laser_vel_mms;
float last_controlThrottle = 0.0f;
float target_altitude_mm = 0.0f;
float hover_base_thrust = 0.0f;
bool alt_hold_active = false;
unsigned long throttle_settle_timer = 0;
// ------------------------------

void control() {
	interpretControls();
	failsafe();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	if (controlMode < 0.25) mode = flightModes[0];
	else if (controlMode <= 0.75) mode = flightModes[1];
	else if (controlMode > 0.75) mode = flightModes[2];

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

	if (controlThrottle < 0.05 && controlYaw > 0.95) armed = true; // arm gesture
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

	// --- DELTA-THROTTLE ALTITUDE LOCK WITH PD BRAKING ---
	if (mode == STAB && armed && controlThrottle > 0.1f) {
		// 1. Detect if the virtual stick is moving
		if (abs(controlThrottle - last_controlThrottle) > 0.005f) { 
			alt_hold_active = false;
			last_controlThrottle = controlThrottle;
			throttle_settle_timer = millis(); 
			thrustTarget = controlThrottle; // Pilot is flying manually
		} 
		// 2. The stick has stopped moving
		else {
			// Wait 300ms for physical momentum to settle before locking height
			if (!alt_hold_active && (millis() - throttle_settle_timer > 300)) {
				target_altitude_mm = laser_alt_mm;
				hover_base_thrust = controlThrottle;
				alt_hold_active = true;
			}

			if (alt_hold_active) {
				float error_mm = target_altitude_mm - laser_alt_mm;
				float d_term = -laser_vel_mms * ALT_KD; // Opposes vertical speed
				
				// 3. Outside the 50 mm (5 cm) Buffer Zone: Full P + D control
				if (abs(error_mm) > ALT_BUFFER_MM) {
					float p_term = error_mm * ALT_KP;
					float pd_adjustment = constrain(p_term + d_term, -0.25f, 0.25f);
					thrustTarget = hover_base_thrust + pd_adjustment; 
				} 
				// 4. Inside the 50 mm Buffer Zone: Brake fast momentum, ignore small noise
				else {
					if (abs(laser_vel_mms) > 80.0f) {
						// Moving faster than 8 cm/s through the buffer: apply D-brake
						thrustTarget = hover_base_thrust + constrain(d_term, -0.15f, 0.15f);
					} else {
						// Settled inside buffer: hold steady base thrust
						thrustTarget = hover_base_thrust;
					}
				}
			} else {
				thrustTarget = controlThrottle;
			}
		}
		thrustTarget = constrain(thrustTarget, 0.1f, 1.0f);
	} else {
		// Disarmed, landing, ACRO, or RAW: Normal 1:1 pass-through
		thrustTarget = controlThrottle;
		last_controlThrottle = controlThrottle;
		alt_hold_active = false;
	}
	// ----------------------------------------------------

	if (mode == STAB) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw(); // reset yaw target
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax, controlPitch * tiltMax, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z; // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == RAW) { // direct torque control
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.invalidate(); // skip rate control
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1;
	}
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < 0.1) return; // skip attitude control

	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}

void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < 0.1) return; // skip rates control

	Vector error = ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
}

void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		memset(motors, 0, sizeof(motors)); // stop motors if disarmed
		return;
	}

	if (thrustTarget < 0.1) {
		motors[0] = 0.1; // idle thrust
		motors[1] = 0.1;
		motors[2] = 0.1;
		motors[3] = 0.1;
		return;
	}

	motors[MOTOR_FRONT_LEFT] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	desaturate(motors[MOTOR_FRONT_LEFT], motors[MOTOR_FRONT_RIGHT], motors[MOTOR_REAR_LEFT], motors[MOTOR_REAR_RIGHT]);

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}

void desaturate(float& a, float& b, float& c, float& d) {
	float maxThrust = max(max(a, b), max(c, d));
	if (maxThrust > 1) {
		float diff = maxThrust - 1;
		a -= diff;
		b -= diff;
		c -= diff;
		d -= diff;
	}
}

const char* getModeName() {
	switch (mode) {
		case RAW: return "RAW";
		case ACRO: return "ACRO";
		case STAB: return "STAB";
		case AUTO: return "AUTO";
		default: return "UNKNOWN";
	}
}