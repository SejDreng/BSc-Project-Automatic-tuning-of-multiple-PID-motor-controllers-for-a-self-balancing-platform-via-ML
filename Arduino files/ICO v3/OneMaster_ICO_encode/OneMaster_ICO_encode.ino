#include <Wire.h>
#include <DFRobot_BMX160.h>

DFRobot_BMX160 bmx160;
long encoder_position = 0; // Not volatile if only accessed in loop()
int prevA0_state;
int prevA1_state;

// TUNABLE VALUES
const float dt_calc = 0.03f;                               // Time step for calculations (seconds)
const unsigned long ENCODER_READ_INTERVAL_MS = 1;         // Read encoder every 1 ms
const unsigned long CONTROL_LOOP_INTERVAL_MS = 30;       // Run PID etc. every 10 ms 
const int NUM_JOINTS = 2;                                 // Keep as 1 if using single obsAngle
const float mu = 0.0000001f;  
const float w0 = 1;
const float max_change_per_cycle = 30.5f;                 // Tune this value for maximum allowed control signal change pr iteration.

// Poisson Filter generated in matlab (ensure it's the same as used in MATLAB script if a comparisson if simulation comparison is wished)
const int FILTER_LEN = 101;
const float H_poisson[FILTER_LEN] = {
    0.00000000f, 0.01062212f, 0.01680249f, 0.02029530f, 0.02216432f, 
    0.02305463f, 0.02335732f, 0.02330947f, 0.02305472f, 0.02268005f, 
    0.02223807f, 0.02176059f, 0.02126673f, 0.02076798f, 0.02027118f, 
    0.01978037f, 0.01929787f, 0.01882497f, 0.01836234f, 0.01791028f, 
    0.01746887f, 0.01703804f, 0.01661767f, 0.01620755f, 0.01580749f, 
    0.01541727f, 0.01503666f, 0.01466542f, 0.01430335f, 0.01395020f, 
    0.01360578f, 0.01326985f, 0.01294222f, 0.01262268f, 0.01231102f, 
    0.01200706f, 0.01171061f, 0.01142147f, 0.01113948f, 0.01086444f, 
    0.01059620f, 0.01033458f, 0.01007942f, 0.00983055f, 0.00958784f, 
    0.00935111f, 0.00912023f, 0.00889505f, 0.00867543f, 0.00846124f, 
    0.00825233f, 0.00804858f, 0.00784986f, 0.00765604f, 0.00746701f, 
    0.00728265f, 0.00710284f, 0.00692747f, 0.00675643f, 0.00658962f, 
    0.00642692f, 0.00626824f, 0.00611347f, 0.00596253f, 0.00581532f, 
    0.00567174f, 0.00553170f, 0.00539512f, 0.00526192f, 0.00513200f, 
    0.00500529f, 0.00488171f, 0.00476118f, 0.00464362f, 0.00452897f, 
    0.00441715f, 0.00430809f, 0.00420173f, 0.00409798f, 0.00399680f, 
    0.00389812f, 0.00380188f, 0.00370801f, 0.00361646f, 0.00352717f, 
    0.00344008f, 0.00335515f, 0.00327231f, 0.00319151f, 0.00311271f, 
    0.00303586f, 0.00296091f, 0.00288780f, 0.00281650f, 0.00274696f, 
    0.00267914f, 0.00261299f, 0.00254848f, 0.00248555f, 0.00242418f, 
    0.00236433f
};

unsigned long lastEncoderReadTime = 0;
unsigned long lastControlLoopTime = 0;

static const int8_t ENCODER_TRANSITIONS[16] = {
    // prevA prevB currA currB | Effect   | Transition
    // ----- ----- ----- ----- | -------- | ----------
    /* 0000 */ 0,              // 00 -> 00 | No change
    /* 0001 */ -1,             // 00 -> 01 | CCW
    /* 0010 */ 1,              // 00 -> 10 | CW
    /* 0011 */ 0,              // 00 -> 11 | Error (skip) -> No change
    /* 0100 */ 1,              // 01 -> 00 | CW
    /* 0101 */ 0,              // 01 -> 01 | No change
    /* 0110 */ 0,              // 01 -> 10 | Error (skip) -> No change
    /* 0111 */ -1,             // 01 -> 11 | CCW
    /* 1000 */ -1,             // 10 -> 00 | CCW
    /* 1001 */ 0,              // 10 -> 01 | Error (skip) -> No change
    /* 1010 */ 0,              // 10 -> 10 | No change
    /* 1011 */ 1,              // 10 -> 11 | CW
    /* 1100 */ 0,              // 11 -> 00 | Error (skip) -> No change
    /* 1101 */ 1,              // 11 -> 01 | CW
    /* 1110 */ -1,             // 11 -> 10 | CCW
    /* 1111 */ 0               // 11 -> 11 | No change
};

//I2C package struct for slave transmission
struct controlPacket {
  float typeID;
  float obsAngle;
  float reference;
  float kp;
  float ki;
  float kd;
  float error;
  float integral; 
};

// ADD MORE STRUCTS IF USING MORE MOTORS
controlPacket transf_outer = {1, 0.0, 0.0, 0.0, 0.0, 0.0};
controlPacket transf_inner = {1, 0.0, 0.0, 0.0, 0.0, 0.0};

          
float target_angle[NUM_JOINTS];                       // Target angle for each joint
const float angle_tolerance_deg = 8.0f;               // degrees

float error[NUM_JOINTS];
float prev_error[NUM_JOINTS];
float X0[NUM_JOINTS];
float X0_prev[NUM_JOINTS];                            // For ICO derivative input
float integral[NUM_JOINTS];
float U[NUM_JOINTS];                                  // simulated Control output U for claming and saturation 
float prev_U[NUM_JOINTS];

float derivativeBuffer[NUM_JOINTS][FILTER_LEN];       // Stores ico_filter_input history
int bufferWriteIndex[NUM_JOINTS];
float filteredDerivative[NUM_JOINTS];                 // Output of the FIR filter

float X1_kp[NUM_JOINTS];
// float dw1_kp[NUM_JOINTS]; // Can be local
float w1_kp[NUM_JOINTS];
float Kp[NUM_JOINTS];

float X1_ki[NUM_JOINTS];
// float dw1_ki[NUM_JOINTS]; // Can be local
float w1_ki[NUM_JOINTS];
float Ki[NUM_JOINTS];

float X1_kd[NUM_JOINTS];
// float dw1_kd[NUM_JOINTS]; // Can be local
float w1_kd[NUM_JOINTS];
float Kd[NUM_JOINTS];

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial); // Wait for Serial port to connect (for some Arduinos)

  Serial.println("Initializing...");

  if (bmx160.begin() != true) {
    Serial.println("BMX160 init false");
    while (1);
  }
  bmx160.wakeUp(); // Ensure sensor is active
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  prevA0_state = digitalRead(A0);
  prevA1_state = digitalRead(A1);

  Serial.println("Initializing Filter and PID variables...");

  for (int i = 0; i < NUM_JOINTS; ++i) {
    target_angle[i] = 0.0f; // Set initial target, can be changed later

    // Initialize w1 terms to 1.0 as in MATLAB
    w1_kp[i] = 1.0f;
    w1_ki[i] = 1.0f;
    w1_kd[i] = 1.0f;
     
    // Initial Kp, Ki, Kd will be calculated
    Kp[i] = 0.0f;
    Ki[i] = 0.0f;
    Kd[i] = 0.0f;

    // Initialize the control params
    prev_error[i] = 0.0f; 
    X0_prev[i] = 0.0f;
    prev_U[i] = 0.0f;
    integral[i] = 0.0f;
    // error[i] will be calculated in the first loop pass.

    // Configure the filter buffers
    filteredDerivative[i] = 0.0f;
    bufferWriteIndex[i] = 0;
    for (int j = 0; j < FILTER_LEN; ++j) {
      derivativeBuffer[i][j] = 0.0f;
    }
  }
  Serial.println("Initialization Complete. Starting loop.");
  delay(1000); // Give a moment before loop starts
}

void loop() {

  // TO ensure maximum encoder accuracy, it is run every 1ms independently of the main control loop, which is goverened by a set sampling time.
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderReadTime >= ENCODER_READ_INTERVAL_MS) {
    lastEncoderReadTime = currentTime;
    readEncoderNonBlocking();
  }
  if (currentTime - lastControlLoopTime >= CONTROL_LOOP_INTERVAL_MS) {
    lastControlLoopTime = currentTime;

    sBmx160SensorData_t Omagn, Ogyro, Oaccel;
    bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);

    transf_outer.obsAngle = atan2(Oaccel.y, Oaccel.z) * RAD_TO_DEG;
    transf_outer.reference = target_angle[0];

    // Calculate Kp, Ki, Kd using ICO
    ICO_PID();

    // Configure I2C structs for transmission
    transf_inner.kp = Kp[0];
    transf_inner.ki = Ki[0];
    transf_inner.kd = Kd[0];
    transf_inner.error = error[0];
    transf_inner.integral = integral[0];

    transf_outer.kp = Kp[1];
    transf_outer.ki = Ki[1];
    transf_outer.kd = Kd[1];
    transf_outer.error = error[1];
    transf_outer.integral = integral[1];

    // Send to slaves
    Wire.beginTransmission(1);
    Wire.write((byte*)&transf_outer, sizeof(controlPacket));
    Wire.endTransmission();

    Wire.beginTransmission(2);
    Wire.write((byte*)&transf_inner, sizeof(controlPacket));
    Wire.endTransmission();
  }
}  

void readEncoderNonBlocking() {
    // Read the current state of the encoder pins
    int currentA0_state = digitalRead(A0); // Read digital signal from pin A0
    int currentA1_state = digitalRead(A1); // Read digital signal from pin A1

    // Create an index for the state transition table.
    // The index is a 4-bit number representing the transition:
    // Bit 3: prevA0_state
    // Bit 2: prevA1_state
    // Bit 1: currentA0_state
    // Bit 0: currentA1_state
    uint8_t transition_index = (prevA0_state << 3) | (prevA1_state << 2) | (currentA0_state << 1) | currentA1_state;

    // Look up the change in position from the table
    int8_t position_change = ENCODER_TRANSITIONS[transition_index];

    // Update the encoder position
    encoder_position += position_change;

    // Update the previous states for the next call
    prevA0_state = currentA0_state;
    prevA1_state = currentA1_state;
}

void ICO_PID() {
  for (int i = 0; i < NUM_JOINTS; ++i) { // Loop for each joint (here, NUM_JOINTS = 1)
    // 1. Calculate Error (target - current)
    // If NUM_JOINTS > 1, obsAngle needs to be an array or fetched per joint.
    float current_obs_angle = (i == 0) ? transf_outer.obsAngle + encoder_position: transf_outer.obsAngle; //This is currently configured for two joints, should add a commulative encoder dynamic for more joints
    float raw_error = target_angle[i] - current_obs_angle;

    // Apply tolerance: if within, treat as zero error
    if (fabs(raw_error) <= angle_tolerance_deg) {
        error[i] = 0.0f;
    } else {
        error[i] = raw_error / 90.0f; // Normalized for consistent controller gain scaling
    }
    X0[i] = prev_error[i];

    // Remove small inacuracies from the IMU
    if(abs(error[i] - prev_error[i]) <= 0.01){
      error[i] = prev_error[i];
    }

    // 2. Calculate Raw Derivative Term (dError/dt = (error - prev_error) / dt)
    float current_raw_derivative = (error[i] - prev_error[i]) / dt_calc;

    // 3. Calculate ICO Filter Input ((prev_error - prev_prev_error) / dt)
    float ico_filter_input = (X0[i] - X0_prev[i]) / dt_calc;

    U[i] = Kp[i] * error[i] + Ki[i] * integral[i] + Kd[i] * current_raw_derivative;

    float change = U[i] - prev_U[i];
    if (change > max_change_per_cycle) {
      U[i] = prev_U[i] + max_change_per_cycle;
    } else if (change < -max_change_per_cycle) {
      U[i] = prev_U[i] - max_change_per_cycle;
    }
    prev_U[i] = U[i]; // Store for next iteration
    
    // Apply Saturation to U (as in MATLAB example)
    const float U_MAX = 255; // Example, match your system's maximum
    if (U[i] > U_MAX) {
      U[i] = U_MAX;
    } else if (U[i] < -U_MAX) {
      U[i] = -U_MAX;
    }

    // 4. Calculate Integral Term (Integral = Integral + error * dt)
    // Anti-windup (Clamper) Logic from MATLAB
    bool is_saturated = fabs(U[i]) >= U_MAX; // Check against actual saturation limit
    bool signs_match = (error[i] * U[i] > 0); // Simpler check for sign match (both positive or both negative)
                                               // Note: if error or U is zero, this is false.
                                               // For strict sign match: (error[i]>0 && U[i]>0) || (error[i]<0 && U[i]<0)
    bool is_winding_up = signs_match; 
    bool is_error_growing = fabs(error[i]) > fabs(prev_error[i]);
    
    float clamper_value = 1.0f;

    if ((is_saturated && is_winding_up && is_error_growing)) {
        clamper_value = 0.0f;
    }

    integral[i] += error[i] * dt_calc * clamper_value;

    // 5. Update Derivative Buffer and Apply FIR Filter for filteredDerivative
    // --- FIR Filter Section ---
    // Store the newest ico_filter_input (x[n]) into the buffer
    derivativeBuffer[i][bufferWriteIndex[i]] = ico_filter_input;
    
    // To match MATLAB's conv(A,H,'same')(end) behavior, the filter effectively
    // calculates sum H_poisson[j] * x[n-1-j].
    // The "newest" sample for this sum's perspective is x[n-1].
    // bufferWriteIndex[i] currently points to x[n].
    // So, (bufferWriteIndex[i] - 1 + FILTER_LEN) % FILTER_LEN points to x[n-1].
    int idx_base_for_sum = (bufferWriteIndex[i] - 1 + FILTER_LEN) % FILTER_LEN;

    filteredDerivative[i] = 0.0f;
    for (int j_H = 0; j_H < FILTER_LEN; ++j_H) { // j_H is the tap index for H_poisson
        int sampleIndex = (idx_base_for_sum - j_H + FILTER_LEN) % FILTER_LEN;
        filteredDerivative[i] += H_poisson[j_H] * derivativeBuffer[i][sampleIndex];
    }
    
    // Advance buffer write index for the next cycle AFTER using its current value as x[n]'s location
    bufferWriteIndex[i] = (bufferWriteIndex[i] + 1) % FILTER_LEN;
    // --- End FIR Filter Section ---

    // 6. ICO for Kp
    X1_kp[i] = error[i];
    float dw1_kp = mu * (X1_kp[i] * filteredDerivative[i]);
    w1_kp[i] += dw1_kp;
    Kp[i] = ((X0[i] * w0) + (X1_kp[i] * w1_kp[i]));

    // 7. ICO for Ki
    X1_ki[i] = error[i] * integral[i];
    float dw1_ki = -mu * (X1_ki[i] * filteredDerivative[i]);
    w1_ki[i] += dw1_ki;
    Ki[i] = ((X0[i] * w0) + (X1_ki[i] * w1_ki[i]));

    // 8. ICO for Kd
    X1_kd[i] = error[i]*current_raw_derivative; // Using the raw derivative of error
    float dw1_kd = mu * (X1_kd[i] * filteredDerivative[i]);
    w1_kd[i] += dw1_kd;
    Kd[i] = ((X0[i] * w0) + (X1_kd[i] * w1_kd[i]));

    // 9. Update error history for next iteration
    X0_prev[i] = X0[i];
    prev_error[i] = error[i];
    
  }
}
