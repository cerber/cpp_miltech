#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <cmath>

const int MAX_STEPS = 10000;

const int MAX_AMMO_NAME = 16;
const int TARGETS_COUNT = 5;
const int TARGETS_POINTS = 60;

#define DEBUG 1

using namespace std;

// The drone states
enum droneStates {
    EN_DS_STOPPED,      // The drone is stationary
    EN_DS_ACCELERATING, // The drone is accelerating towards the target
    EN_DS_DECELERATING, // The drone is decelerating towards the target
    EN_DS_TURNING,      // The drone is turning
    EN_DS_MOVING,       // The drone is moving towards the target
    EN_DS_FIRING        // The drone is firing at the target
};

int targetInterpolation(const int target, 
    const float targetXInTime[TARGETS_COUNT][TARGETS_POINTS], 
    const float targetYInTime[TARGETS_COUNT][TARGETS_POINTS], 
    const float targetTimeStep, const float simTime,
    float &targetX, float &targetY);
int calcBallistic(float xd, float yd, float zd, float targetX, float targetY, 
    float attackSpeed, float accelerationPath, float m, float d, float l,
    float &fireX, float &fireY);
int simulationStep(droneStates &state, float directionToTarget, float distanceToTarget, 
    float turnThreshold, float hitRadius, float angularSpeed, 
    float acceleration, float attackSpeed, float simTimeStep, 
    float &direction, float &currentSpeed, float &xd, float &yd); 

float estimateTimeToTarget(float , float acceleration, float speed, float speedAttack, 
    float delta, float threshold, float angularSpeed, float hitRadius);
float accelMotionPath(float v0, float a, float t);
float turningTime(float delta, float angularSpeed);
float movingTime(float distance, float speed);
float acceleratingTime(float v0, float a, float targetSpeed);
float distance(float x1, float y1, float x2, float y2);

int main() {
    // Open the input file
    ifstream inputFile("input.txt");
    if (!inputFile) {
        cerr << "Error opening file!" << endl;
        return 1;
    }

    // Initialize variables for prevent the side-effects
    // initial coordinates of the drone
    float xd{0.0f}, yd{0.0f}, zd{0.0f};
    inputFile >> xd >> yd >> zd;

    // initial direction of the drone
    float direction{0.0f};
    inputFile >> direction;

    // attack speed of the drone
    float attackSpeed{0.0f};
    inputFile >> attackSpeed;

    // acceleration path of the drone
    float accelerationPath{0.0f};
    inputFile >> accelerationPath;

    // ammunition name
    char ammoName[MAX_AMMO_NAME]{"UNDEFINED"};
    inputFile >> ammoName;

    // time step for the target movement
    float targetTimeStep{0.0f};
    inputFile >> targetTimeStep;

    // time for simulation step
    float simTimeStep{0.0f};
    inputFile >> simTimeStep;    

    // hit radius of the target
    float hitRadius{0.0f};
    inputFile >> hitRadius;

    // the drone angular speed for the maneuver
    float angularSpeed{0.0f};
    inputFile >> angularSpeed;

    // the threshold for the turn decision
    float turnThreshold{0.0f};
    inputFile >> turnThreshold;
    
    // Close the input file
    inputFile.close();

    // store the targets coordinates in arrays
    float targetXInTime[TARGETS_COUNT][TARGETS_POINTS];
    float targetYInTime[TARGETS_COUNT][TARGETS_POINTS];

    // Output data
    float positions[MAX_STEPS][2]; // Store the drone's position at each step
    float directions[MAX_STEPS]; // Store the drone's direction at each step
    float speeds[MAX_STEPS]; // Store the drone's speed at each step
    droneStates states[MAX_STEPS]; // Store the drone's state at each step
    int currentTargets[MAX_STEPS]; // Store the current target index at each step

    
    ifstream targetFile("targets.txt");
    if (!targetFile) {
        cerr << "Error opening target file!" << endl;
        return 1;
    }

    for (int i = 0; i < TARGETS_COUNT; i++) {
        for (int j = 0; j < TARGETS_POINTS; j++) {
            targetFile >> targetXInTime[i][j];
        }
    }
    for (int i = 0; i < TARGETS_COUNT; i++) {
        for (int j = 0; j < TARGETS_POINTS; j++) {
            targetFile >> targetYInTime[i][j];
        }
    }
    targetFile.close();

    // Set the ammunition parameters in arrays for better organization and scalability
    char  bombName[][MAX_AMMO_NAME] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
    float bombMass[] = {0.35f,  0.6f,   1.2f,   0.45f,  1.4f};
    float bombDrag[] = {0.07f,  0.10f,  0.10f,  0.10f,  0.10f};
    float bombLift[] = {0.0f,   0.0f,   0.0f,   1.0f,   1.0f};

    // Find the index of the ammunition parameters based on the name
    int ammoIndex = -1;
    for (int i = 0; i < sizeof(bombName) / sizeof(bombName[0]); i++) {
        if (strncmp(ammoName, bombName[i], MAX_AMMO_NAME) == 0) {
            ammoIndex = i;
            break;
        }
    }

    if (ammoIndex == -1) {
        cerr << "Invalid ammunition name!" << endl;
        return 1;
    }

    float m = bombMass[ammoIndex];
    float d = bombDrag[ammoIndex];
    float l = bombLift[ammoIndex];

    droneStates state = EN_DS_STOPPED; // Initial state of the drone
    
    // Drone acceleration and maneuver parameters
    float currentSpeed = 0.0f;
    float acceleration = pow(attackSpeed, 2) / (2 * attackSpeed); // Acceleration rate of the drone

    float simTime = 0.0f;     // Simulation time variable
    int iteration = 0;        // Iteration counter for the simulation loop
    int currentTarget = -1;   // Index of the current target

    do {
        float distanceToTarget[TARGETS_COUNT];  // Array to store the distance from the drone to each target
        float directionToTarget[TARGETS_COUNT]; // Array to store the direction from the drone to each target
        float estimatedTimeToTarget[TARGETS_COUNT]; // Array to store the estimated time to reach each target

        int bestTarget = -1; // Variable to store the index of the best target to attack
        float bestTargetTime = numeric_limits<float>::max(); // Variable to store the best estimated time to target

#ifdef DEBUG        
        cout << std::fixed << std::setprecision(2);

        cout << "######### Simulation Time: " << simTime << " seconds" << endl;
        cout << "Drone Position: (" << xd << ", " << yd << ", " << zd << ")" << " Direction: " << direction << " radians";
        cout << " Speed: " << currentSpeed << " m/s State: " << state << endl;
#endif

        // loop through each target to calculate the distance and direction from the drone to the target
        for (int target = 0; target < TARGETS_COUNT; target++) { 
            float targetX{0.0f}, targetY{0.0f};
            float fireX{0.0f}, fireY{0.0f}; // store the calculated fire coordinates for each target

            targetInterpolation(target, targetXInTime, targetYInTime, targetTimeStep, simTime, targetX, targetY);
            if (calcBallistic(xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath, m, d, l, fireX, fireY) > 0) {
                // If the ballistics calculation fails, set the distance and direction to target to zero
                distanceToTarget[target] = 0.0f;
                directionToTarget[target] = 0.0f;
                continue;
            };

            distanceToTarget[target] = distance(fireX, fireY, xd, yd);
            directionToTarget[target] = atan2(fireY - yd, fireX - xd);

            if (distanceToTarget[target] < hitRadius) {
                estimatedTimeToTarget[target] = 0.0f; // Already within the hit radius, no time needed
            } else {
                estimatedTimeToTarget[target] = estimateTimeToTarget(distanceToTarget[target], acceleration, currentSpeed, attackSpeed, 
                    directionToTarget[target] - direction, turnThreshold, angularSpeed, hitRadius);
            }

            // Decision making based on the current state and the distance/direction to the target
            if (estimatedTimeToTarget[target] < bestTargetTime) {
                bestTargetTime = estimatedTimeToTarget[target];
                bestTarget = target;
            }

#ifdef DEBUG           
            cout << "Target " << target + 1 << ": (" << targetX << ", " << targetY << ")";
            cout << ",\tFire: (" << fireX << ", " << fireY << ")";
            cout << ",\tDistance: " << distanceToTarget[target] << ",\tDirection: " << directionToTarget[target] << " rad";
            cout << ",\tEst. Time: " << estimatedTimeToTarget[target] << " sec" << endl;
#endif
        }

        if (bestTarget != currentTarget) {
            currentTarget = bestTarget; // Update the current target to the best target based on the estimated time to target
#ifdef DEBUG
            cout << "*** Best Target: " << currentTarget + 1 << " with estimated time: " << bestTargetTime << " seconds" << endl;
#endif
        }
        simulationStep(state, directionToTarget[currentTarget], distanceToTarget[currentTarget], 
            turnThreshold, hitRadius, angularSpeed, acceleration, attackSpeed, simTimeStep, 
            direction, currentSpeed, xd, yd);
        simTime += simTimeStep;

        // Store the drone's position, direction, state, and current target index at each step for output
        positions[iteration][0] = xd;
        positions[iteration][1] = yd;
        directions[iteration] = direction;
        states[iteration] = state;
        currentTargets[iteration] = currentTarget;
        speeds[iteration] = currentSpeed;

    // Continue the simulation loop until the maximum number of steps is reached or a valid target is found
    } while ((++iteration < MAX_STEPS) && (currentTarget != -1) && (state != EN_DS_FIRING)); 

    ofstream outputFile("output.txt");
    if (!outputFile) {
        cerr << "Error opening output file!" << endl;
        return 1;
    }

    // Write the drone's position, direction, state, and current target index at each step to the output file
    outputFile << std::fixed << std::setprecision(2);
    outputFile << iteration << endl;
    for (int i = 0; i < iteration; i++) outputFile << positions[i][0] << " " << positions[i][1] << " "; outputFile << endl;
    for (int i = 0; i < iteration; i++) outputFile << directions[i] << " "; outputFile << endl;
    for (int i = 0; i < iteration; i++) outputFile << states[i] << " "; outputFile << endl;
    for (int i = 0; i < iteration; i++) outputFile << currentTargets[i] << " "; outputFile << endl;
    // for (int i = 0; i < iteration; i++) outputFile << speeds[i] << " "; outputFile << endl;
    outputFile.close();
}

int targetInterpolation(const int target, 
    const float targetXInTime[TARGETS_COUNT][TARGETS_POINTS], 
    const float targetYInTime[TARGETS_COUNT][TARGETS_POINTS], 
    const float targetTimeStep, const float simTime,
    float &targetX, float &targetY) {
    // Implement interpolation logic to get the target coordinates at the given simulation time
    // For example, you can use linear interpolation between the two closest time steps
    int idx = static_cast<int>(floor(simTime / targetTimeStep)) % TARGETS_POINTS;
    int next = (idx + 1) % TARGETS_POINTS;

    float frac = (simTime - idx * targetTimeStep) / targetTimeStep;

    targetX = targetXInTime[target][idx] + frac * 
        (targetXInTime[target][next] - targetXInTime[target][idx]);
    targetY = targetYInTime[target][idx] + frac * 
        (targetYInTime[target][next] - targetYInTime[target][idx]);

    return 0;
}

int calcBallistic(float xd, float yd, float zd, float targetX, float targetY, 
    float attackSpeed, float accelerationPath, float m, float d, float l,
    float &fireX, float &fireY) {

    const float g = 9.81f; // Acceleration due to gravity

    // Calculate coefficients for solve the cubic equation of motion
    float a = d * g * m - 2.0 * pow(d, 2) * l * attackSpeed;    
    float b = -3.0 * g * pow(m, 2) + 3.0 * d * l * m * attackSpeed;
    float c = 6.0 * pow(m, 2) * zd;

    // Check if the coefficient a is zero to avoid division by zero
    // compare float with zero using a small threshold to account for floating-point precision issues
    if (fabs(a) < 1e-6) {
        cerr << "Coefficient a is too close to zero! May be a speed or ammunition params issue." << endl;
        return 1;
    }

    float p = - pow(b, 2) / (3.0 * pow(a, 2));
    float q = 2.0 * pow(b, 3) / (27.0 * pow(a, 3)) + c / a;

    float arg = 3.0 * q / ( 2.0 * p ) * sqrt( -3.0 / p );
    if (arg < -1.0 || arg > 1.0) {
        cerr << "The argument for arccos is out of range! May be a height issue." << endl;
        return 1;
    }
    float phi = acos(arg);

    float t = 2 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);

    // Horisontal distance to the target
    float h = 
        attackSpeed * t - 
        pow(t, 2) * d * attackSpeed / (2.0 * m) + 
        pow(t, 3) * (
            6.0 * d * g * l * m - 6.0 * d * d * (l * l - 1.0) * attackSpeed
        ) / (36.0 * m * m) + 
        pow(t, 4) * (
            -6.0 * pow(d, 2)* g * l * (1 + pow(l, 2) + pow(l, 4)) * m + 
            3.0 * pow(d, 3) * pow(l, 2) * (1.0 + pow(l, 2)) * attackSpeed + 
            6.0 * pow(d, 3) * pow(l, 4) * (1.0 + pow(l, 2)) * attackSpeed
        ) / (36.0 * pow(1 + pow(l, 2), 2) * pow(m, 3)) +
        pow(t, 5) * (
            3.0 * pow(d, 3) * g * pow(l, 3) * m -
            3.0 * pow(d, 4) * pow(l, 2) * (1.0 + pow(l, 2)) * attackSpeed
        ) / (36.0 * (1 + pow(l, 2)) * pow(m, 4));

    // Determine the drop point
    float distanceTarget = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));

    // Check the drop conditions
    if (t <= 0.0 || h <= 0.0 || distanceTarget <= 0.0) {
        cerr << "The drone cannot hit the target! May be a speed or height issue." << endl;
        return 1;
    }
   
    // Calculate the coordinates of the drop point
    float ratio = (distanceTarget - h) / distanceTarget;
    fireX = xd + (targetX - xd) * ratio;
    fireY = yd + (targetY - yd) * ratio; 

    return 0;
}

int simulationStep(droneStates &state, float directionToTarget, float distanceToTarget, 
    float turnThreshold, float hitRadius, float angularSpeed, 
    float acceleration, float attackSpeed, float simTimeStep, 
    float &direction, float &currentSpeed, float &xd, float &yd) {

    droneStates newState = state; // Variable to hold the new state after decision making
    float delta = directionToTarget - direction;

    switch (state) {
        case EN_DS_STOPPED:
            if (currentSpeed > 0.0f) {
                newState = EN_DS_DECELERATING;
                break;
            } else if (fabs(delta) > turnThreshold) {
                newState = EN_DS_TURNING;
            } else {
                direction = directionToTarget; // Align the drone's direction with the target
                newState = EN_DS_ACCELERATING;
            } 
            break;
        case EN_DS_TURNING:
            if (fabs(delta) <= turnThreshold) {
                direction = directionToTarget; // Align the drone's direction with the target
                newState = EN_DS_ACCELERATING;
            } else if (currentSpeed > 0.0f) {
                newState = EN_DS_DECELERATING;
            } else {
                // Simulate the turning by adjusting the drone's direction
                direction += (delta > 0 ? 1 : -1) * angularSpeed * simTimeStep;
            }
            break;
        case EN_DS_ACCELERATING:
            if (fabs(delta) > turnThreshold && distanceToTarget > hitRadius) {
                newState = EN_DS_DECELERATING;
            } else {
                // Simulate the acceleration by increasing the drone's speed
                currentSpeed += acceleration * simTimeStep;
                if (currentSpeed >= attackSpeed) {
                    currentSpeed = attackSpeed; // Cap the speed at the attack speed
                    newState = EN_DS_MOVING; // Transition to moving state once the attack speed is reached
                }
                // Update the drone's position based on the current speed and direction
                xd += currentSpeed * cos(direction) * simTimeStep;
                yd += currentSpeed * sin(direction) * simTimeStep;
            }
            break;
        case EN_DS_DECELERATING:
            {
                // Simulate the deceleration by decreasing the drone's speed
                currentSpeed -= acceleration * simTimeStep;
                if (currentSpeed < 0)
                {
                    currentSpeed = 0.0f;      // Ensure the speed does not go negative
                    newState = EN_DS_STOPPED; // Transition to stopped state if speed reaches zero
                }
                // Update the drone's position based on the current speed and direction
                xd += currentSpeed * cos(direction) * simTimeStep;
                yd += currentSpeed * sin(direction) * simTimeStep;
            }
            break;
        case EN_DS_MOVING:
            
            if (distanceToTarget < hitRadius) {
                newState = EN_DS_FIRING;
            } else if (fabs(delta) > turnThreshold) {
                newState = EN_DS_TURNING;
            } else {
                // Update the drone's position based on the current speed and direction
                xd += currentSpeed * cos(direction) * simTimeStep;
                yd += currentSpeed * sin(direction) * simTimeStep;
            }
            break;
        case EN_DS_FIRING:
            // Simulate the firing action (e.g., print a message or update a counter)
            cout << "Firing at target!" << endl;
        default:
            break;
        }

        state = newState; // Update the state after decision making
    return 0;
}

float estimateTimeToTarget(float distance, float acceleration, float speed, float speedAttack, 
    float delta, float threshold, float angularSpeed, float hitRadius) {
    if (distance < hitRadius) {
        return 0.0f; // Already within the hit radius, no time needed
    }
    // Calculate the time to reach the target based on distance and speed
    if (fabs(delta) > threshold) {  
        // If the direction to the target is outside the threshold, we need to turn first
        float decelTime = acceleratingTime(speed, -acceleration, 0.0f); // Time to decelerate to zero
        float decelPath = accelMotionPath(speed, -acceleration, decelTime); // Distance covered during deceleration
        float turnTime = turningTime(delta, angularSpeed); // Time to turn
        float accelTime = acceleratingTime(0.0f, acceleration, speedAttack); // Time to accelerate back to speed
        float accelPath = accelMotionPath(0.0f, acceleration, accelTime); // Distance covered during acceleration
        float remainingDistance = distance + decelPath - accelPath; // Remaining distance after deceleration and acceleration
        if (remainingDistance + hitRadius < 0.0f) {
            return numeric_limits<float>::max(); // target unreachable within the given parameters
        }
        return decelTime + turnTime + accelTime + remainingDistance / speedAttack; // Total time to target
    } else {
        if (speed < speedAttack) {
            float accelTime = acceleratingTime(speed, acceleration, speedAttack); // Time to accelerate to attack speed
            float accelPath = accelMotionPath(speed, acceleration, accelTime); // Distance covered during acceleration
            if (distance + hitRadius - accelPath < 0.0f) {
                return numeric_limits<float>::max(); // target unreachable within the given parameters
            }
            float remainingDistance = distance - accelPath; // Remaining distance after acceleration
            return accelTime + remainingDistance / speedAttack; // Total time to target
        } 
        return distance / speed; // Time to target at current speed
    }
}

float accelMotionPath(float v0, float a, float t) {
    // Calculate the distance covered under constant acceleration
    return v0 * t + 0.5f * a * pow(t, 2);
}

float turningTime(float delta, float angularSpeed) {
    // Calculate the time required to turn by a certain angle at a given angular speed
    return fabs(delta) / angularSpeed;
}

float movingTime(float distance, float speed) {
    // Calculate the time required to move a certain distance at a given speed
    return distance / speed;
}

float acceleratingTime(float v0, float a, float targetSpeed) {
    // Calculate the time required to accelerate from v0 to targetSpeed at a given acceleration
    return (targetSpeed - v0) / a;
}

float distance(float x1, float y1, float x2, float y2) {
    // Calculate the distance between two points
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}