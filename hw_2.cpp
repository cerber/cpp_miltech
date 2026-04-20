#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <cmath>

#define MAX_STEPS 1

#define MAX_AMMO_NAME 16
#define TARGETS_COUNT 5
#define TARGETS_POINTS 60

#define DEBUG 1

using namespace std;

int targetInterpolation(const int target, 
    const float targetXInTime[TARGETS_COUNT][TARGETS_POINTS], 
    const float targetYInTime[TARGETS_COUNT][TARGETS_POINTS], 
    const float targetTimeStep, const float simTime,
    float &targetX, float &targetY);
int ballistics(float xd, float yd, float zd, float targetX, float targetY, 
    float attackSpeed, float accelerationPath, float m, float d, float l,
    float &fireX, float &fireY, float &xd1, float &yd1);

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

    // for (int i = 0; i < TARGETS_COUNT; i++) {
    //     for (int j = 0; j < TARGETS_POINTS; j++) {
    //         // Simulate the target movement and store the coordinates in arrays
    //         // For example, you can use a simple circular motion for the target
    //         float angle = targetTimeStep * j + i * M_PI / 2; // Different phase for each target
    //         targetXInTime[i][j] = 100.0f * cos(angle); // Example radius of 100 units
    //         targetYInTime[i][j] = 100.0f * sin(angle);
    //     }
    // }   

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

   // The drone states
    enum droneStates {
        EN_DS_STOPPED,      // The drone is stationary
        EN_DS_ACCELERATING, // The drone is accelerating towards the target
        EN_DS_DECELERATING, // The drone is decelerating towards the target
        EN_DS_TURNING,      // The drone is turning
        EN_DS_MOVING        // The drone is moving towards the target
    };

    droneStates state = EN_DS_STOPPED; // Initial state of the drone
    
    // Drone acceleration and maneuver parameters
    float currentSpeed = 0.0f;
    float acceleration = pow(attackSpeed, 2) / (2 * attackSpeed); // Acceleration rate of the drone

    float simTime = 0.0f;   // Simulation time variable
    int iteration = 0;      // Iteration counter for the simulation loop
    int currentTarget = 0;  // Index of the current target

    do {
        float distanceToTarget[TARGETS_COUNT];
        float directionToTarget[TARGETS_COUNT];

#ifdef DEBUG        
        cout << std::fixed << std::setprecision(2);
        cout << "######### Simulation Time: " << simTime << " seconds" << endl;
        cout << "Drone Position: (" << xd << ", " << yd << ", " << zd << ")" << " Direction: " << direction << " radians";
        cout << " Speed: " << currentSpeed << " m/s State: " << state << endl;
#endif

        for (int target = 0; target < TARGETS_COUNT; target++) {
            float targetX{0.0f}, targetY{0.0f};
            float fireX{0.0f}, fireY{0.0f}, xd1{0.0f}, yd1{0.0f};

            targetInterpolation(target, targetXInTime, targetYInTime, targetTimeStep, simTime, targetX, targetY);
            ballistics(xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath, m, d, l, fireX, fireY, xd1, yd1);
            distanceToTarget[target] = sqrt(pow(fireX - xd, 2) + pow(fireY - yd, 2));
            directionToTarget[target] = atan2(fireY - yd, fireX - xd);

#ifdef DEBUG           
            cout << "Target " << target + 1 << ": (" << targetX << ", " << targetY << ")";
            cout << ",\tFire Point: (" << fireX << ", " << fireY << ")" << ",\tManeuver Point: (" << xd1 << ", " << yd1 << ")";
            cout << ",\tDistance to Target: " << distanceToTarget[target] << ",\tDirection to Target: " << directionToTarget[target] << " radians" << endl;
#endif
        }
        simTime += simTimeStep;

    } while (++iteration < MAX_STEPS);

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

int ballistics(float xd, float yd, float zd, float targetX, float targetY, 
    float attackSpeed, float accelerationPath, float m, float d, float l,
    float &fireX, float &fireY, 
    float &xd1, float &yd1) {

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

    // Check if the drop point is within the target area
    if (h + accelerationPath > distanceTarget) {
        cerr << "Should to perform a maneuver!" << endl;
        xd1 = targetX - (targetX - xd) * (h + accelerationPath) / distanceTarget;
        yd1 = targetY - (targetY - yd) * (h + accelerationPath) / distanceTarget;
    } 
    
    // Calculate the coordinates of the drop point
    float ratio = (distanceTarget - h) / distanceTarget;
    fireX = xd + (targetX - xd) * ratio;
    fireY = yd + (targetY - yd) * ratio; 

    return 0;
}