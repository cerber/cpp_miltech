#include <iostream>
#include <fstream>
#include <cmath>

#define MAX_AMMO_NAME 16
#define MAX_STEPS 10000

using namespace std;

int targetInterpolation(const int target, const float targetXInTime[5][60], const float targetYInTime[5][60], 
    const float targetTimeStep, const float simTime,
    float &targetX, float &targetY);
int ballistics(float xd, float yd, float zd, float targetX, float targetY, 
    float attackSpeed, float accelerationPath, float m, float d, float l);

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
    float initialDir{0.0f};
    inputFile >> initialDir;

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
    float targetXInTime[5][60], targetYInTime[5][60];
    
    ifstream targetFile("targets.txt");
    if (!targetFile) {
        cerr << "Error opening target file!" << endl;
        return 1;
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 60; j++) {
            targetFile >> targetXInTime[i][j];
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 60; j++) {
            targetFile >> targetYInTime[i][j];
        }
    }
    targetFile.close();

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 60; j++) {
            // Simulate the target movement and store the coordinates in arrays
            // For example, you can use a simple circular motion for the target
            float angle = targetTimeStep * j + i * M_PI / 2; // Different phase for each target
            targetXInTime[i][j] = 100.0f * cos(angle); // Example radius of 100 units
            targetYInTime[i][j] = 100.0f * sin(angle);
        }
    }   

    // Set the ammunition parameters in arrays for better organization and scalability
    char  bombName[][MAX_AMMO_NAME] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
    float bombMass[] = {0.35f,  0.6f,   1.2f,   0.45f,  1.4f};
    float bombDrag[] = {0.07f,  0.10f,  0.10f,  0.10f,  0.10f};
    float bombLift[] = {0.0f,   0.0f,   0.0f,   1.0f,   1.0f};

    // The drone states
    enum droneStates {
        EN_DS_STOPPED,      // The drone is stationary
        EN_DS_ACCELERATING, // The drone is accelerating towards the target
        EN_DS_DECELERATING, // The drone is decelerating towards the target
        EN_DS_TURNING,      // The drone is turning
        EN_DS_MOVING        // The drone is moving towards the target
    };

    droneStates state = EN_DS_STOPPED; // Initial state of the drone
    
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

    int iteration = 0;

    do {
        for (int target = 0; target < 5; target++) {
            float targetX, targetY;
            targetInterpolation(target, targetXInTime, targetYInTime, targetTimeStep, simTimeStep, targetX, targetY);
            ballistics(xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath, m, d, l);
        }
    } while (++iteration < MAX_STEPS);

}

int targetInterpolation(const int target, const float targetXInTime[5][60], const float targetYInTime[5][60], 
    const float targetTimeStep, const float simTime,
    float &targetX, float &targetY) {
    // Implement interpolation logic to get the target coordinates at the given simulation time
    // For example, you can use linear interpolation between the two closest time steps
    int idx = static_cast<int>floor(simTime / targetTimeStep) % 60;
    int next = (idx + 1) % 60;

    float frac = (simTime - idx * targetTimeStep) / targetTimeStep;

    float targetX = targetXInTime[target][idx] + frac * (targetXInTime[target][next] - targetXInTime[target][idx]);
    float targetY = targetYInTime[target][idx] + frac * (targetYInTime[target][next] - targetYInTime[target][idx]);

    return 0;
}

int ballistics(float xd, float yd, float zd, float targetX, float targetY, 
    float attackSpeed, float accelerationPath, float m, float d, float l) {

    const float g = 9.81f; // Acceleration due to gravity

    // Calculate coefficients for solve the cubic equation of motion
    float a = d * g * m - 2.0 * pow(d, 2) * l * attackSpeed;    
    float b = -3.0 * g * pow(m, 2) + 3.0 * d * l * m * attackSpeed;
    float c = 6.0 * pow(m, 2) * zd;

    if (a == 0.0) {
        cerr << "May be a speed or ammunition params issue." << endl;
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
            6.0 * pow(d, 3) * pow(l, 4) * (1 + pow(l, 2)) * attackSpeed
        ) / (36.0 * pow(1 + pow(l, 2), 2) * pow(m, 3)) +
        pow(t, 5) * (
            3.0 * pow(d, 3) * g * pow(l, 3) * m -
            3.0 * pow(d, 4) * pow(l, 2) * (1 + pow(l, 2)) * attackSpeed
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
        float xd1 = targetX - (targetX - xd) * (h + accelerationPath) / distanceTarget;
        float yd1 = targetY - (targetY - yd) * (h + accelerationPath) / distanceTarget;
    } 
    
    // Calculate the coordinates of the drop point
    float ratio = (distanceTarget - h) / distanceTarget;
    float fireX = xd + (targetX - xd) * ratio;
    float fireY = yd + (targetY - yd) * ratio; 

    return 0;
}