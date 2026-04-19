#include <iostream>
#include <fstream>
#include <cmath>

#define MAX_AMMUNITION_NAME_LENGTH 16

using namespace std;

int main() {
    // Open the input file
    ifstream inputFile("input.txt");
    if (!inputFile) {
        cerr << "Error opening file!" << endl;
        return 1;
    }

    // Initialize variables for prevent the side-effects
    // Read the coordinates of the drone
    float xd{0.0}, yd{0.0}, zd{0.0};
    inputFile >> xd >> yd >> zd;

    // Read the coordinates of the target
    float targetX{0.0}, targetY{0.0}   ;
    inputFile >> targetX >> targetY;

    // Read the attack speed of the drone
    float attackSpeed{0.0};
    inputFile >> attackSpeed;

    // Read the acceleration path of the drone
    float accelerationPath{0.0};
    inputFile >> accelerationPath;

    // Read the ammunition name
    string ammunitionName{"UNDEFINED"};
    inputFile >> ammunitionName;

    // Close the input file
    inputFile.close();

    // Choose the appropriate ammunition based on the name
    float m, d, l; // mass, drag coefficient, lift
    // Compare the ammunition name with the predefined names and set the parameters accordingly
    // Use strncmp to compare the strings safely, ensuring we do not exceed the maximum length 
    if (strncmp(ammunitionName.c_str(), "VOG-17", 
        MAX_AMMUNITION_NAME_LENGTH) == 0) {
        m = 0.35; d = 0.07; l = 0.0;
    } else if (strncmp(ammunitionName.c_str(), "M67", 
        MAX_AMMUNITION_NAME_LENGTH) == 0) {
        m = 0.6; d = 0.10; l = 0.0; 
    } else if (strncmp(ammunitionName.c_str(), "RKG-3", 
        MAX_AMMUNITION_NAME_LENGTH) == 0) {
        m = 1.2; d = 0.10; l = 0.0; 
    } else if (strncmp(ammunitionName.c_str(), "GLIDING-VOG", 
        MAX_AMMUNITION_NAME_LENGTH) == 0) {
        m = 0.45; d = 0.10; l = 1.0; 
    } else if (strncmp(ammunitionName.c_str(), "GLIDING-RKG", 
        MAX_AMMUNITION_NAME_LENGTH) == 0) {
        m = 1.4; d = 0.10; l = 1.0; 
    } else {
        cerr << "Invalid ammunition name!" << endl;
        return 1;
    }   

    const float g = 9.81; // Acceleration due to gravity

    // Calculate coefficients for solve the cubic equation of motion
    float a = d * g * m - 2.0 * d * d * l * attackSpeed;    
    float b = -3.0 * g * m * m + 3.0 * d * l * m * attackSpeed;
    float c = 6.0 * m * m * zd;

    if (a == 0.0) {
        cerr << "May be a speed or ammunition params issue." << endl;
        return 1;
    }

    float p = -b * b / (3.0 * a * a);
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

    // Open the output file
    ofstream outFile("output.txt");
    if (!outFile) {
        cerr << "Error opening output file!" << endl;
        return 1;
    }

    // Check if the drop point is within the target area
    if (h + accelerationPath > distanceTarget) {
        cerr << "Should to perform a maneuver!" << endl;
        float xd1 = targetX - (targetX - xd) * (h + accelerationPath) / distanceTarget;
        float yd1 = targetY - (targetY - yd) * (h + accelerationPath) / distanceTarget;
        outFile << xd1 << " " << yd1 << " ";
    } 
    
    // Calculate the coordinates of the drop point
    float ratio = (distanceTarget - h) / distanceTarget;
    float fireX = xd + (targetX - xd) * ratio;
    float fireY = yd + (targetY - yd) * ratio; 

    outFile << fireX << " " << fireY << endl;
    outFile.close();

    return 0;
}