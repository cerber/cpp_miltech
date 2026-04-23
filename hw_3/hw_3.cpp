#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>

#define ENABLE_LOG 1
#define ENABLE_DEBUG 1

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

const int MAX_STEPS = 10000;
const int LENGTH_AMMO_NAME = 32;

enum States {
  EN_DS_STOPPED,       // The drone is stationary
  EN_DS_ACCELERATING,  // The drone is accelerating towards the target
  EN_DS_DECELERATING,  // The drone is decelerating towards the target
  EN_DS_TURNING,       // The drone is turning
  EN_DS_MOVING,        // The drone is moving towards the target
  EN_DS_FIRING         // The drone is firing at the target
};

struct Coord {
  float x;  // X coordinate
  float y;  // Y coordinate

  // Relod the addition operator to add two coordinates together
  Coord operator+(const Coord& other) const {
    return {x + other.x, y + other.y};
  }

  // Relod the subtraction operator to subtract one coordinate from another
  Coord operator-(const Coord& other) const {
    return {x - other.x, y - other.y};
  }

  // Relod the multiplication operator to scale a coordinate by a factor
  Coord operator*(float factor) const { return {x * factor, y * factor}; }

  // Relod the division operator to scale a coordinate by the inverse of a
  // factor
  Coord operator/(float factor) const { return {x / factor, y / factor}; }

  // Calculate the distance from this coordinate to another coordinate
  float distanceTo(const Coord& other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
  }

  // Calculate the angle from this coordinate to another coordinate in radians
  float angleTo(const Coord& other) const {
    return std::atan2(other.y - y, other.x - x);
  }

  // Normalize the coordinate to a unit vector
  Coord normalize() const {
    float length = std::sqrt(x * x + y * y);
    if (length > 0) {
      return {x / length, y / length};
    } else {
      return {0, 0};  // Return a zero vector if the length is zero to avoid
                      // division by zero
    }
  }

  // Rotate the coordinate by a given angle in radians
  Coord rotate(float angle) const {
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);
    return {x * cosAngle - y * sinAngle, x * sinAngle + y * cosAngle};
  }

  // Calculate the dot product of this coordinate with another coordinate
  float dot(const Coord& other) const { return x * other.x + y * other.y; }
};

std::ostream& operator<<(std::ostream& s, const Coord& c) {
  std::ios_base::fmtflags old_flags = s.flags();
  std::streamsize old_prec = s.precision();
  s << std::fixed << std::setprecision(6);
  s << "(" << c.x << ", " << c.y << ")";
  s.precision(old_prec);
  s.flags(old_flags);
  return s;
}

struct AmmoParams {
  char name[LENGTH_AMMO_NAME];
  float mass;  // Mass of the ammunition
  float drag;  // Drag coefficient of the ammunition
  float lift;  // Lift coefficient of the ammunition
};

struct DroneConfig {
  Coord startPos;     // Starting position of the drone (x, y)
  float altitude;     // Altitude of the drone (z)
  float initialDir;   // Initial direction of the drone (radians)
  float attackSpeed;  // Speed at which the drone should attack the target (m/s)
  float accelPath;    // Acceleration path of the drone (m)
  char ammoName[LENGTH_AMMO_NAME];  // Name of the ammunition used by the drone
  float arrayTimeStep;              // Time step for the target movement (s)
  float simTimeStep;                // Time step for the simulation (s)
  float hitRadius;                  // Hit radius of the target (m)
  float angularSpeed;   // Angular speed of the drone for maneuvering (rad/s)
  float turnThreshold;  // Threshold for the turn decision (rad)
};

struct SimStep {
  Coord pos;     // Position of the drone at the current step (x, y)
  States state;  // State of the drone at the current step (e.g., stopped,
                 // turning, accelerating, etc.)
  float direction;  // Direction of the drone at the current step (radians)
  float speed;      // Speed of the drone at the current step (m/s)
  int targetIdx;    // Index of the current target being engaged (if any)
  Coord dropPoint;  // Coordinates where the bomb would drop if released at the
                    // current step (x, y)
  Coord aimPoint;   // Coordinates where the drone should aim to hit the target
                    // (x, y)
  Coord predictedTarget;  // Predicted coordinates of the target at the time of
                          // impact (x, y)
};

struct Targets {
  int tgtCount;
  int timeSteps;
  Coord** pos;
};

using json = nlohmann::json;

/******************************************************************************/
/* Defining global variables                                                  */
/******************************************************************************/
DroneConfig config;
AmmoParams ammo;
Targets targets;

int loadDroneConfig(const char* filename, DroneConfig& config) {
  std::ifstream inputFile(filename);
  if (!inputFile) {
    std::cerr << "Error opening configuration file!" << std::endl;
    return 1;
  }

  json j = json::parse(inputFile);

  config.startPos.x = j["drone"]["position"]["x"];
  config.startPos.y = j["drone"]["position"]["y"];
  config.altitude = j["drone"]["altitude"];
  config.initialDir = j["drone"]["initialDirection"];
  config.attackSpeed = j["drone"]["attackSpeed"];
  config.accelPath = j["drone"]["accelerationPath"];
  config.angularSpeed = j["drone"]["angularSpeed"];
  config.turnThreshold = j["drone"]["turnThreshold"];
  strncpy(config.ammoName, j["ammo"].get<std::string>().c_str(),
          LENGTH_AMMO_NAME);
  config.arrayTimeStep = j["targetArrayTimeStep"];
  config.simTimeStep = j["simulation"]["timeStep"];
  config.hitRadius = j["simulation"]["hitRadius"];

  LOG("Drone configuration loaded successfully.");
//   DEBUG("Drone starting position: (" << config.startPos.x << ", "
//                                      << config.startPos.y << ")");
  DEBUG("Drone starting position: " << config.startPos);
  DEBUG("Drone altitude: " << config.altitude);
  DEBUG("Drone initial direction: " << config.initialDir << " radians");
  DEBUG("Drone attack speed: " << config.attackSpeed << " m/s");
  DEBUG("Drone acceleration path: " << config.accelPath << " m");
  DEBUG("Drone angular speed: " << config.angularSpeed << " rad/s");
  DEBUG("Drone turn threshold: " << config.turnThreshold << " radians");
  DEBUG("Ammunition name: " << config.ammoName);
  DEBUG("Target array time step: " << config.arrayTimeStep << " seconds");

  inputFile.close();
  return 0;
}

int loadAmmoParams(const char* filename, char* ammoName, AmmoParams& ammo) {
  std::ifstream inputFile(filename);
  if (!inputFile) {
    std::cerr << "Error opening ammunition parameters file!" << std::endl;
    return 1;
  }

  json j = json::parse(inputFile);

  int ammoCount = j.size();
  for (int i = 0; i < ammoCount; i++) {
    strncpy(ammo.name, j[i]["name"].get<std::string>().c_str(),
            LENGTH_AMMO_NAME);
    if (strncmp(ammo.name, ammoName, LENGTH_AMMO_NAME) == 0) {
      ammo.mass = j[i]["mass"];
      ammo.drag = j[i]["drag"];
      ammo.lift = j[i]["lift"];
      break;
    }
  }
  LOG("Ammunition parameters initialized.");
  DEBUG("Ammunition name: " << ammo.name);
  DEBUG("Ammunition mass: " << ammo.mass << " kg");
  DEBUG("Ammunition drag coefficient: " << ammo.drag);
  DEBUG("Ammunition lift coefficient: " << ammo.lift);

  inputFile.close();
  return 0;
}

int loadTargets(const char* filename, Targets &t) {
  std::ifstream inputFile(filename);
  if (!inputFile) {
    std::cerr << "Error opening targets parameters file!" << std::endl;
    return 1;
  }

  json j = json::parse(inputFile);
  t.tgtCount = j["targetCount"];
  t.timeSteps = j["timeSteps"];

  t.pos = new Coord*[t.tgtCount];
  for (int i = 0; i < t.tgtCount; i++) {
    t.pos[i] = new Coord[t.timeSteps];
    for (int k = 0; k < t.timeSteps; k++) {
      t.pos[i][k].x = j["targets"][i]["positions"][k]["x"];
      t.pos[i][k].y = j["targets"][i]["positions"][k]["y"];
    }
  }
  LOG("Targets coordinates loaded");
  return 0;
}

void deleteTargets(Targets &t) {
  for (int i = 0; i < t.tgtCount; i++) delete[] t.pos[i];
  delete[] t.pos;
}

int targetInterpolation(int iteration, Coord* tc) {
  float simTime = config.simTimeStep * iteration;
  int idx = static_cast<int>(floor(simTime / config.arrayTimeStep)) %
            targets.timeSteps;
  int next = (idx + 1) % targets.timeSteps;
  float frac = (simTime - idx * targets.timeSteps) / targets.timeSteps;

  for (int i = 0; i < targets.tgtCount; i++) {
    tc[i] = targets.pos[i][idx] + (targets.pos[i][next] - targets.pos[i][idx]) * frac;
  }
}

void simulationLoop(SimStep &sim) {
  Coord* tc = new Coord[targets.tgtCount];
  LOG("Starting simulation loop...");

  for (int iteration = 0; iteration < MAX_STEPS; iteration++) {
    targetInterpolation(iteration, tc);
  }
  delete[] tc;
}

int main() {
  if (loadDroneConfig("config.json", config) != 0) {
    return 1;  // Exit if configuration loading fails
  }
  if (loadAmmoParams("ammo.json", config.ammoName, ammo) != 0) {
    return 1;  // Exit if ammunition parameters loading fails
  }
  if (loadTargets("targets.json", targets) !=0) {
    return 1;
  }

  SimStep sim;

  simulationLoop(sim);

  deleteTargets(targets);
}