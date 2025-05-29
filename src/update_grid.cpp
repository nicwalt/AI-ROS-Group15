#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/ros.h>
#include <streambuf>
#include <fstream>
#include <map>
#include <cmath>
#include <mutex>
#include "geometry_msgs/Point.h"
#include "group_15/UpdateGrid.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SetModelState.h"
#include "group_15/Sensor.h"
#include "communal_defines.cpp"

#define GRID_WIDTH 1.0

ros::ServiceClient bot_location;
int currentGrid[BOARD_H][BOARD_W];
std::mutex grid_mutex;

geometry_msgs::Point coordinates[BOARD_H][BOARD_W];

ros::ServiceClient spawnClient;
ros::ServiceClient deleteClient;
ros::ServiceClient setClient;

int numSurvivors = 0;
int numHostiles = 0;
bool robot_saver1Spawned = false;
bool robot_saver2Spawned = false;
std::string homeDir = getenv("HOME");
std::string modelDir = homeDir + "/catkin_workspace/src/AI-ROS-Group15/models/";

// Position tracking for submarines
geometry_msgs::Point saver1_position;
geometry_msgs::Point saver2_position;

struct ComparePoints
{
    bool operator()(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) const
    {
        if (p1.x != p2.x)
            return p1.x < p2.x;
        if (p1.y != p2.y)
            return p1.y < p2.y;
        return p1.z < p2.z;
    }
};

std::map<geometry_msgs::Point, std::string, ComparePoints> objectPositions;

gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position);
bool updateGrid(group_15::UpdateGrid::Request &req, group_15::UpdateGrid::Response &res);
bool hostileSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res, const std::string& modelName);
bool survivorSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res, const std::string& modelName);

// Wrapper functions
bool hostileSensor1(group_15::Sensor::Request &req, group_15::Sensor::Response &res) {
    return hostileSensor(req, res, "robot_saver1");
}
bool hostileSensor2(group_15::Sensor::Request &req, group_15::Sensor::Response &res) {
    return hostileSensor(req, res, "robot_saver2");
}
bool survivorSensor1(group_15::Sensor::Request &req, group_15::Sensor::Response &res) {
    return survivorSensor(req, res, "robot_saver1");
}
bool survivorSensor2(group_15::Sensor::Request &req, group_15::Sensor::Response &res) {
    return survivorSensor(req, res, "robot_saver2");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_object_manager");
    ros::NodeHandle n;
    bot_location = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // Initialize submarine positions
    saver1_position.x = -1;
    saver1_position.y = -1;
    saver1_position.z = 0;
    saver2_position.x = -1;
    saver2_position.y = -1;
    saver2_position.z = 0;

    for (int i = 0; i < BOARD_H; ++i)
        for (int j = 0; j < BOARD_W; ++j)
            currentGrid[i][j] = EMPTY;

    for (int i = 0; i < BOARD_H; ++i)
        for (int j = 0; j < BOARD_W; ++j)
        {
            coordinates[i][j].x = i * GRID_WIDTH;
            coordinates[i][j].y = j * GRID_WIDTH;
            coordinates[i][j].z = 0;
        }

    setClient = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
    spawnClient = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    deleteClient = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    
    // Advertise services
    ros::ServiceServer HostileSenService1 = n.advertiseService("/sub1/hostile_sensor", hostileSensor1);
    ros::ServiceServer SurvivorSenService1 = n.advertiseService("/sub1/survivor_sensor", survivorSensor1);
    ros::ServiceServer HostileSenService2 = n.advertiseService("/sub2/hostile_sensor", hostileSensor2);
    ros::ServiceServer SurvivorSenService2 = n.advertiseService("/sub2/survivor_sensor", survivorSensor2);
    
    ros::ServiceServer updateGridService = n.advertiseService("update_grid", updateGrid);
    ros::spin();
    return 0;
}

gazebo_msgs::SpawnModel createSpawnRequest(int modelType, geometry_msgs::Point position)
{
    static std::map<int, std::string> modelCache;

    std::string modelName, modelKey;
    std::string modelPath;

    if (modelType == SURVIVOR)
    {
        modelName = "bowl" + std::to_string(numSurvivors++);
        modelKey = "bowl";
        modelPath = modelDir + "bowl/model.sdf";
    }
    else if (modelType == HOSTILE)
    {
        modelName = "cardboard_box" + std::to_string(numHostiles++);
        modelKey = "cardboard_box";
        modelPath = modelDir + "cardboard_box/model.sdf";
    }
    else if (modelType == SUB)
    {
        modelName = "robot_saver1";
        modelKey = "robot_saver1";
        modelPath = modelDir + "turtlebot3_burger/model.sdf";
    }
    else if (modelType == SUB2)
    {
        modelName = "robot_saver2";
        modelKey = "robot_saver2";
        modelPath = modelDir + "turtlebot3_burger/model.sdf";
    }
    else
    {
        ROS_ERROR("Unknown model type %d", modelType);
        throw std::runtime_error("Unknown model type");
    }

    // Cache model XML
    if (modelCache.find(modelType) == modelCache.end())
    {
        std::ifstream file(modelPath);
        if (!file)
        {
            ROS_WARN("Could not open model file: %s", modelPath.c_str());
            throw std::runtime_error("Model file not found");
        }
        std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        modelCache[modelType] = xml;
    }

    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name = modelName;
    spawn.request.model_xml = modelCache[modelType];
    spawn.request.initial_pose.position = position;
    spawn.request.initial_pose.orientation.w = 1.0;

    return spawn;
}

bool updateGrid(group_15::UpdateGrid::Request &req, group_15::UpdateGrid::Response &res)
{
    std::lock_guard<std::mutex> lock(grid_mutex);
    std_msgs::Int32MultiArray read_grid = req.grid;
    gazebo_msgs::SpawnModel spawn;
    gazebo_msgs::SetModelState set;
    gazebo_msgs::DeleteModel del;

    // Find submarine positions first
    geometry_msgs::Point saver1_new_pos, saver2_new_pos;
    bool saver1_found = false;
    bool saver2_found = false;
    
    for (int i = 0; i < BOARD_H; ++i) {
        for (int j = 0; j < BOARD_W; ++j) {
            int index = read_grid.data[i * BOARD_W + j];
            if (index == SUB) {
                saver1_new_pos = coordinates[i][j];
                saver1_found = true;
            } else if (index == SUB2) {
                saver2_new_pos = coordinates[i][j];
                saver2_found = true;
            }
        }
    }

    // Process submarines first
    if (saver1_found) {
        if (robot_saver1Spawned) {
            // Only move if position changed
            if (saver1_position.x != saver1_new_pos.x || saver1_position.y != saver1_new_pos.y) {
                set.request.model_state.model_name = "robot_saver1";
                set.request.model_state.pose.position = saver1_new_pos;
                set.request.model_state.pose.orientation.w = 1.0;
                
                if (setClient.call(set)) {
                    saver1_position = saver1_new_pos;
                    ROS_INFO("Moved robot_saver1 to (%.1f, %.1f)", saver1_new_pos.x, saver1_new_pos.y);
                } else {
                    ROS_WARN("Failed to move robot_saver1");
                }
            }
        } else {
            spawn = createSpawnRequest(SUB, saver1_new_pos);
            if (spawnClient.call(spawn)) {
                robot_saver1Spawned = true;
                saver1_position = saver1_new_pos;
                ROS_INFO("Spawned robot_saver1 at (%.1f, %.1f)", saver1_new_pos.x, saver1_new_pos.y);
            } else {
                ROS_WARN("Failed to spawn robot_saver1");
            }
        }
    }
    
    if (saver2_found) {
        if (robot_saver2Spawned) {
            // Only move if position changed
            if (saver2_position.x != saver2_new_pos.x || saver2_position.y != saver2_new_pos.y) {
                set.request.model_state.model_name = "robot_saver2";
                set.request.model_state.pose.position = saver2_new_pos;
                set.request.model_state.pose.orientation.w = 1.0;
                
                if (setClient.call(set)) {
                    saver2_position = saver2_new_pos;
                    ROS_INFO("Moved robot_saver2 to (%.1f, %.1f)", saver2_new_pos.x, saver2_new_pos.y);
                } else {
                    ROS_WARN("Failed to move robot_saver2");
                }
            }
        } else {
            spawn = createSpawnRequest(SUB2, saver2_new_pos);
            if (spawnClient.call(spawn)) {
                robot_saver2Spawned = true;
                saver2_position = saver2_new_pos;
                ROS_INFO("Spawned robot_saver2 at (%.1f, %.1f)", saver2_new_pos.x, saver2_new_pos.y);
            } else {
                ROS_WARN("Failed to spawn robot_saver2");
            }
        }
    }

    // Process grid updates
    for (int i = 0; i < BOARD_H; ++i) {
        for (int j = 0; j < BOARD_W; ++j) {
            int oldIndex = currentGrid[i][j];
            int newIndex = read_grid.data[i * BOARD_W + j];
            
            // Skip submarine cells since we already processed them
            if (newIndex == SUB || newIndex == SUB2) {
                currentGrid[i][j] = newIndex;
                continue;
            }
            
            if (oldIndex != newIndex) {
                geometry_msgs::Point point = coordinates[i][j];

                // Handle survivor spawn
                if (oldIndex == EMPTY && newIndex == SURVIVOR) {
                    spawn = createSpawnRequest(SURVIVOR, point);
                    objectPositions[point] = spawn.request.model_name;
                    if (!spawnClient.call(spawn)) {
                        ROS_WARN("Failed to spawn survivor at (%.1f, %.1f)", point.x, point.y);
                    }
                }
                
                // Handle hostile spawn
                else if (oldIndex == EMPTY && newIndex == HOSTILE) {
                    spawn = createSpawnRequest(HOSTILE, point);
                    objectPositions[point] = spawn.request.model_name;
                    if (!spawnClient.call(spawn)) {
                        ROS_WARN("Failed to spawn hostile at (%.1f, %.1f)", point.x, point.y);
                    }
                }
                
                // Handle survivor pickup
                else if (oldIndex == SURVIVOR && (newIndex == EMPTY || newIndex == SUB || newIndex == SUB2)) {
                    if (objectPositions.find(point) != objectPositions.end()) {
                        del.request.model_name = objectPositions[point];
                        if (!deleteClient.call(del)) {
                            ROS_WARN("Failed to delete survivor at (%.1f, %.1f)", point.x, point.y);
                        }
                        objectPositions.erase(point);
                    }
                }
                
                // Handle hostile removal
                else if (oldIndex == HOSTILE && (newIndex == EMPTY || newIndex == SUB || newIndex == SUB2)) {
                    if (objectPositions.find(point) != objectPositions.end()) {
                        del.request.model_name = objectPositions[point];
                        if (!deleteClient.call(del)) {
                            ROS_WARN("Failed to delete hostile at (%.1f, %.1f)", point.x, point.y);
                        }
                        objectPositions.erase(point);
                    }
                }
                
                currentGrid[i][j] = newIndex;
            }
        }
    }

    res.altered_grid = req.grid;
    return true;
}

bool hostileSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res, const std::string& modelName)
{
    std::lock_guard<std::mutex> lock(grid_mutex);
    res.objectNorth = false;
    res.objectSouth = false;
    res.objectWest = false;
    res.objectEast = false;
    res.objectDetected = false;

    int x, y;
    if (modelName == "robot_saver1") {
        x = std::round(saver1_position.x / GRID_WIDTH);
        y = std::round(saver1_position.y / GRID_WIDTH);
    } else {
        x = std::round(saver2_position.x / GRID_WIDTH);
        y = std::round(saver2_position.y / GRID_WIDTH);
    }

    // Validate position
    if (x < 0 || x >= BOARD_H || y < 0 || y >= BOARD_W) {
        ROS_WARN("Invalid position for %s: (%d, %d)", modelName.c_str(), x, y);
        return true;
    }

    int range = req.sensorRange;

    res.northRadar = std::vector<int32_t>(range, 0);
    res.southRadar = std::vector<int32_t>(range, 0);
    res.eastRadar = std::vector<int32_t>(range, 0);
    res.westRadar = std::vector<int32_t>(range, 0);

    for (int i = 1; i <= range; ++i)
    {
        // North (decreasing x)
        if (x - i >= 0 && currentGrid[x - i][y] == HOSTILE) {
            res.objectNorth = true;
            res.northRadar[i - 1] = 1;
        }
        // South (increasing x)
        if (x + i < BOARD_H && currentGrid[x + i][y] == HOSTILE) {
            res.objectSouth = true;
            res.southRadar[i - 1] = 1;
        }
        // West (decreasing y)
        if (y - i >= 0 && currentGrid[x][y - i] == HOSTILE) {
            res.objectWest = true;
            res.westRadar[i - 1] = 1;
        }
        // East (increasing y)
        if (y + i < BOARD_W && currentGrid[x][y + i] == HOSTILE) {
            res.objectEast = true;
            res.eastRadar[i - 1] = 1;
        }
    }

    if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
        res.objectDetected = true;
    return true;
}

bool survivorSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res, const std::string& modelName)
{
    std::lock_guard<std::mutex> lock(grid_mutex);
    res.objectNorth = false;
    res.objectSouth = false;
    res.objectWest = false;
    res.objectEast = false;
    res.objectDetected = false;

    int x, y;
    if (modelName == "robot_saver1") {
        x = std::round(saver1_position.x / GRID_WIDTH);
        y = std::round(saver1_position.y / GRID_WIDTH);
    } else {
        x = std::round(saver2_position.x / GRID_WIDTH);
        y = std::round(saver2_position.y / GRID_WIDTH);
    }

    // Validate position
    if (x < 0 || x >= BOARD_H || y < 0 || y >= BOARD_W) {
        ROS_WARN("Invalid position for %s: (%d, %d)", modelName.c_str(), x, y);
        return true;
    }

    int range = req.sensorRange;

    res.northRadar = std::vector<int32_t>(range, 0);
    res.southRadar = std::vector<int32_t>(range, 0);
    res.eastRadar = std::vector<int32_t>(range, 0);
    res.westRadar = std::vector<int32_t>(range, 0);

    for (int i = 1; i <= range; ++i)
    {
        // North (decreasing x)
        if (x - i >= 0 && currentGrid[x - i][y] == SURVIVOR) {
            res.objectNorth = true;
            res.northRadar[i - 1] = 1;
        }
        // South (increasing x)
        if (x + i < BOARD_H && currentGrid[x + i][y] == SURVIVOR) {
            res.objectSouth = true;
            res.southRadar[i - 1] = 1;
        }
        // West (decreasing y)
        if (y - i >= 0 && currentGrid[x][y - i] == SURVIVOR) {
            res.objectWest = true;
            res.westRadar[i - 1] = 1;
        }
        // East (increasing y)
        if (y + i < BOARD_W && currentGrid[x][y + i] == SURVIVOR) {
            res.objectEast = true;
            res.eastRadar[i - 1] = 1;
        }
    }

    if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
        res.objectDetected = true;
    return true;
}