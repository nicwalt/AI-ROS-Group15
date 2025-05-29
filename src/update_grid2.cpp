#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/ros.h>
#include <streambuf>
#include <fstream>
#include <map>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "group_15/UpdateGrid2.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SetModelState.h"
#include "group_15/Sensor2.h"
#include "communal_defines.cpp"

#define GRID_WIDTH 1.0

ros::ServiceClient bot_location;
gazebo_msgs::GetModelState srv;
int currentGrid2[BOARD_H][BOARD_W];

geometry_msgs::Point coordinates[BOARD_H][BOARD_W];

ros::ServiceClient spawnClient2;
ros::ServiceClient deleteClient2;
ros::ServiceClient setClient2;

int numSurvivors = 0;
int numHostiles = 0;
bool robot_saver2Spawned = false;
std::string homeDir = getenv("HOME");
std::string modelDir = homeDir + "/catkin_workspace/src/AI-ROS-Group15/models/";

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
bool updateGrid2(group_15::UpdateGrid2::Request &req, group_15::UpdateGrid2::Response &res);
bool hostileSensor2(group_15::Sensor2::Request &req, group_15::Sensor2::Response &res);
bool survivorSensor2(group_15::Sensor2::Request &req, group_15::Sensor2::Response &res);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_object_manager2");  // Changed node name
	ros::NodeHandle n;
	bot_location = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	srv.request.model_name = "robot_saver2";  // Will be used after spawn

	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
			currentGrid2[i][j] = EMPTY;

	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
		{
			coordinates[i][j].x = j * GRID_WIDTH;  // FIXED: j->x, i->y
			coordinates[i][j].y = i * GRID_WIDTH;  // FIXED: Swapped coordinates
			coordinates[i][j].z = 0;
		}

	setClient2 = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
	spawnClient2 = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	deleteClient2 = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
	ros::ServiceServer HostileSenService = n.advertiseService("hostile_sensor2", hostileSensor2);
	ros::ServiceServer SurvivorSenService = n.advertiseService("survivor_sensor2", survivorSensor2);
	ros::ServiceServer updateGrid2Service = n.advertiseService("update_grid2", updateGrid2);
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
	spawn.request.initial_pose.orientation.w = 1.0; // valid identity quaternion

	return spawn;
}


bool updateGrid2(group_15::UpdateGrid2::Request &req, group_15::UpdateGrid2::Response &res)
{
	std_msgs::Int32MultiArray read_grid = req.grid;
	gazebo_msgs::SetModelState set;
	gazebo_msgs::DeleteModel del;
	gazebo_msgs::SpawnModel spawn;

	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
		{
			int oldIndex = currentGrid2[i][j];
			int newIndex = read_grid.data[i * BOARD_W + j];
			if (oldIndex != newIndex)
			{
				geometry_msgs::Point point = coordinates[i][j];

				if (oldIndex == EMPTY && newIndex == SURVIVOR)
				{
					spawn = createSpawnRequest(SURVIVOR, point);
					if (spawnClient2.call(spawn)) {
						objectPositions[point] = spawn.request.model_name;
					} else {
						ROS_ERROR("Failed to spawn survivor at (%f,%f)", point.x, point.y);
					}
				}
				if (oldIndex == SURVIVOR && newIndex == SUB2)
				{
					del.request.model_name = objectPositions[point];
					if (deleteClient2.call(del)) {
						objectPositions.erase(point);
					} else {
						ROS_ERROR("Failed to delete survivor at (%f,%f)", point.x, point.y);
					}
					
					if (robot_saver2Spawned) {
						set.request.model_state.model_name = "robot_saver2";
						set.request.model_state.pose.position = point;
						if (!setClient2.call(set)) {
							ROS_ERROR("Failed to move robot_saver2");
						}
					}
				}
				if (oldIndex == EMPTY && newIndex == HOSTILE)
				{
					spawn = createSpawnRequest(HOSTILE, point);
					if (spawnClient2.call(spawn)) {
						objectPositions[point] = spawn.request.model_name;
					} else {
						ROS_ERROR("Failed to spawn hostile at (%f,%f)", point.x, point.y);
					}
				}
				if ((oldIndex == EMPTY || oldIndex == VISITED) && newIndex == SUB2)
				{
					if (robot_saver2Spawned)
					{
						set.request.model_state.model_name = "robot_saver2";
						set.request.model_state.pose.position = point;
						if (!setClient2.call(set)) {
							ROS_ERROR("Failed to move robot_saver2");
						}
					}
					else
					{
						spawn = createSpawnRequest(SUB2, point);
						if (spawnClient2.call(spawn)) {
							objectPositions[point] = spawn.request.model_name;
							robot_saver2Spawned = true;
							ROS_INFO("Spawned robot_saver2 at (%f,%f)", point.x, point.y);
						} else {
							ROS_ERROR("Failed to spawn robot_saver2");
						}
					}
				}
				currentGrid2[i][j] = newIndex;
			}
		}

	res.altered_grid = req.grid;
	return true;
}

bool hostileSensor2(group_15::Sensor2::Request &req, group_15::Sensor2::Response &res)
{
	res.objectNorth = false;
	res.objectSouth = false;
	res.objectWest = false;
	res.objectEast = false;
	res.objectDetected = false;

	// Skip if robot not spawned yet
	if (!robot_saver2Spawned) {
		ROS_WARN("robot_saver2 not spawned, skipping sensor call");
		return true;
	}

	if (!bot_location.call(srv))
	{
		ROS_WARN("Failed to call ROS GetModelState service to get bot location");
		return false;
	}

	// FIXED: Convert world coordinates to grid indices
	int x = std::round(srv.response.pose.position.y / GRID_WIDTH); // y->row
	int y = std::round(srv.response.pose.position.x / GRID_WIDTH); // x->col
	int range = req.sensorRange;

	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid2[x - i][y] == HOSTILE)
		{ // North
			res.objectNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid2[x + i][y] == HOSTILE)
		{ // South
			res.objectSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid2[x][y - i] == HOSTILE)
		{ // West
			res.objectWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid2[x][y + i] == HOSTILE)
		{ // East
			res.objectEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}

	if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
		res.objectDetected = true;
	return true;
}

bool survivorSensor2(group_15::Sensor2::Request &req, group_15::Sensor2::Response &res)
{
	res.objectNorth = false;
	res.objectSouth = false;
	res.objectWest = false;
	res.objectEast = false;
	res.objectDetected = false;

	// Skip if robot not spawned yet
	if (!robot_saver2Spawned) {
		ROS_WARN("robot_saver2 not spawned, skipping sensor call");
		return true;
	}

	if (!bot_location.call(srv))
	{
		ROS_WARN("Failed to call ROS GetModelState service to get bot location");
		return false;
	}

	// FIXED: Convert world coordinates to grid indices
	int x = std::round(srv.response.pose.position.y / GRID_WIDTH); // y->row
	int y = std::round(srv.response.pose.position.x / GRID_WIDTH); // x->col
	int range = req.sensorRange;

	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid2[x - i][y] == SURVIVOR)
		{ // North
			res.objectNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid2[x + i][y] == SURVIVOR)
		{ // South
			res.objectSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid2[x][y - i] == SURVIVOR)
		{ // West
			res.objectWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid2[x][y + i] == SURVIVOR)
		{ // East
			res.objectEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}

	if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
		res.objectDetected = true;
	return true;
}