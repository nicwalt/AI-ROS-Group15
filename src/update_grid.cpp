#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/ros.h>
#include <streambuf>
#include <fstream>
#include <map>
#include <cmath>
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
gazebo_msgs::GetModelState srv;
int currentGrid[BOARD_H][BOARD_W];

geometry_msgs::Point coordinates[BOARD_H][BOARD_W];

ros::ServiceClient spawnClient;
ros::ServiceClient deleteClient;
ros::ServiceClient setClient;

int numSurvivors = 0;
int numHostiles = 0;
bool robot_saverSpawned = false;
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
bool updateGrid(group_15::UpdateGrid::Request &req, group_15::UpdateGrid::Response &res);
bool hostileSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res);
bool survivorSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_object_manager");
	ros::NodeHandle n;
	bot_location = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	srv.request.model_name = "robot_saver";

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
	ros::ServiceServer HostileSenService = n.advertiseService("hostile_sensor", hostileSensor);
	ros::ServiceServer SurvivorSenService = n.advertiseService("survivor_sensor", survivorSensor);
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
		modelName = "robot_saver";
		modelKey = "robot_saver";
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


bool updateGrid(group_15::UpdateGrid::Request &req, group_15::UpdateGrid::Response &res)
{
	std_msgs::Int32MultiArray read_grid = req.grid;
	gazebo_msgs::SetModelState set;
	gazebo_msgs::DeleteModel del;
	gazebo_msgs::SpawnModel spawn;

	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
		{
			int oldIndex = currentGrid[i][j];
			int newIndex = read_grid.data[i * BOARD_W + j];
			if (oldIndex != newIndex)
			{
				geometry_msgs::Point point = coordinates[i][j];

				if (oldIndex == EMPTY && newIndex == SURVIVOR)
				{
					spawn = createSpawnRequest(SURVIVOR, point);
					objectPositions[point] = spawn.request.model_name;
					spawnClient.call(spawn);
				}
				if (oldIndex == SURVIVOR && newIndex == SUB)
				{
					del.request.model_name = objectPositions[point];
					deleteClient.call(del);
					objectPositions.erase(point);
					set.request.model_state.model_name = "robot_saver";
					set.request.model_state.pose.position = point;
					setClient.call(set);
				}
				if (oldIndex == EMPTY && newIndex == HOSTILE)
				{
					spawn = createSpawnRequest(HOSTILE, point);
					objectPositions[point] = spawn.request.model_name;
					spawnClient.call(spawn);
				}
				if ((oldIndex == EMPTY || oldIndex == VISITED) && newIndex == SUB)
				{
					if (robot_saverSpawned)
					{
						ROS_INFO("Moving sub to pos: (%.0f, %.0f)", point.x, point.y);
						set.request.model_state.model_name = "robot_saver";
						set.request.model_state.pose.position = point;
						setClient.call(set);
					}
					else
					{
						spawn = createSpawnRequest(SUB, point);
						objectPositions[point] = spawn.request.model_name;
						spawnClient.call(spawn);
						robot_saverSpawned = true;
					}
				}
				currentGrid[i][j] = newIndex;
			}
		}

	res.altered_grid = req.grid;
	return true;
}

bool hostileSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res)
{
	res.objectNorth = false;
	res.objectSouth = false;
	res.objectWest = false;
	res.objectEast = false;
	res.objectDetected = false;

	if (!bot_location.call(srv))
	{
		ROS_WARN("Failed to call ROS GetModelState service to get bot location");
		return false;
	}

	int x = std::round(srv.response.pose.position.x);
	int y = std::round(srv.response.pose.position.y);
	int range = req.sensorRange;

	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid[x - i][y] == HOSTILE)
		{ // North
			res.objectNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid[x + i][y] == HOSTILE)
		{ // South
			res.objectSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid[x][y - i] == HOSTILE)
		{ // West
			res.objectWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid[x][y + i] == HOSTILE)
		{ // East
			res.objectEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}

	if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
		res.objectDetected = true;
	return true;
}

bool survivorSensor(group_15::Sensor::Request &req, group_15::Sensor::Response &res)
{
	res.objectNorth = false;
	res.objectSouth = false;
	res.objectWest = false;
	res.objectEast = false;
	res.objectDetected = false;

	if (!bot_location.call(srv))
	{
		ROS_WARN("Failed to call ROS GetModelState service to get bot location");
		return false;
	}

	int x = std::round(srv.response.pose.position.x);
	int y = std::round(srv.response.pose.position.y);
	int range = req.sensorRange;

	res.northRadar = std::vector<int32_t>(range, 0);
	res.southRadar = std::vector<int32_t>(range, 0);
	res.eastRadar = std::vector<int32_t>(range, 0);
	res.westRadar = std::vector<int32_t>(range, 0);

	for (int i = 1; i <= range; ++i)
	{
		if (x - i >= 0 && currentGrid[x - i][y] == SURVIVOR)
		{ // North
			res.objectNorth = true;
			res.northRadar[i - 1] = 1;
		}
		if (x + i < BOARD_H && currentGrid[x + i][y] == SURVIVOR)
		{ // South
			res.objectSouth = true;
			res.southRadar[i - 1] = 1;
		}
		if (y - i >= 0 && currentGrid[x][y - i] == SURVIVOR)
		{ // West
			res.objectWest = true;
			res.westRadar[i - 1] = 1;
		}
		if (y + i < BOARD_W && currentGrid[x][y + i] == SURVIVOR)
		{ // East
			res.objectEast = true;
			res.eastRadar[i - 1] = 1;
		}
	}

	if (res.objectNorth || res.objectEast || res.objectSouth || res.objectWest)
		res.objectDetected = true;
	return true;
}