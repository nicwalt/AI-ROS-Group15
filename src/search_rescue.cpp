// search_rescue.cpp

#include <ros/ros.h>
#include "group_15/UpdateGrid.h"
#include "group_15/Sensor.h"
#include "communal_defines.cpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>
#include <random>
#include <fstream>
#include <queue>
#include <string>
#include <utility>

#define GAZEBO_SIMULATION_RATE 1
#define SubIsHome(sub_x, sub_y) (sub_x == SUB_START_X && sub_y == SUB_START_Y)

#define SURVEY_AREA 0
#define COLLECT_SURVIVORS 1
#define GO_HOME 2

std::string homeDir = getenv("HOME");
#define PAT_EXE_DIR homeDir + "/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe"
#define PAT_PATH_CSP_EXPLORE_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/explore.csp"
#define PAT_PATH_CSP_HOME_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/return_home.csp"
#define PAT_PATH_CSP_COLLECT_SURVIVORS_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/collect_survivors.csp"
#define PAT_OUTPUT_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/pat_output.txt"
#define PAT_WORLD_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/world.csp"
#define MAX_BFS_TIME 10
std::string PAT_CMD_EXPLORE = "mono " + PAT_EXE_DIR + " " + PAT_PATH_CSP_EXPLORE_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_GO_HOME_BFS = "timeout " + std::to_string(MAX_BFS_TIME) + "s mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_HOME_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_GO_HOME_DFS = "mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_HOME_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS_BFS = "timeout " + std::to_string(MAX_BFS_TIME) + "s mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_COLLECT_SURVIVORS_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS_DFS = "mono " + PAT_EXE_DIR + " " + PAT_PATH_CSP_COLLECT_SURVIVORS_DIR + " " + PAT_OUTPUT_DIR;

void detect_hostiles(group_15::Sensor &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
int detect_survivors(group_15::Sensor &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
std::pair<int, int> update_position(std::string &move, int &x, int &y);
void update_directions(std::queue<std::string> &q);
void generate_world(int (&world)[BOARD_H][BOARD_W]);
void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard);
std::vector<int> translate_world(int (&world)[BOARD_H][BOARD_W]);
void regenerate_moves(int (&current_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard, std::queue<std::string> &q, int &currentPath);
void execute_move(int (&current_world)[BOARD_H][BOARD_W], int (&true_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, std::pair<int, int> &new_coords);
std_msgs::Int32MultiArray createGrid(int (&true_world)[BOARD_H][BOARD_W]);

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "testing");
	ros::NodeHandle n;
	ros::ServiceClient gridClient = n.serviceClient<group_15::UpdateGrid>("/update_grid");
	ros::ServiceClient hostileSensorClient = n.serviceClient<group_15::Sensor>("/hostile_sensor");
	ros::ServiceClient survivorSensorClient = n.serviceClient<group_15::Sensor>("/survivor_sensor");
	group_15::UpdateGrid grid_srv;
	group_15::Sensor hostile_srv;
	group_15::Sensor survivor_srv;
	std_msgs::Int32MultiArray true_grid;

	int survivors_saved = 0;
	int survivors_seen = 0;
	int OnBoard = 0;
	int true_world[BOARD_H][BOARD_W];
	int current_world[BOARD_H][BOARD_W];
	for (int i = 0; i < BOARD_H; i++)
		for (int j = 0; j < BOARD_W; j++)
			current_world[i][j] = EMPTY;
	current_world[SUB_START_X][SUB_START_Y] = VISITED;
	int sub_x = SUB_START_X;
	int sub_y = SUB_START_Y;
	int currentPath = SURVEY_AREA;

	generate_world(true_world);

	true_grid = createGrid(true_world);
	grid_srv.request.grid = true_grid;

	if (!gridClient.call(grid_srv))
	{
		ROS_ERROR("Failed to call update_grid service");
		return EXIT_FAILURE;
	}

	std::queue<std::string> q;
	hostile_srv.request.sensorRange = HOSTILE_DETECTION_RANGE;
	survivor_srv.request.sensorRange = SURVIVOR_DETECTION_RANGE;
	if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
	{
		ROS_ERROR("Failed to call sensor services");
		return EXIT_FAILURE;
	}

	detect_hostiles(hostile_srv, current_world, sub_x, sub_y);
	int newSurvivorsDetected = detect_survivors(survivor_srv, current_world, sub_x, sub_y);

	if (newSurvivorsDetected)
	{
		// ROS_INFO("New survivor(s) detected!");
		survivors_seen += newSurvivorsDetected;
		currentPath = COLLECT_SURVIVORS;
    }

	regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);

	std::string next_move;

	ros::Rate rate(GAZEBO_SIMULATION_RATE);

	while (ros::ok())
	{
		// ROS_INFO("-- Start of move cycle --");

		if (SubIsHome(sub_x, sub_y) && OnBoard)
		{
			survivors_saved += OnBoard;
			std::cout << "Saved " << OnBoard << " survivors. Total survivors now saved: " << survivors_saved << std::endl;
			OnBoard = 0;
		}

        if (q.empty())
		{
			if ((survivors_saved + OnBoard) == SURVIVOR_COUNT)
			{
				if (SubIsHome(sub_x, sub_y))
				{
					// ROS_INFO("Mission successful!\nFinal internal representation of environment:");
					for (int i = 0; i < BOARD_H; i++)
					{
						for (int j = 0; j < BOARD_W; j++)
						{
							if (current_world[i][j] != VISITED)
								std::cout << " ";
							std::cout << current_world[i][j] << " ";
						}
						std::cout << std::endl;
					}
					return EXIT_SUCCESS;
				}
				else 
					currentPath = GO_HOME;
			}
			else
			{
				// ROS_INFO("We've run out of moves, but there's still people left to be saved!");
				if (survivors_seen > (survivors_saved + OnBoard))
					currentPath = COLLECT_SURVIVORS;
				else											
					currentPath = SURVEY_AREA;
			}
			regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
		}
		next_move = std::string(q.front());
		q.pop();
		// ROS_INFO("Next move is: %s", next_move.c_str());

		std::pair<int, int> new_coords = update_position(next_move, sub_x, sub_y);
		int new_x = new_coords.first;
		int new_y = new_coords.second;

		if (current_world[new_x][new_y] == HOSTILE)
		{
			// ROS_INFO("About to move into hostile, recalculating PAT directions");
			regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
			rate.sleep();
			continue;
		}

		if (current_world[new_x][new_y] == SURVIVOR)
		{
			// ROS_INFO("About to pick up a survivor :) Hooray!");
			OnBoard++;
			std::cout << "Now have " << OnBoard << " survivors onboard" << std::endl;
		}

		execute_move(current_world, true_world, sub_x, sub_y, new_coords);

		true_grid.data = translate_world(true_world);
		grid_srv.request.grid = true_grid;

		if (!gridClient.call(grid_srv))
		{
			ROS_ERROR("Failed to call update_grid service");
			return EXIT_FAILURE;
		}

		sub_x = new_x;
		sub_y = new_y;

		if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
		{
			ROS_ERROR("Failed to call sensor services");
			return EXIT_FAILURE;
		}

		// update our current world with any detected hostiles or survivors
		detect_hostiles(hostile_srv, current_world, sub_x, sub_y);
		newSurvivorsDetected = detect_survivors(survivor_srv, current_world, sub_x, sub_y);

		if (newSurvivorsDetected)
		{
			// ROS_INFO("New survivor(s) detected!");
			survivors_seen += newSurvivorsDetected;
			currentPath = COLLECT_SURVIVORS;
			regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
		}

		// ROS_INFO("-- End of move cycle --\n");
		rate.sleep();
		ros::spinOnce();
	}
}

void detect_hostiles(group_15::Sensor &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y)
{
	if (hostile_srv.response.objectEast)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.eastRadar[i])
			{
				curr_world[sub_x][sub_y + 1 + i] = HOSTILE;
				// ROS_INFO("Robot has detected a hostile east!");
			}
	if (hostile_srv.response.objectWest)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.westRadar[i])
			{
				curr_world[sub_x][sub_y - 1 - i] = HOSTILE;
				// ROS_INFO("Robot has detected a hostile west!");
			}
	if (hostile_srv.response.objectNorth)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.northRadar[i])
			{
				curr_world[sub_x - 1 - i][sub_y] = HOSTILE;
				// ROS_INFO("Robot has detected a hostile north!");
			}
	if (hostile_srv.response.objectSouth)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.southRadar[i])
			{
				curr_world[sub_x + 1 + i][sub_y] = HOSTILE;
				// ROS_INFO("Robot has detected a hostile south!");
			}
}

int detect_survivors(group_15::Sensor &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y)
{
	int newSurvivorsDetected = 0;
	if (survivor_srv.response.objectEast)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.eastRadar[i])
				if (curr_world[sub_x][sub_y + 1 + i] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x][sub_y + 1 + i] = SURVIVOR;
					// ROS_INFO("Robot has detected a survivor east!");
				}
	if (survivor_srv.response.objectWest)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.westRadar[i])
				if (curr_world[sub_x][sub_y - 1 - i] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x][sub_y - 1 - i] = SURVIVOR;
					// ROS_INFO("Robot has detected a survivor west!");
				}
	if (survivor_srv.response.objectNorth)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.northRadar[i])
				if (curr_world[sub_x - 1 - i][sub_y] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x - 1 - i][sub_y] = SURVIVOR;
					// ROS_INFO("Robot has detected a survivor north!");
				}
	if (survivor_srv.response.objectSouth)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.southRadar[i])
				if (curr_world[sub_x + 1 + i][sub_y] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[sub_x + 1 + i][sub_y] = SURVIVOR;
					// ROS_INFO("Robot has detected a survivor south!");
				}
	return newSurvivorsDetected;
}

std::pair<int, int> update_position(std::string &move, int &x, int &y)
{
	if (move == "moveRight")
		return {x, y + 1};
	else if (move == "moveLeft")
		return {x, y - 1};
	else if (move == "moveUp")
		return {x - 1, y};
	else if (move == "moveDown")
		return {x + 1, y};
	std::cerr << "update_position found invalid move: " << move << std::endl;
	return {x, y};
}

void update_directions(std::queue<std::string> &q)
{
	std::ifstream pat_output(PAT_OUTPUT_DIR);
	if (!pat_output.is_open())
	{
		std::cerr << "Failed to open PAT output file!" << std::endl;
		exit(1);
	}
	while (!q.empty())
		q.pop();
	std::string line;
	std::string move;
	while (getline(pat_output, line))
	{							
		if (line[0] == '<') 
		{
			std::istringstream ss(line);
			ss >> move;
			while (ss >> move) 
			{
				ss >> move;
				q.push(move); 
			}
			q.back().pop_back();
			break; 
		}
	}
	pat_output.close();
	return;
}

void generate_world(int (&world)[BOARD_H][BOARD_W])
{
	for (int i = 0; i < BOARD_H; ++i)
		for (int j = 0; j < BOARD_W; ++j)
			world[i][j] = EMPTY;

	world[SUB_START_X][SUB_START_Y] = SUB;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> rowDist(0, BOARD_H - 1);
	std::uniform_int_distribution<int> colDist(0, BOARD_W - 1);

	int placed = 0;
	while (placed < SURVIVOR_COUNT)
	{
		int rand_row = rowDist(gen);
		int rand_col = colDist(gen);
		if (world[rand_row][rand_col] == EMPTY)
		{
			world[rand_row][rand_col] = SURVIVOR;
			placed++;
		}
	}

	placed = 0;
	while (placed < HOSTILE_COUNT)
	{
		int rand_row = rowDist(gen);
		int rand_col = colDist(gen);
		if (world[rand_row][rand_col] == EMPTY)
		{
			world[rand_row][rand_col] = HOSTILE;
			placed++;
		}
	}
}

void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard)
{
	std::ofstream file(PAT_WORLD_DIR);
	if (!file.is_open())
	{
		std::cerr << "Failed to save the current world to world.csp" << std::endl;
		exit(1);
	}
	// ROS_INFO("Writing robot's current interpretation to world file.");

	file << "#define Visited " << VISITED << ";\n";
	file << "#define Unvisited " << EMPTY << ";\n";
	file << "#define Sub " << SUB << ";\n";
	file << "#define Hostile " << HOSTILE << ";\n";
	file << "#define Survivor " << SURVIVOR << ";\n\n";
	file << "#define SUB_HOME_X " << SUB_START_X << ";\n";
	file << "#define SUB_HOME_Y " << SUB_START_Y << ";\n";
	file << "#define Rows " << BOARD_H << ";\n";
	file << "#define Cols " << BOARD_W << ";\n";
	file << "#define maxCapacity " << SUB_CAP << ";\n";

	file << "\nvar world[Rows][Cols]:{Visited..Survivor} = [\n";
	for (int i = 0; i < BOARD_H; i++)
	{
		for (int j = 0; j < BOARD_W; j++)
		{
			if (i == BOARD_H - 1 && j == BOARD_W - 1)
				file << world[i][j];
			else
				file << world[i][j] << ", ";
		}
		file << "\n";
	}
	file << "];\n\n";

	file << "// Position of sub\n";
	file << "var xpos:{0..Rows-1} = " << sub_x << ";\n";
	file << "var ypos:{0..Cols-1} = " << sub_y << ";\n";
	file << "var onBoard:{0..maxCapacity} = " << onBoard << ";\n";

	file.close();
}

std::vector<int> translate_world(int (&world)[BOARD_H][BOARD_W])
{
	std::vector<int> vec(BOARD_W * BOARD_H, EMPTY);
	for (int i = 0; i < BOARD_H; i++)
		for (int j = 0; j < BOARD_W; j++)
			vec[i * BOARD_W + j] = world[i][j];
	return vec;
}

void regenerate_moves(int (&current_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard, std::queue<std::string> &q, int &currentPath)
{
	generate_known_world(current_world, sub_x, sub_y, onBoard);

	if (currentPath == SURVEY_AREA)
	{
		// ROS_INFO("Calculating a path to survey remaining area");
		std::system(PAT_CMD_EXPLORE.c_str());
	}
	else if (currentPath == COLLECT_SURVIVORS)
	{
		// ROS_INFO("Calculating a path to collect remaining survivors");
		int status = std::system(PAT_CMD_COLLECT_SURVIVORS_BFS.c_str());
		if (status < 0)
		{
			std::cout << "There has been a fatal error!: " << strerror(errno) << '\n';
			exit(1);
		}
		else
		{
			if (WIFEXITED(status))
			{
				if (WEXITSTATUS(status) == 124)
				{ 
                    // ROS_INFO("BFS Path calculation for survivor collection took too long, now calculating DFS path.");
					std::system(PAT_CMD_COLLECT_SURVIVORS_DFS.c_str());
				}
			}
			// system call must have been killed, or fatal error
			else
			{
				std::cout << "PAT call was killed :(\n";
				exit(1);
			}
		}
	}
	else if (currentPath == GO_HOME)
	{
		// ROS_INFO("Calculating a path to go home");
		int status = std::system(PAT_CMD_GO_HOME_BFS.c_str());
		if (status < 0)
		{
			std::cout << "There has been a fatal error!: " << strerror(errno) << '\n';
			exit(1);
		}
		else
		{
			if (WIFEXITED(status))
			{
				if (WEXITSTATUS(status) == 124)
				{ 
                    // ROS_INFO("BFS Path calculation for going home took too long, now calculating DFS path.");
					std::system(PAT_CMD_GO_HOME_DFS.c_str());
				}
			}
			else
			{
				std::cout << "PAT call was killed :(\n";
				exit(1);
			}
		}
	}
	else
	{
		ROS_WARN("Received unknown path command! Aborting mission!");
		exit(1);
	}

	update_directions(q);
}

void execute_move(int (&current_world)[BOARD_H][BOARD_W], int (&true_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, std::pair<int, int> &new_coords)
{
	int new_x = new_coords.first;
	int new_y = new_coords.second;
	current_world[new_x][new_y] = VISITED;

	true_world[sub_x][sub_y] = VISITED;
	true_world[new_x][new_y] = SUB;
}

std_msgs::Int32MultiArray createGrid(int (&true_world)[BOARD_H][BOARD_W])
{
	std_msgs::Int32MultiArray true_grid;
	true_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
	true_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
	true_grid.layout.dim[0].label = "height";
	true_grid.layout.dim[1].label = "width";
	true_grid.layout.dim[0].size = BOARD_H;
	true_grid.layout.dim[1].size = BOARD_W;
	true_grid.layout.dim[0].stride = BOARD_H * BOARD_W;
	true_grid.layout.dim[1].stride = BOARD_W;
	true_grid.layout.data_offset = 0;
	true_grid.data = translate_world(true_world);
	return true_grid;
}