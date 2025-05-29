#include <ros/ros.h>
#include "group_15/UpdateGrid2.h"
#include "group_15/Sensor2.h"
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
#include <sstream>  // Added for proper string parsing
#include <cstring>  // Added for strerror
#include <sys/wait.h>  // Added for WEXITSTATUS

#define GAZEBO_SIMULATION_RATE 1
#define SubIsHome(sub_x, sub_y) (sub_x == SUB_START_X && sub_y == SUB_START_Y)

#define SURVEY_AREA 0
#define COLLECT_SURVIVORS 1
#define GO_HOME 2

std::string homeDir = getenv("HOME");
#define PAT_EXE_DIR homeDir + "/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe"
#define PAT_PATH2_CSP_EXPLORE_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/explore2.csp"
#define PAT_PATH2_CSP_HOME_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/return_home2.csp"
#define PAT_PATH2_CSP_COLLECT_SURVIVORS_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/collect_survivors2.csp"
#define pat_output2_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/pat_output2.txt"
#define PAT_WORLD_DIR homeDir + "/catkin_workspace/src/AI-ROS-Group15/pat/world2.csp"
#define MAX_BFS_TIME 10
std::string PAT_CMD_EXPLORE = "mono " + PAT_EXE_DIR + " " + PAT_PATH2_CSP_EXPLORE_DIR + " " + pat_output2_DIR;
std::string PAT_CMD_GO_HOME_BFS = "timeout " + std::to_string(MAX_BFS_TIME) + "s mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH2_CSP_HOME_DIR + " " + pat_output2_DIR;
std::string PAT_CMD_GO_HOME_DFS = "mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH2_CSP_HOME_DIR + " " + pat_output2_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS_BFS = "timeout " + std::to_string(MAX_BFS_TIME) + "s mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH2_CSP_COLLECT_SURVIVORS_DIR + " " + pat_output2_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS_DFS = "mono " + PAT_EXE_DIR + " " + PAT_PATH2_CSP_COLLECT_SURVIVORS_DIR + " " + pat_output2_DIR;

void detect_hostiles(group_15::Sensor2 &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
int detect_survivors(group_15::Sensor2 &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
std::pair<int, int> update_position(std::string &move, int &x, int &y);
void update_directions(std::queue<std::string> &q2);
void generate_world(int (&world)[BOARD_H][BOARD_W]);
void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard);
std::vector<int> translate_world(int (&world)[BOARD_H][BOARD_W]);
void regenerate_moves(int (&current_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard, std::queue<std::string> &q2, int &currentPATH2);
void execute_move(int (&current_world)[BOARD_H][BOARD_W], int (&true_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, std::pair<int, int> &new_coords);
std_msgs::Int32MultiArray createGrid(int (&true_world)[BOARD_H][BOARD_W]);


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "testing2");
	ros::NodeHandle n;
	ros::ServiceClient gridClient = n.serviceClient<group_15::UpdateGrid2>("/update_grid2");
	ros::ServiceClient hostileSensorClient = n.serviceClient<group_15::Sensor2>("/hostile_sensor");
	ros::ServiceClient survivorSensorClient = n.serviceClient<group_15::Sensor2>("/survivor_sensor");
	group_15::UpdateGrid2 grid_srv;
	group_15::Sensor2 hostile_srv;
	group_15::Sensor2 survivor_srv;
	std_msgs::Int32MultiArray true_grid;

	int survivors_saved2 = 0;
	int survivors_seen2 = 0;
	int OnBoard2 = 0;
	int true_world2[BOARD_H][BOARD_W];
	int current_world2[BOARD_H][BOARD_W];
	
	// Initialize all grids to EMPTY
	for (int i = 0; i < BOARD_H; i++) {
		for (int j = 0; j < BOARD_W; j++) {
			true_world2[i][j] = EMPTY;
			current_world2[i][j] = EMPTY;
		}
	}
	
	current_world2[SUB2_START_X][SUB2_START_Y] = VISITED;
	int SUB2_x = SUB2_START_X;
	int SUB2_y = SUB2_START_Y;
	int currentPATH2 = SURVEY_AREA;

	generate_world(true_world2);

	true_grid = createGrid(true_world2);
	grid_srv.request.grid = true_grid;

	if (!gridClient.call(grid_srv))
	{
		ROS_ERROR("Failed to call update_grid2 service");
		return EXIT_FAILURE;
	}

	std::queue<std::string> q2;
	hostile_srv.request.sensorRange = HOSTILE_DETECTION_RANGE;
	survivor_srv.request.sensorRange = SURVIVOR_DETECTION_RANGE;
	if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
	{
		ROS_ERROR("Failed to call sensor services");
		return EXIT_FAILURE;
	}

	detect_hostiles(hostile_srv, current_world2, SUB2_x, SUB2_y);
	int newSurvivorsDetected = detect_survivors(survivor_srv, current_world2, SUB2_x, SUB2_y);

	if (newSurvivorsDetected)
	{
		survivors_seen2 += newSurvivorsDetected;
		currentPATH2 = COLLECT_SURVIVORS;
    }

	regenerate_moves(current_world2, SUB2_x, SUB2_y, OnBoard2, q2, currentPATH2);

	// Check if moves were generated
	if (q2.empty()) {
		ROS_ERROR("No moves generated after initial PAT call");
		return EXIT_FAILURE;
	}

	std::string next_move;

	ros::Rate rate(GAZEBO_SIMULATION_RATE);

	while (ros::ok())
	{
		if (SubIsHome(SUB2_x, SUB2_y) && OnBoard2)
		{
			survivors_saved2 += OnBoard2;
			ROS_INFO("Saved %d survivors. Total saved: %d", OnBoard2, survivors_saved2);
			OnBoard2 = 0;
		}

        if (q2.empty())
		{
			if ((survivors_saved2 + OnBoard2) == SURVIVOR_COUNT)
			{
				if (SubIsHome(SUB2_x, SUB2_y))
				{
					ROS_INFO("Mission completed successfully!");
					return EXIT_SUCCESS;
				}
				else 
					currentPATH2 = GO_HOME;
			}
			else
			{
				if (survivors_seen2 > (survivors_saved2 + OnBoard2))
					currentPATH2 = COLLECT_SURVIVORS;
				else											
					currentPATH2 = SURVEY_AREA;
			}
			regenerate_moves(current_world2, SUB2_x, SUB2_y, OnBoard2, q2, currentPATH2);
			
			// Re-check if moves were generated
			if (q2.empty()) {
				ROS_ERROR("No moves generated after PAT recalculation");
				return EXIT_FAILURE;
			}
		}
		next_move = q2.front();
		q2.pop();

		std::pair<int, int> new_coords = update_position(next_move, SUB2_x, SUB2_y);
		int new_x = new_coords.first;
		int new_y = new_coords.second;

		// Validate new coordinates
		if (new_x < 0 || new_x >= BOARD_H || new_y < 0 || new_y >= BOARD_W) {
			ROS_ERROR("Invalid move coordinates: (%d, %d)", new_x, new_y);
			continue;
		}

		if (current_world2[new_x][new_y] == HOSTILE)
		{
			ROS_WARN("Hostile detected at (%d, %d), recalculating PATH2", new_x, new_y);
			regenerate_moves(current_world2, SUB2_x, SUB2_y, OnBoard2, q2, currentPATH2);
			rate.sleep();
			continue;
		}

		if (current_world2[new_x][new_y] == SURVIVOR)
		{
			OnBoard2++;
			ROS_INFO("Picked up survivor. Now carrying: %d", OnBoard2);
		}

		execute_move(current_world2, true_world2, SUB2_x, SUB2_y, new_coords);

		true_grid.data = translate_world(true_world2);
		grid_srv.request.grid = true_grid;

		if (!gridClient.call(grid_srv))
		{
			ROS_ERROR("Failed to call update_grid2 service");
			return EXIT_FAILURE;
		}

		SUB2_x = new_x;
		SUB2_y = new_y;

		if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
		{
			ROS_ERROR("Failed to call sensor services");
			return EXIT_FAILURE;
		}

		detect_hostiles(hostile_srv, current_world2, SUB2_x, SUB2_y);
		newSurvivorsDetected = detect_survivors(survivor_srv, current_world2, SUB2_x, SUB2_y);

		if (newSurvivorsDetected)
		{
			survivors_seen2 += newSurvivorsDetected;
			currentPATH2 = COLLECT_SURVIVORS;
			regenerate_moves(current_world2, SUB2_x, SUB2_y, OnBoard2, q2, currentPATH2);
		}

		rate.sleep();
		ros::spinOnce();
	}
}

void detect_hostiles(group_15::Sensor2 &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &SUB2_x, int &SUB2_y)
{
	// FIX: Convert sensor indices to grid coordinates
	if (hostile_srv.response.objectEast)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.eastRadar[i])
				curr_world[SUB2_x][SUB2_y + 1 + i] = HOSTILE;
	if (hostile_srv.response.objectWest)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.westRadar[i])
				curr_world[SUB2_x][SUB2_y - 1 - i] = HOSTILE;
	if (hostile_srv.response.objectNorth)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.northRadar[i])
				curr_world[SUB2_x - 1 - i][SUB2_y] = HOSTILE;
	if (hostile_srv.response.objectSouth)
		for (int i = 0; i < hostile_srv.request.sensorRange; i++)
			if (hostile_srv.response.southRadar[i])
				curr_world[SUB2_x + 1 + i][SUB2_y] = HOSTILE;
}

int detect_survivors(group_15::Sensor2 &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &SUB2_x, int &SUB2_y)
{
	int newSurvivorsDetected = 0;
	if (survivor_srv.response.objectEast)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.eastRadar[i])
				if (curr_world[SUB2_x][SUB2_y + 1 + i] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[SUB2_x][SUB2_y + 1 + i] = SURVIVOR;
				}
	if (survivor_srv.response.objectWest)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.westRadar[i])
				if (curr_world[SUB2_x][SUB2_y - 1 - i] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[SUB2_x][SUB2_y - 1 - i] = SURVIVOR;
				}
	if (survivor_srv.response.objectNorth)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.northRadar[i])
				if (curr_world[SUB2_x - 1 - i][SUB2_y] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[SUB2_x - 1 - i][SUB2_y] = SURVIVOR;
				}
	if (survivor_srv.response.objectSouth)
		for (int i = 0; i < survivor_srv.request.sensorRange; i++)
			if (survivor_srv.response.southRadar[i])
				if (curr_world[SUB2_x + 1 + i][SUB2_y] != SURVIVOR)
				{
					newSurvivorsDetected++;
					curr_world[SUB2_x + 1 + i][SUB2_y] = SURVIVOR;
				}
	return newSurvivorsDetected;
}

// ... (rest of the functions remain unchanged) ...

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

void update_directions(std::queue<std::string> &q2)
{
	std::ifstream pat_output2(pat_output2_DIR);
	if (!pat_output2.is_open())
	{
		// ROS_ERROR("Failed to open PAT output file: %s", pat_output2_DIR);
		exit(1);
	}
	
	// Clear existing moves
	while (!q2.empty()) q2.pop();
	
	std::string line;
	while (getline(pat_output2, line))
	{
		// Look for the solution line
		if (line.find("The result is:") != std::string::npos) {
			// Extract moves from solution line
			size_t start = line.find("<") + 1;
			size_t end = line.find(">");
			if (start == std::string::npos || end == std::string::npos) {
				ROS_ERROR("Invalid solution format in PAT output");
				break;
			}
			
			std::string moves = line.substr(start, end - start);
			std::istringstream ss(moves);
			std::string move;
			
			while (getline(ss, move, ';')) {
				// Trim whitespace
				move.erase(0, move.find_first_not_of(" \t"));
				move.erase(move.find_last_not_of(" \t") + 1);
				
				if (!move.empty()) {
					q2.push(move);
					// ROS_DEBUG("Added move: %s", move.c_str());
				}
			}
			break;
		}
	}
	pat_output2.close();
	
	if (q2.empty()) {
		ROS_ERROR("No valid moves found in PAT output");
	}
}

void generate_world(int (&world)[BOARD_H][BOARD_W])
{
	// Initialize entire grid to EMPTY
	for (int i = 0; i < BOARD_H; i++) {
		for (int j = 0; j < BOARD_W; j++) {
			world[i][j] = EMPTY;
		}
	}

	// Place robot at start position
	world[SUB2_START_X][SUB2_START_Y] = SUB2;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> rowDist(0, BOARD_H - 1);
	std::uniform_int_distribution<int> colDist(0, BOARD_W - 1);

	// // Place survivors
	// int placed = 0;
	// while (placed < SURVIVOR_COUNT)
	// {
	// 	int rand_row = rowDist(gen);
	// 	int rand_col = colDist(gen);
	// 	if (world[rand_row][rand_col] == EMPTY)
	// 	{
	// 		world[rand_row][rand_col] = SURVIVOR;
	// 		placed++;
	// 	}
	// }

	// Place hostiles
	// placed = 0;
	// while (placed < HOSTILE_COUNT)
	// {
	// 	int rand_row = rowDist(gen);
	// 	int rand_col = colDist(gen);
	// 	if (world[rand_row][rand_col] == EMPTY)
	// 	{
	// 		world[rand_row][rand_col] = HOSTILE;
	// 		placed++;
	// 	}
	// }
}

void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &SUB2_x, int &SUB2_y, int &onBoard)
{
	std::ofstream file(PAT_WORLD_DIR);
	if (!file.is_open())
	{
		// ROS_ERROR("Failed to open world file: %s", PAT_WORLD_DIR);
		exit(1);
	}

	file << "#define Visited " << VISITED << ";\n";
	file << "#define Unvisited " << EMPTY << ";\n";
	file << "#define SUB2 " << SUB2 << ";\n";
	file << "#define Hostile " << HOSTILE << ";\n";
	file << "#define Survivor " << SURVIVOR << ";\n\n";
	file << "#define SUB2_HOME_X " << SUB2_START_X << ";\n";
	file << "#define SUB2_HOME_Y " << SUB2_START_Y << ";\n";
	file << "#define Rows " << BOARD_H << ";\n";
	file << "#define Cols " << BOARD_W << ";\n";
	file << "#define maxCapacity " << SUB2_CAP << ";\n";

	file << "\nvar world[Rows][Cols]:{Visited..Survivor} = [\n";
	for (int i = 0; i < BOARD_H; i++)
	{
		file << "[";
		for (int j = 0; j < BOARD_W; j++)
		{
			file << world[i][j];
			if (j < BOARD_W - 1) file << ", ";
		}
		file << "]";
		if (i < BOARD_H - 1) file << ",";
		file << "\n";
	}
	file << "];\n\n";

	file << "// Position of SUB2\n";
	file << "var xpos:{0..Rows-1} = " << SUB2_x << ";\n";
	file << "var ypos:{0..Cols-1} = " << SUB2_y << ";\n";
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

void regenerate_moves(int (&current_world2)[BOARD_H][BOARD_W], int &SUB2_x, int &SUB2_y, int &onBoard, std::queue<std::string> &q2, int &currentPATH2)
{
	generate_known_world(current_world2, SUB2_x, SUB2_y, onBoard);

	if (currentPATH2 == SURVEY_AREA)
	{
		// ROS_INFO("Calculating a PATH2 to survey remaining area");
		std::system(PAT_CMD_EXPLORE.c_str());
	}
	else if (currentPATH2 == COLLECT_SURVIVORS)
	{
		// ROS_INFO("Calculating a PATH2 to collect remaining survivors");
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
                    // ROS_INFO("BFS PATH2 calculation for survivor collection took too long, now calculating DFS PATH2.");
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
	else if (currentPATH2 == GO_HOME)
	{
		// ROS_INFO("Calculating a PATH2 to go home");
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
                    // ROS_INFO("BFS PATH2 calculation for going home took too long, now calculating DFS PATH2.");
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
		ROS_WARN("Received unknown PATH2 command! Aborting mission!");
		exit(1);
	}

	update_directions(q2);
}

void execute_move(int (&current_world2)[BOARD_H][BOARD_W], int (&true_world2)[BOARD_H][BOARD_W], int &SUB2_x, int &SUB2_y, std::pair<int, int> &new_coords)
{
	int new_x = new_coords.first;
	int new_y = new_coords.second;
	current_world2[new_x][new_y] = VISITED;

	true_world2[SUB2_x][SUB2_y] = VISITED;
	true_world2[new_x][new_y] = SUB2;
}

std_msgs::Int32MultiArray createGrid(int (&true_world2)[BOARD_H][BOARD_W])
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
	true_grid.data = translate_world(true_world2);
	return true_grid;
}