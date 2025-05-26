#include "ros/ros.h"
#include "std_msgs/String.h"
#include <curl/curl.h>
#include <sstream>

const int GRID_MIN = 0;
const int GRID_MAX = 7;

// int robot_x = 4;
// int robot_y = 4;
// float sonar_front = 1.2;
// float sonar_left = 0.4;
// float sonar_right = 0.5;

ros::Publisher proposed_pub;

// === GEMMA CALL ===
std::string sendToOllama(const std::string& prompt) {
    CURL* curl;
    CURLcode res;
    std::string response_string;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if (curl) {
        std::string postData = "{\"model\":\"gemma:3.4b\",\"prompt\":\"" + prompt + "\",\"stream\":false}";
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:11434/api/generate");
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, +[](void *contents, size_t size, size_t nmemb, std::string *output) {
            size_t totalSize = size * nmemb;
            output->append((char *)contents, totalSize);
            return totalSize;
        });
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

        res = curl_easy_perform(curl);
        if (res != CURLE_OK)
            ROS_ERROR("curl_easy_perform() failed: %s", curl_easy_strerror(res));

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    }

    curl_global_cleanup();

    size_t pos = response_string.find("\"response\":\"");
    if (pos != std::string::npos) {
        size_t start = pos + 12;
        size_t end = response_string.find("\"", start);
        return response_string.substr(start, end - start);
    }

    return "Invalid response from LLM.";
}

// === PROMPT BUILDING ===
std::string buildPrompt() {
    std::stringstream ss;
    ss << R"(You are a robotics assistant operating in a 8x8 grid (0–7). Respond ONLY with one movement command: "move north", "move south", "move east", "move west", or "stay".)"
       << "\nLocation: (" << robot_x << "," << robot_y << ")"
       << "\nSonar Front: " << sonar_front << "m"
       << "\nSonar Left: " << sonar_left << "m"
       << "\nSonar Right: " << sonar_right << "m"
       << "\n\nWhat action should the robot take?";
    return ss.str();
}

// === VALIDATOR RESPONSE ===
void validatorCallback(const std_msgs::String::ConstPtr& msg) {
    std::string result = msg->data;
    ROS_INFO("[Validator] Decision: %s", result.c_str());

    if (result == "valid") {
        ROS_INFO("Action approved: executing...");
        // (Optional) simulate action execution here
    } else {
        ROS_WARN("Action rejected by validator.");
    }
}

// === MAIN ===
int main(int argc, char **argv) {
    ros::init(argc, argv, "ai_validator_gemma");
    ros::NodeHandle nh;

    proposed_pub = nh.advertise<std_msgs::String>("/ai/proposed_action", 10);
    ros::Subscriber validator_sub = nh.subscribe("/ai/validated_action", 10, validatorCallback);

    ros::Rate loop_rate(0.2);  // 1 action every 5 seconds

    while (ros::ok()) {
        std::string prompt = buildPrompt();
        std::string action = sendToOllama(prompt);

        ROS_INFO("[Planner] Proposed Action: %s", action.c_str());

        std_msgs::String msg;
        msg.data = action;
        proposed_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
