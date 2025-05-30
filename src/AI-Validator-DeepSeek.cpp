// LLM: deepseek-r1:7b     Size: 4.7 GB    
#include <ros/ros.h>
#include "assignment_three/Plan.h"
#include "assignment_three/ValidationResult.h"
#include <curl/curl.h>
#include <string>

// Helper function to call Ollama (deepseek-r1:1.5b)
std::string call_ollama_deepseek(const std::string& prompt) {
    CURL* curl = curl_easy_init();
    std::string response;
    if (curl) {
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        std::string data = "{\"model\":\"deepseek-r1:1.5b\",\"prompt\":\"" + prompt + "\"}";
        curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:11434/api/generate");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, +[](void* ptr, size_t size, size_t nmemb, void* userdata) -> size_t {
            std::string* str = static_cast<std::string*>(userdata);
            str->append(static_cast<char*>(ptr), size * nmemb);
            return size * nmemb;
        });
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
    return response;
}

ros::Publisher val_pub;

void planCallback(const assignment_three::Plan::ConstPtr& msg) {
    assignment_three::ValidationResult result;
    std::string prompt = "Validate the following plan: " + msg->plan_steps;
    std::string ai_response = call_ollama_deepseek(prompt);
    // For demo, treat any non-empty response as valid
    result.is_valid = !ai_response.empty();
    result.reason = ai_response;
    val_pub.publish(result);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "validator_agent");
    ros::NodeHandle nh;
    val_pub = nh.advertise<assignment_three::ValidationResult>("validation_result", 10);
    ros::Subscriber plan_sub = nh.subscribe("plan", 10, planCallback);
    ros::spin();
    return 0;
}
