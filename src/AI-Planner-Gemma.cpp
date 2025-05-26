// LLM: gemma3:4b  size: 3.3 GB   


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <curl/curl.h>
#include <sstream>

std::string sendToOllama(const std::string& prompt) {
    CURL* curl;
    CURLcode res;
    std::string response_string;
    std::string header_string;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if(curl) {
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

        if(res != CURLE_OK)
            ROS_ERROR("curl_easy_perform() failed: %s", curl_easy_strerror(res));

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    }

    curl_global_cleanup();

    // Parse output manually for "response" field (basic handling)
    size_t pos = response_string.find("\"response\":\"");
    if (pos != std::string::npos) {
        size_t start = pos + 12;
        size_t end = response_string.find("\"", start);
        return response_string.substr(start, end - start);
    }

    return "Invalid response from LLM.";
}

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: [%s]", msg->data.c_str());

    std::string ai_response = sendToOllama(msg->data);
    ROS_INFO("Gemma says: %s", ai_response.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ai_validator_gemma");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ai_input", 1000, callback);
    ROS_INFO("AI Validator Gemma Node Started.");
    ros::spin();

    return 0;
}
