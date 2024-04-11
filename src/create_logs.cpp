#include <filesystem>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>


int main(int argc, char **argv) {

    // Initialize Node and read in global parameters
    ros::init(argc, argv, "log_setup");
    ros::NodeHandle n;

    // I/O parameters
    std::string absolutePackagePath = ros::package::getPath("urquhart");
    std::string outputPath;
    n.param<std::string>("outputDirName", outputPath, "GH_OUT");


    bool isLogging = n.param("logging", false);
    if (isLogging) {
        std::cout << "Creating output directory: '" << outputPath << "' ... ";
        outputPath = absolutePackagePath+"/output/"+outputPath;
        std::filesystem::remove_all(outputPath);
        std::filesystem::create_directory(outputPath);
        std::filesystem::create_directory(outputPath+"/global");
        std::filesystem::create_directory(outputPath+"/global/p");
        std::filesystem::create_directory(outputPath+"/global/d");
        std::filesystem::create_directory(outputPath+"/global/t");
        std::filesystem::create_directory(outputPath+"/global/h");
        std::filesystem::create_directory(outputPath+"/global/graph_nodes");
        std::filesystem::create_directory(outputPath+"/global/graph_edges");
        // std::filesystem::create_directory(outputPath+"/global/err");
        std::filesystem::create_directory(outputPath+"/local");
        std::filesystem::create_directory(outputPath+"/local/p");
        std::filesystem::create_directory(outputPath+"/local/d");
        std::filesystem::create_directory(outputPath+"/local/t");
        std::filesystem::create_directory(outputPath+"/local/h");
        std::filesystem::create_directory(outputPath+"/local/pts");
        std::filesystem::create_directory(outputPath+"/match");
        std::filesystem::create_directory(outputPath+"/finalAssoc");
        std::filesystem::create_directory(outputPath+"/global_obs");
        std::filesystem::create_directory(outputPath+"/local_obs");
        std::cout << "   Done!" << std::endl;
    }
    return 0;
}