/* mapGenerationTestImplementation.cpp
*
* Author: Birhanu Shimelis Girma, Carnegie Mellon University Africa
* Email: bgirmash@andrew.cmu.edu
* Date: June 05, 2025
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*
* This implementation provides helper functions for testing the actual mapGeneration node.
* It does NOT replicate the map generation functionality, but rather provides utilities
* for launching the node, validating outputs, and managing test data.
*/

#include <mapGenerationTest/mapGenerationTestInterface.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <cstring>
#include <vector>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
#include <chrono>

std::string node_name;
extern std::ofstream logFile; // Reference to the global log file

namespace mapGenerationTest {

// Helper function to check if file exists
bool fileExists(const std::string& filepath) {
    struct stat buffer;
    return (stat(filepath.c_str(), &buffer) == 0);
}

// Helper function to get file size
long getFileSize(const std::string& filepath) {
    struct stat stat_buf;
    int rc = stat(filepath.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

// Count obstacle pixels in a map (for validation)
int countObstaclePixels(const cv::Mat& map) {
    int count = 0;
    for (int i = 0; i < map.rows; i++) {
        for (int j = 0; j < map.cols; j++) {
            if (map.at<uchar>(i, j) == 0) { // 0 = obstacle pixel
                count++;
            }
        }
    }
    return count;
}

// Implementation of FormattedOutputListener methods
std::string FormattedOutputListener::getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm* timeinfo = std::localtime(&time_now);
    std::stringstream ss;
    ss << "[" << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S") << "] ";
    return ss.str();
}

void FormattedOutputListener::OnTestProgramStart(const ::testing::UnitTest& unit_test) {
    std::string message = "mapGenerationTest: =======================================================================";
    std::cout << getCurrentTimeString() << message << std::endl;
    logFile << getCurrentTimeString() << message << std::endl;
    
    auto now = std::chrono::system_clock::now();
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm* timeinfo = std::localtime(&time_now);
    
    std::stringstream ss;
    ss << "mapGenerationTest: === New Map Generation Test Run Started at ";
    ss << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S") << " ===";
    
    std::cout << getCurrentTimeString() << ss.str() << std::endl;
    logFile << getCurrentTimeString() << ss.str() << std::endl;
    
    std::cout << getCurrentTimeString() << "mapGenerationTest: =======================================================================" << std::endl;
    logFile << getCurrentTimeString() << "mapGenerationTest: =======================================================================" << std::endl;
    
    std::cout << getCurrentTimeString() << "mapGenerationTest: Testing Actual Map Generation Node: STARTED" << std::endl;
    logFile << getCurrentTimeString() << "mapGenerationTest: Testing Actual Map Generation Node: STARTED" << std::endl;
    
    std::cout << getCurrentTimeString() << "mapGenerationTest: -----------------------------------------------------------------------" << std::endl;
    logFile << getCurrentTimeString() << "mapGenerationTest: -----------------------------------------------------------------------" << std::endl;
}

void FormattedOutputListener::OnTestStart(const ::testing::TestInfo& test_info) {
    std::string message = "mapGenerationTest: Test Case: " + std::string(test_info.test_case_name()) + "." + std::string(test_info.name());
    std::cout << getCurrentTimeString() << message << std::endl;
    logFile << getCurrentTimeString() << message << std::endl;
}

void FormattedOutputListener::OnTestEnd(const ::testing::TestInfo& test_info) {
    std::cout << getCurrentTimeString() << "mapGenerationTest: -----------------------------------------------------------------------" << std::endl;
    logFile << getCurrentTimeString() << "mapGenerationTest: -----------------------------------------------------------------------" << std::endl;
}

void FormattedOutputListener::OnTestProgramEnd(const ::testing::UnitTest& unit_test) {
    std::cout << getCurrentTimeString() << "mapGenerationTest: =======================================================================" << std::endl;
    logFile << getCurrentTimeString() << "mapGenerationTest: =======================================================================" << std::endl;
    
    auto now = std::chrono::system_clock::now();
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm* timeinfo = std::localtime(&time_now);
    
    std::stringstream ss;
    ss << "mapGenerationTest: === Map Generation Test Run (CAD Mode) Completed at ";
    ss << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S") << " ===";
    
    std::cout << getCurrentTimeString() << ss.str() << std::endl;
    logFile << getCurrentTimeString() << ss.str() << std::endl;
    
    if (unit_test.Passed()) {
        std::cout << getCurrentTimeString() << "mapGenerationTest: === All tests PASSED ===" << std::endl;
        logFile << getCurrentTimeString() << "mapGenerationTest: === All tests PASSED ===" << std::endl;
    } else {
        std::cout << getCurrentTimeString() << "mapGenerationTest: === Some tests FAILED ===" << std::endl;
        logFile << getCurrentTimeString() << "mapGenerationTest: === Some tests FAILED ===" << std::endl;
    }
    
    std::cout << getCurrentTimeString() << "mapGenerationTest: =======================================================================" << std::endl;
    logFile << getCurrentTimeString() << "mapGenerationTest: =======================================================================" << std::endl;
}

// Logging functions implementation
void LogTestStep(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm* timeinfo = std::localtime(&time_now);
    
    std::stringstream ss;
    ss << "[" << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S") << "] mapGenerationTest: " << message;
    
    std::cout << ss.str() << std::endl;
    logFile << ss.str() << std::endl;
}

void LogTestSuccess(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm* timeinfo = std::localtime(&time_now);
    
    std::stringstream ss;
    ss << "[" << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S") << "] mapGenerationTest: [SUCCESS] " << message;
    
    std::cout << ss.str() << std::endl;
    logFile << ss.str() << std::endl;
}

void LogTestPassed(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm* timeinfo = std::localtime(&time_now);
    
    std::stringstream ss;
    ss << "[" << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S") << "] mapGenerationTest: " << message << ": PASSED";
    
    std::cout << ss.str() << std::endl;
    logFile << ss.str() << std::endl;
}

class MapGenerationTest : public ::testing::Test {
protected:
    std::string packagePath;
    std::string testDataDir;
    std::string mapGenDataDir;
    std::string configDir;
    std::vector<float> robotRadii;
    
    virtual void SetUp() {
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME);
        testDataDir = packagePath + "/mapGenerationTest/data";
        configDir = packagePath + "/mapGenerationTest/config";
        
        // Get the mapGeneration node's data directory
        std::string mapGenPackagePath = ros::package::getPath("cssr_system");
        mapGenDataDir = mapGenPackagePath + "/mapGeneration/data";
        
        // Create test output directory if it doesn't exist
        std::string testOutputDir = testDataDir + "/testOutput";
        int result = system(("mkdir -p " + testOutputDir).c_str());
        (void)result; // Suppress warning
        
        // Load robot radii from file
        LoadRobotRadii();
        
        // Clean up any previous outputs
        CleanupPreviousOutputs();
    }
    
    void LoadRobotRadii() {
        std::string radiusFilePath = testDataDir + "/testRobotRadius.dat";
        std::ifstream radiusFile(radiusFilePath);
        
        if (!radiusFile.good()) {
            LogTestStep("Warning: Could not open robot radius file: " + radiusFilePath);
            LogTestStep("Using default robot radius values");
            robotRadii = {0.1, 0.2, 0.3, 0.5, 0.8}; // Default values
            return;
        }
        
        std::string label;
        float radius;
        
        while (radiusFile >> label >> radius) {
            robotRadii.push_back(radius);
            LogTestStep("Loaded " + label + ": " + std::to_string(radius) + "m");
        }
        
        radiusFile.close();
        
        if (robotRadii.empty()) {
            LogTestStep("Warning: No robot radii loaded, using default values");
            robotRadii = {0.1, 0.2, 0.3, 0.5, 0.8}; // Default values
        }
    }
    
    void CleanupPreviousOutputs() {
        // Clean up previous outputs from mapGeneration node
        int result = system(("rm -f " + mapGenDataDir + "/environmentMap.png").c_str());
        (void)result; // Suppress warning
        result = system(("rm -f " + mapGenDataDir + "/configurationSpaceMap.png").c_str());
        (void)result; // Suppress warning
    }
    
    // Test the actual mapGeneration node
    bool TestMapGenerationNode(const std::string& inputFileName, 
                                const std::string& configFileName,
                                float expectedRobotRadius) {
        
        // Copy test input file to mapGeneration data directory
        std::string srcInput = testDataDir + "/" + inputFileName;
        std::string dstInput = mapGenDataDir + "/mapGenerationInput.dat";
        
        std::string copyCmd = "cp " + srcInput + " " + dstInput;
        if (system(copyCmd.c_str()) != 0) {
            LogTestStep("Failed to copy input file: " + srcInput);
            return false;
        }
        
        // Copy test config file to mapGeneration config directory  
        std::string srcConfig = configDir + "/" + configFileName;
        std::string dstConfig = ros::package::getPath("cssr_system") + "/mapGeneration/config/mapGenerationConfiguration.ini";
        
        copyCmd = "cp " + srcConfig + " " + dstConfig;
        if (system(copyCmd.c_str()) != 0) {
            LogTestStep("Failed to copy config file: " + srcConfig);
            return false;
        }
        
        // Copy test obstacles file if it exists
        std::string srcObstacles = testDataDir + "/testObstacles.dat";
        std::string dstObstacles = mapGenDataDir + "/testObstacles.dat";
        copyCmd = "cp " + srcObstacles + " " + dstObstacles + " 2>/dev/null";
        int result = system(copyCmd.c_str()); // Don't fail if this doesn't exist
        (void)result; // Suppress warning
        
        LogTestStep("Launching actual mapGeneration node...");
        
        // Launch the actual mapGeneration node
        std::string command = "cd " + ros::package::getPath("cssr_system") + 
                                " && timeout 30s rosrun cssr_system mapGeneration > /tmp/mapgen_output.log 2>&1";
        
        result = system(command.c_str());
        
        // Check if the command completed successfully (exit code 0 or timeout code)
        if (result != 0 && result != 31744) { // 31744 is timeout exit code
            LogTestStep("mapGeneration node failed with exit code: " + std::to_string(result));
            return false;
        }
        
        LogTestSuccess("mapGeneration node completed execution");
        
        // Wait a moment for file system to sync
        usleep(500000); // 0.5 seconds
        
        // Verify output files were created
        std::string envMapPath = mapGenDataDir + "/environmentMap.png";
        std::string configMapPath = mapGenDataDir + "/configurationSpaceMap.png";
        
        bool envMapExists = fileExists(envMapPath);
        bool configMapExists = fileExists(configMapPath);
        
        if (!envMapExists) {
            LogTestStep("Environment map not created: " + envMapPath);
            return false;
        }
        
        if (!configMapExists) {
            LogTestStep("Configuration space map not created: " + configMapPath);
            return false;
        }
        
        LogTestSuccess("Both output maps were created successfully");
        
        // Copy outputs to test directory for inspection
        std::string testOutputDir = testDataDir + "/testOutput";
        std::string testPrefix = "test_" + std::to_string(expectedRobotRadius) + "_";
        
        copyCmd = "cp " + envMapPath + " " + testOutputDir + "/" + testPrefix + "environmentMap.png";
        result = system(copyCmd.c_str());
        (void)result; // Suppress warning
        
        copyCmd = "cp " + configMapPath + " " + testOutputDir + "/" + testPrefix + "configurationSpaceMap.png";
        result = system(copyCmd.c_str());
        (void)result; // Suppress warning
        
        return ValidateOutputMaps(envMapPath, configMapPath, expectedRobotRadius);
    }
    
    bool ValidateOutputMaps(const std::string& envMapPath, 
                            const std::string& configMapPath,
                            float robotRadius) {
        
        // Load the generated maps
        cv::Mat envMap = cv::imread(envMapPath, cv::IMREAD_GRAYSCALE);
        cv::Mat configMap = cv::imread(configMapPath, cv::IMREAD_GRAYSCALE);
        
        if (envMap.empty()) {
            LogTestStep("Failed to load environment map: " + envMapPath);
            return false;
        }
        
        if (configMap.empty()) {
            LogTestStep("Failed to load configuration space map: " + configMapPath);
            return false;
        }
        
        LogTestStep("Validating map dimensions...");
        
        // Basic validation - maps should have same dimensions
        if (envMap.size() != configMap.size()) {
            LogTestStep("Map dimensions don't match!");
            return false;
        }
        
        LogTestStep("Maps have correct dimensions: " + 
                    std::to_string(envMap.cols) + "x" + std::to_string(envMap.rows));
        
        // Configuration space should have more obstacles than environment map
        int envObstacles = countObstaclePixels(envMap);
        int configObstacles = countObstaclePixels(configMap);
        
        LogTestStep("Environment map obstacles: " + std::to_string(envObstacles));
        LogTestStep("Configuration space obstacles: " + std::to_string(configObstacles));
        
        if (configObstacles <= envObstacles) {
            LogTestStep("Configuration space should have more obstacles than environment map!");
            return false;
        }
        
        LogTestSuccess("Configuration space correctly has more obstacles than environment map");
        
        return true;
    }
    
    bool fileExists(const std::string& filepath) {
        std::ifstream file(filepath);
        return file.good();
    }

protected:
    void CreateConfigFile(const std::string& filepath, float robotRadius) {
        std::ofstream configFile(filepath);
        configFile << "mode CAD\n";
        configFile << "verboseMode false\n";
        configFile << "resolution 0.01\n";
        configFile << "robotRadius " << robotRadius << "\n";
        configFile << "inputFile mapGenerationInput.dat\n";
        configFile.close();
    }
    
    void CreateVerboseConfigFile(const std::string& filepath, bool verbose) {
        std::ofstream configFile(filepath);
        configFile << "mode CAD\n";
        configFile << "verboseMode " << (verbose ? "true" : "false") << "\n";
        configFile << "resolution 0.01\n";
        configFile << "robotRadius 0.3\n";
        configFile << "inputFile mapGenerationInput.dat\n";
        configFile.close();
    }
};

// Test basic map creation
TEST_F(MapGenerationTest, BasicMapGeneration) {
    LogTestStep("Testing basic map generation with actual mapGeneration node...");
    
    bool result = TestMapGenerationNode("mapGenerationInput.dat", 
                                    "mapGenerationTestConfiguration.ini",
                                    0.3);
    
    EXPECT_TRUE(result) << "Basic map generation should succeed";
    LogTestPassed("Basic map generation test");
}

// Test different robot radii by creating different config files
TEST_F(MapGenerationTest, DifferentRobotRadii) {
    LogTestStep("Testing different robot radii configurations...");
    
    // Test with different robot radii
    std::vector<float> testRadii = {0.1, 0.3, 0.5};
    std::vector<int> obstacleCounts;
    
    for (float radius : testRadii) {
        LogTestStep("Testing with robot radius: " + std::to_string(radius) + "m");
        
        // Create a temporary config file with this radius
        std::string tempConfigPath = configDir + "/tempConfig_" + std::to_string(radius) + ".ini";
        CreateConfigFile(tempConfigPath, radius);
        
        std::string tempConfigName = "tempConfig_" + std::to_string(radius) + ".ini";
        
        bool result = TestMapGenerationNode("mapGenerationInput.dat", tempConfigName, radius);
        EXPECT_TRUE(result) << "Map generation should succeed for radius " << radius;
        
        if (result) {
            // Count obstacles in the generated configuration space
            std::string configMapPath = mapGenDataDir + "/configurationSpaceMap.png";
            cv::Mat configMap = cv::imread(configMapPath, cv::IMREAD_GRAYSCALE);
            
            if (!configMap.empty()) {
                int obstacleCount = countObstaclePixels(configMap);
                obstacleCounts.push_back(obstacleCount);
                LogTestStep("Robot radius " + std::to_string(radius) + "m: " + 
                        std::to_string(obstacleCount) + " obstacle pixels");
            }
        }
        
        // Clean up temp config
        int result_cleanup = system(("rm -f " + tempConfigPath).c_str());
        (void)result_cleanup; // Suppress warning
    }
    
    // Verify that larger radius means more obstacles
    for (size_t i = 1; i < obstacleCounts.size(); i++) {
        EXPECT_GT(obstacleCounts[i], obstacleCounts[i-1]) 
            << "Larger robot radius should result in more obstacles";
    }
    
    LogTestPassed("Different robot radii test");
}

// Test verbose mode configuration
TEST_F(MapGenerationTest, VerboseModeTest) {
    LogTestStep("Testing verbose mode configuration...");
    
    // Create config with verboseMode = true
    std::string verboseConfigPath = configDir + "/verboseConfig.ini";
    CreateVerboseConfigFile(verboseConfigPath, true);
    
    bool result = TestMapGenerationNode("mapGenerationInput.dat", "verboseConfig.ini", 0.3);
    EXPECT_TRUE(result) << "Map generation should succeed in verbose mode";
    
    // Clean up
    int result_cleanup = system(("rm -f " + verboseConfigPath).c_str());
    (void)result_cleanup; // Suppress warning
    
    LogTestPassed("Verbose mode configuration test");
}

// Validate that a map has expected properties
bool validateMapProperties(const std::string& mapPath, int expectedWidth, int expectedHeight) {
    if (!fileExists(mapPath)) {
        std::cerr << "Map file does not exist: " << mapPath << std::endl;
        return false;
    }
    
    cv::Mat map = cv::imread(mapPath, cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
        std::cerr << "Failed to load map: " << mapPath << std::endl;
        return false;
    }
    
    if (map.cols != expectedWidth || map.rows != expectedHeight) {
        std::cerr << "Map dimensions don't match. Expected: " << expectedWidth << "x" << expectedHeight
                  << ", Got: " << map.cols << "x" << map.rows << std::endl;
        return false;
    }
    
    // Check that the map has both obstacles (0) and free space (255)
    bool hasObstacles = false;
    bool hasFreeSpace = false;
    
    for (int i = 0; i < map.rows && (!hasObstacles || !hasFreeSpace); i++) {
        for (int j = 0; j < map.cols && (!hasObstacles || !hasFreeSpace); j++) {
            uchar pixel = map.at<uchar>(i, j);
            if (pixel == 0) hasObstacles = true;
            if (pixel == 255) hasFreeSpace = true;
        }
    }
    
    if (!hasObstacles) {
        std::cerr << "Map has no obstacles" << std::endl;
        return false;
    }
    
    if (!hasFreeSpace) {
        std::cerr << "Map has no free space" << std::endl;
        return false;
    }
    
    return true;
}

// Compare two maps to ensure configuration space has more obstacles
bool compareConfigurationSpaces(const std::string& envMapPath, const std::string& configMapPath) {
    cv::Mat envMap = cv::imread(envMapPath, cv::IMREAD_GRAYSCALE);
    cv::Mat configMap = cv::imread(configMapPath, cv::IMREAD_GRAYSCALE);
    
    if (envMap.empty() || configMap.empty()) {
        std::cerr << "Failed to load maps for comparison" << std::endl;
        return false;
    }
    
    if (envMap.size() != configMap.size()) {
        std::cerr << "Maps have different sizes" << std::endl;
        return false;
    }
    
    int envObstacles = countObstaclePixels(envMap);
    int configObstacles = countObstaclePixels(configMap);
    
    std::cout << "Environment map obstacles: " << envObstacles << std::endl;
    std::cout << "Configuration space obstacles: " << configObstacles << std::endl;
    
    return configObstacles > envObstacles;
}

// Read input file to get expected dimensions (for validation)
bool getExpectedDimensions(const std::string& inputFilePath, int& width, int& height) {
    std::ifstream inputFile(inputFilePath);
    if (!inputFile.good()) {
        std::cerr << "Cannot open input file: " << inputFilePath << std::endl;
        return false;
    }
    
    inputFile >> width >> height;
    inputFile.close();
    
    return (width > 0 && height > 0);
}

// Create a test input file with specific parameters
bool createTestInputFile(const std::string& filepath, 
                        int width, 
                        int height, 
                        const std::string& obstacleFile,
                        const std::string& envMapFile,
                        const std::string& configMapFile) {
    std::ofstream outFile(filepath);
    if (!outFile.good()) {
        std::cerr << "Cannot create test input file: " << filepath << std::endl;
        return false;
    }
    
    outFile << width << " " << height << std::endl;
    outFile << obstacleFile << std::endl;
    outFile << envMapFile << std::endl;
    outFile << configMapFile << std::endl;
    
    outFile.close();
    return true;
}

// Create a test configuration file with specific robot radius
bool createTestConfigFile(const std::string& filepath, 
                         float robotRadius, 
                         bool verboseMode,
                         const std::string& inputFile) {
    std::ofstream outFile(filepath);
    if (!outFile.good()) {
        std::cerr << "Cannot create test config file: " << filepath << std::endl;
        return false;
    }
    
    outFile << "mode CAD" << std::endl;
    outFile << "verboseMode " << (verboseMode ? "true" : "false") << std::endl;
    outFile << "resolution 0.01" << std::endl;
    outFile << "robotRadius " << robotRadius << std::endl;
    outFile << "inputFile " << inputFile << std::endl;
    
    outFile.close();
    return true;
}

// Launch the actual mapGeneration node and wait for completion
bool launchMapGenerationNode(const std::string& packagePath, int timeoutSeconds) {
    std::string command = "cd " + packagePath + "/mapGeneration && timeout " + 
                         std::to_string(timeoutSeconds) + "s rosrun cssr_system mapGeneration > /tmp/mapgen_test_output.log 2>&1";
    
    std::cout << "Launching mapGeneration node..." << std::endl;
    std::cout << "Command: " << command << std::endl;
    
    int result = system(command.c_str());
    
    // Check output log for any errors
    std::ifstream logFile("/tmp/mapgen_test_output.log");
    if (logFile.good()) {
        std::string line;
        bool hasError = false;
        while (std::getline(logFile, line)) {
            if (line.find("ERROR") != std::string::npos || 
                line.find("FATAL") != std::string::npos ||
                line.find("Failed") != std::string::npos) {
                std::cerr << "Node output: " << line << std::endl;
                hasError = true;
            }
        }
        logFile.close();
        
        if (hasError) {
            return false;
        }
    }
    
    // Return codes: 0 = success, 31744 = timeout (which might be OK if maps were generated)
    return (result == 0 || result == 31744);
}

// Verify that the mapGeneration node produced the expected outputs
bool verifyNodeOutputs(const std::string& dataDir, 
                      const std::string& expectedEnvMap,
                      const std::string& expectedConfigMap,
                      int expectedWidth,
                      int expectedHeight) {
    
    std::string envMapPath = dataDir + "/" + expectedEnvMap;
    std::string configMapPath = dataDir + "/" + expectedConfigMap;
    
    std::cout << "Verifying output files..." << std::endl;
    std::cout << "Environment map: " << envMapPath << std::endl;
    std::cout << "Configuration map: " << configMapPath << std::endl;
    
    // Check that files exist
    if (!fileExists(envMapPath)) {
        std::cerr << "Environment map not found: " << envMapPath << std::endl;
        return false;
    }
    
    if (!fileExists(configMapPath)) {
        std::cerr << "Configuration space map not found: " << configMapPath << std::endl;
        return false;
    }
    
    // Validate map properties
    if (!validateMapProperties(envMapPath, expectedWidth, expectedHeight)) {
        std::cerr << "Environment map validation failed" << std::endl;
        return false;
    }
    
    if (!validateMapProperties(configMapPath, expectedWidth, expectedHeight)) {
        std::cerr << "Configuration space map validation failed" << std::endl;
        return false;
    }
    
    // Verify that configuration space has more obstacles
    if (!compareConfigurationSpaces(envMapPath, configMapPath)) {
        std::cerr << "Configuration space comparison failed" << std::endl;
        return false;
    }
    
    std::cout << "Output verification successful!" << std::endl;
    return true;
}

// Clean up test files
void cleanupTestFiles(const std::string& dataDir) {
    // Remove temporary files
    int result = system(("rm -f " + dataDir + "/environmentMap.png").c_str());
    (void)result; // Suppress warning
    result = system(("rm -f " + dataDir + "/configurationSpaceMap.png").c_str());
    (void)result; // Suppress warning
    result = system("rm -f /tmp/mapgen_test_output.log");
    (void)result; // Suppress warning
}

// Copy files for test setup
bool copyFile(const std::string& source, const std::string& destination) {
    std::string command = "cp " + source + " " + destination;
    return system(command.c_str()) == 0;
}

// Setup test environment by copying necessary files
bool setupTestEnvironment(const std::string& testDataDir, 
                         const std::string& mapGenDataDir,
                         const std::string& inputFileName,
                         const std::string& obstacleFileName) {
    
    // Copy input file
    std::string srcInput = testDataDir + "/" + inputFileName;
    std::string dstInput = mapGenDataDir + "/mapGenerationInput.dat";
    
    if (!copyFile(srcInput, dstInput)) {
        std::cerr << "Failed to copy input file: " << srcInput << std::endl;
        return false;
    }
    
    // Copy obstacle file if it exists
    std::string srcObstacles = testDataDir + "/" + obstacleFileName;
    if (fileExists(srcObstacles)) {
        std::string dstObstacles = mapGenDataDir + "/" + obstacleFileName;
        if (!copyFile(srcObstacles, dstObstacles)) {
            std::cerr << "Failed to copy obstacle file: " << srcObstacles << std::endl;
            return false;
        }
    }
    
    return true;
}

// Test the actual mapGeneration node with given parameters
bool testActualMapGenerationNode(const std::string& testName,
                                const std::string& inputFileName,
                                const std::string& configFileName,
                                float expectedRobotRadius,
                                int expectedWidth,
                                int expectedHeight) {
    
    std::cout << "\n=== Testing: " << testName << " ===" << std::endl;
    
    // Get package paths
    std::string testPackagePath = ros::package::getPath(ROS_PACKAGE_NAME);
    std::string mapGenPackagePath = ros::package::getPath("cssr_system");
    
    std::string testDataDir = testPackagePath + "/mapGenerationTest/data";
    std::string testConfigDir = testPackagePath + "/mapGenerationTest/config";
    std::string mapGenDataDir = mapGenPackagePath + "/mapGeneration/data";
    std::string mapGenConfigDir = mapGenPackagePath + "/mapGeneration/config";
    
    // Clean up previous outputs
    cleanupTestFiles(mapGenDataDir);
    
    // Setup test environment
    if (!setupTestEnvironment(testDataDir, mapGenDataDir, inputFileName, "testObstacles.dat")) {
        std::cerr << "Failed to setup test environment" << std::endl;
        return false;
    }
    
    // Copy test configuration
    std::string srcConfig = testConfigDir + "/" + configFileName;
    std::string dstConfig = mapGenConfigDir + "/mapGenerationConfiguration.ini";
    
    if (!copyFile(srcConfig, dstConfig)) {
        std::cerr << "Failed to copy config file: " << srcConfig << std::endl;
        return false;
    }
    
    // Launch the actual mapGeneration node
    if (!launchMapGenerationNode(mapGenPackagePath, 30)) {
        std::cerr << "mapGeneration node failed to execute properly" << std::endl;
        return false;
    }
    
    // Wait for file system sync
    usleep(500000); // 0.5 seconds
    
    // Verify outputs
    bool success = verifyNodeOutputs(mapGenDataDir, 
                                   "environmentMap.png", 
                                   "configurationSpaceMap.png",
                                   expectedWidth, 
                                   expectedHeight);
    
    if (success) {
        std::cout << "=== " << testName << " PASSED ===" << std::endl;
    } else {
        std::cout << "=== " << testName << " FAILED ===" << std::endl;
    }
    
    return success;
}

// Utility functions (keep these simple helpers)
void display_error_and_exit(const char* error_message) {
    std::cerr << "TEST ERROR: " << error_message << std::endl;
    exit(1);
}

void prompt_and_exit(int status) {
    std::cout << "Test completed with status: " << status << std::endl;
    exit(status);
}

void prompt_and_continue() {
    std::cout << "Press Enter to continue...";
    std::cin.ignore();
}

#ifdef ROS
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif

} // namespace mapGenerationTest