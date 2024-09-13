#include <libobsensor/ObSensor.hpp>

#include "utils.hpp"

#include <iostream>
#include <functional>

#if 0 // 
static const std::unordered_map<std::string, OBLogSeverity> logLevelMap = {
    { "DEBUG", OBLogSeverity::OB_LOG_SEVERITY_DEBUG }, { "ERROR", OBLogSeverity::OB_LOG_SEVERITY_ERROR }, { "FATAL", OBLogSeverity::OB_LOG_SEVERITY_FATAL },
    { "INFO", OBLogSeverity::OB_LOG_SEVERITY_INFO },   { "OFF", OBLogSeverity::OB_LOG_SEVERITY_OFF },     { "WARN", OBLogSeverity::OB_LOG_SEVERITY_WARN },
    { "debug", OBLogSeverity::OB_LOG_SEVERITY_DEBUG }, { "error", OBLogSeverity::OB_LOG_SEVERITY_ERROR }, { "fatal", OBLogSeverity::OB_LOG_SEVERITY_FATAL },
    { "info", OBLogSeverity::OB_LOG_SEVERITY_INFO },   { "off", OBLogSeverity::OB_LOG_SEVERITY_OFF },     { "warn", OBLogSeverity::OB_LOG_SEVERITY_WARN }
};
#endif

void printUsage();
bool handleCommand(std::vector<std::string> &args, const std::shared_ptr<ob::Context> &context);
int  main() try {
    // Configure the output level of the log by creating a context
    std::shared_ptr<ob::Context> context = std::make_shared<ob::Context>();

    // Configure the log level output to the terminal.
    context->setLoggerToConsole(OB_LOG_SEVERITY_ERROR);

    // Configure the log level and path output to the file.
    context->setLoggerToFile(OB_LOG_SEVERITY_DEBUG, "");

    // Register a log callback, you can get log information in the callback.
    context->setLoggerToCallback(OB_LOG_SEVERITY_DEBUG, [](OBLogSeverity severity, const char *logMsg) {
        std::cout << "[CallbackMessage][Level:" << severity << "]" << logMsg;
    });

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config>   config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);
    config->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_RGB);

    // Create a pipeline with default device to manage stream
    std::shared_ptr<ob::Pipeline> pipe   = std::make_shared<ob::Pipeline>();

    // Start the pipeline with config
    pipe->start(config);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Stop the Pipeline, no frame data will be generated
    pipe->stop();

	context->setLoggerToCallback(OB_LOG_SEVERITY_OFF, nullptr);
	std::cout << "\nThe sample has ended normally. Press any key to exit.";
    ob_smpl::waitForKeyPressed();
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    std::cout << "\nPress any key to exit.";
    ob_smpl::waitForKeyPressed();
    exit(EXIT_FAILURE);
}


#if 0
void printUsage() {
    std::cout << "-----------------------------------------------------------------------------" << std::endl;
    std::cout << "Usage: <output_mode> <log_level> [output_path]" << std::endl;
    std::cout << std::endl;
    std::cout << "<output_mode>:" << std::endl;
    std::cout << "    0 - Output to terminal" << std::endl;
    std::cout << "    1 - Output to file" << std::endl;
    std::cout << "<log_level>:" << std::endl;
    std::cout << "    DEBUG / debug - Output DEBUG level and above (INFO, WARN, ERROR, FATAL)" << std::endl;
    std::cout << "    INFO  / info  - Output INFO level and above (WARN, ERROR, FATAL)" << std::endl;
    std::cout << "    WARN  / warn  - Output WARN level and above (ERROR, FATAL)" << std::endl;
    std::cout << "    ERROR / error - Output ERROR level and above (FATAL)" << std::endl;
    std::cout << "    FATAL / fatal - Output FATAL level only" << std::endl;
    std::cout << "    OFF   / off   - Disable logging" << std::endl;
    std::cout << "[output_path]:" << std::endl;
    std::cout << "    (Optional) Path to the log file. Only used when output_mode is set to 1." << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "    0 DEBUG" << std::endl;
    std::cout << "    (Outputs DEBUG level log to the terminal)" << std::endl;
    std::cout << "    1 ERROR ./path/Log/" << std::endl;
    std::cout << "    (Outputs ERROR level log to ./path/Log/OrbbecSDK.log)" << std::endl << std::endl;
    std::cout << "    help / ? - print usage" << std::endl;
    std::cout << "    quit / q - quit application" << std::endl;
    std::cout << "-----------------------------------------------------------------------------" << std::endl;
}

bool handleCommand(std::vector<std::string> &args, const std::shared_ptr<ob::Context> &context) {
    // Error handling
    if(args.empty()) {
        std::cout << "Invalid command. Please input \"help\" or \"h\" for usage instructions." << std::endl;
        return true;  // Continue the loop
    }

    if(args.size() == 1) {
        if(args[0] == "help" || args[0] == "h") {
            printUsage();
        }
        else if(args[0] == "quit" || args[0] == "q") {
            return false;  // Exit the loop
        }
        else {
            std::cout << "Invalid command. Please input \"help\" or \"h\" for usage instructions." << std::endl;
        }
        return true;  // Continue the loop
    }

    if(args.size() < 2) {
        std::cout << "Missing required arguments. Please input \"help\" or \"h\" for usage instructions." << std::endl;
        return true;  // Continue the loop
    }

    // Parse output mode
    if(args[0] != "0" && args[0] != "1") {
        std::cout << "Invalid output_mode. Must be 0 (terminal) or 1 (file)." << std::endl;
        return true;  // Continue the loop
    }

    // Parse log level
    auto iter = logLevelMap.find(args[1]);
    if(iter == logLevelMap.end()) {
        std::cout << "Invalid log_level. Must be DEBUG, INFO, WARN, ERROR, FATAL, or OFF." << std::endl;
        return true;  // Continue the loop
    }

    // Handle output_path if provided
    std::string outputPath = (args.size() == 3 ? args[2] : "");

    if(args[0] == "0" && !outputPath.empty()) {
        std::cerr << "Invalid command. output_path only used when output_mode is set to 1." << std::endl;
        return true;  // Continue the loop
    }

    // Set logging configuration
    if(args[0] == "1") {
        context->setLoggerToFile(iter->second, outputPath.c_str());
        std::cout << "File log_level configuration successful, current level:[" << iter->first << "]" << std::endl;
    }
    else {
        context->setLoggerToConsole(iter->second);
        std::cout << "Terminal log_level configuration successful, current level:[" << iter->first << "]" << std::endl;
    }
    return true;  // Continue the loop
}
#endif
