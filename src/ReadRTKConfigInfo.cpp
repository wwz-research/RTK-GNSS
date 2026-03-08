#include "SPP.h"

bool ReadRTKConfigInfo(const char FName[], ROVERCFGINFO& ANInfo)
{
    FILE* Frin;
    char Line[512];  // 增大缓冲区，适配长文件路径

    // 打开配置文件
    if ((Frin = fopen(FName, "rt")) == NULL) {
        std::cout << "ERROR: Can not Open configuration file: " << FName << std::endl;
        return false;
    }

    // 标记12项配置是否已解析（用于完整性检查）
    bool parsed[12] = { false };

    // 逐行读取配置文件
    while (fgets(Line, sizeof(Line), Frin) != NULL) {
        // 跳过空行/纯空白行
        if (Line[0] == '\n' || Line[0] == '\r' || Line[0] == ' ' || Line[0] == '\t') {
            continue;
        }
        // 跳过EOF标记行
        if (strstr(Line, "EOF") != NULL) {
            continue;
        }

        // ========== 1. 解析 "FROM FILE OR COM:" ==========
        if (strstr(Line, "FROM FILE OR COM:") != NULL) {
            if (sscanf(Line, "FROM FILE OR COM: %hd %*[!\n]", &ANInfo.IsFileData) == 1) {
                parsed[0] = true;
            }
            continue;
        }

        // ========== 2. 解析 "RTK PROCESSING MODE:" ==========
        if (strstr(Line, "RTK PROCESSING MODE:") != NULL) {
            if (sscanf(Line, "RTK PROCESSING MODE: %hd %*[!\n]", &ANInfo.RTKProMode) == 1) {
                parsed[1] = true;
            }
            continue;
        }

        // ========== 3. 解析 "ROVER IP ADDRESS AND PORT:" ==========
        if (strstr(Line, "ROVER IP ADDRESS AND PORT:") != NULL) {
            if (sscanf(Line, "ROVER IP ADDRESS AND PORT: %19s %hd %*[!\n]", ANInfo.RovNetIP, &ANInfo.RovNetPort) == 2) {
                parsed[2] = true;
            }
            continue;
        }

        // ========== 4. 解析 "BASE IP ADDRESS AND PORT:" ==========
        if (strstr(Line, "BASE IP ADDRESS AND PORT:") != NULL) {
            if (sscanf(Line, "BASE IP ADDRESS AND PORT: %19s %hd %*[!\n]", ANInfo.BasNetIP, &ANInfo.BasNetPort) == 2) {
                parsed[3] = true;
            }
            continue;
        }

        // ========== 5. 解析 "ROVER COM SETUP:" ==========
        if (strstr(Line, "ROVER COM SETUP:") != NULL) {
            if (sscanf(Line, "ROVER COM SETUP: %d %d %*[!\n]", &ANInfo.RovPort, &ANInfo.RovBaud) == 2) {
                parsed[4] = true;
            }
            continue;
        }

        // ========== 6. 解析 "BASE OBSDATA SOURCE FILE:" ==========
        if (strstr(Line, "BASE OBSDATA SOURCE FILE:") != NULL) {
            if (sscanf(Line, "BASE OBSDATA SOURCE FILE: %255[^\!\n]", ANInfo.BasObsDatFile) == 1) {
                // 去除路径首尾可能的空格/换行
                char* p = ANInfo.BasObsDatFile;
                while (*p == ' ' && *p != '\0') p++;  // 跳过开头空格
                char* end = p + strlen(p) - 1;
                while (end > p && (*end == ' ' || *end == '\r' || *end == '\n')) end--;  // 跳过结尾空格/换行
                *(end + 1) = '\0';
                parsed[5] = true;
            }
            continue;
        }

        // ========== 7. 解析 "ROVER OBSDATA SOURCE FILE:" ==========
        if (strstr(Line, "ROVER OBSDATA SOURCE FILE:") != NULL) {
            if (sscanf(Line, "ROVER OBSDATA SOURCE FILE: %255[^\!\n]", ANInfo.RovObsDatFile) == 1) {
                char* p = ANInfo.RovObsDatFile;
                while (*p == ' ' && *p != '\0') p++;
                char* end = p + strlen(p) - 1;
                while (end > p && (*end == ' ' || *end == '\r' || *end == '\n')) end--;
                *(end + 1) = '\0';
                parsed[6] = true;
            }
            continue;
        }

        // ========== 8. 解析 "POSITION RESULT FILE:" ==========
        if (strstr(Line, "POSITION RESULT FILE:") != NULL) {
            if (sscanf(Line, "POSITION RESULT FILE: %255[^\!\n]", ANInfo.ResFile) == 1) {
                char* p = ANInfo.ResFile;
                while (*p == ' ' && *p != '\0') p++;
                char* end = p + strlen(p) - 1;
                while (end > p && (*end == ' ' || *end == '\r' || *end == '\n')) end--;
                *(end + 1) = '\0';
                parsed[7] = true;
            }
            continue;
        }

        // ========== 9. 解析 "POSITION DIFF FILE:" ==========
        if (strstr(Line, "POSITION DIFF FILE:") != NULL) {
            if (sscanf(Line, "POSITION DIFF FILE: %255[^\!\n]", ANInfo.DiffFile) == 1) {
                char* p = ANInfo.DiffFile;
                while (*p == ' ' && *p != '\0') p++;
                char* end = p + strlen(p) - 1;
                while (end > p && (*end == ' ' || *end == '\r' || *end == '\n')) end--;
                *(end + 1) = '\0';
                parsed[8] = true;
            }
            continue;
        }

        // ========== 10. 解析 "CODE AND CARRIER PHASE NOISE:" ==========
        if (strstr(Line, "CODE AND CARRIER PHASE NOISE:") != NULL) {
            if (sscanf(Line, "CODE AND CARRIER PHASE NOISE: %lf %lf %*[!\n]", &ANInfo.CodeNoise, &ANInfo.CPNoise) == 2) {
                parsed[9] = true;
            }
            continue;
        }

        // ========== 11. 解析 "THRESHOLD FOR ELEVATION MASK:" ==========
        if (strstr(Line, "THRESHOLD FOR ELEVATION MASK:") != NULL) {
            if (sscanf(Line, "THRESHOLD FOR ELEVATION MASK: %lf %*[!\n]", &ANInfo.ElevThreshold) == 1) {
                parsed[10] = true;
            }
            continue;
        }

        // ========== 12. 解析 "RATIO FOR DD FIXED SOLUTION:" ==========
        if (strstr(Line, "RATIO FOR DD FIXED SOLUTION:") != NULL) {
            if (sscanf(Line, "RATIO FOR DD FIXED SOLUTION: %lf %*[!\n]", &ANInfo.RatioThres) == 1) {
                parsed[11] = true;
            }
            continue;
        }
    }

    // 关闭文件
    fclose(Frin);

    // 检查12项配置是否全部解析成功
    for (int i = 0; i < 12; i++) {
        if (!parsed[i]) {
            std::cout << "ERROR: Missing configuration item (index " << i << ")" << std::endl;
            return false;
        }
    }

    // ========== 使用cout输出解析到的配置参数 ==========
    std::cout << "\n===== RTK Configuration Information =====\n";
    std::cout << "1. FROM FILE OR COM:             " << ANInfo.IsFileData << std::endl;
    std::cout << "2. RTK PROCESSING MODE:          " << ANInfo.RTKProMode << std::endl;
    std::cout << "3. ROVER IP ADDRESS AND PORT:    " << ANInfo.RovNetIP << " : " << ANInfo.RovNetPort << std::endl;
    std::cout << "4. BASE IP ADDRESS AND PORT:     " << ANInfo.BasNetIP << " : " << ANInfo.BasNetPort << std::endl;
    std::cout << "5. ROVER COM SETUP:              Port=" << ANInfo.RovPort << ", Baud=" << ANInfo.RovBaud << std::endl;
    std::cout << "6. BASE OBSDATA SOURCE FILE:     " << ANInfo.BasObsDatFile << std::endl;
    std::cout << "7. ROVER OBSDATA SOURCE FILE:    " << ANInfo.RovObsDatFile << std::endl;
    std::cout << "8. POSITION RESULT FILE:         " << ANInfo.ResFile << std::endl;
    std::cout << "9. POSITION DIFF FILE:           " << ANInfo.DiffFile << std::endl;

    // 设置浮点数精度为4位小数
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "10. CODE AND CARRIER PHASE NOISE: Code=" << ANInfo.CodeNoise << ", CP=" << ANInfo.CPNoise << std::endl;
    std::cout << "11. THRESHOLD FOR ELEVATION MASK: " << ANInfo.ElevThreshold << std::endl;
    std::cout << "12. RATIO FOR DD FIXED SOLUTION:  " << ANInfo.RatioThres << std::endl;
    std::cout << "===========================================\n\n";

    return true;
}