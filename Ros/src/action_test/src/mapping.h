#include <unordered_map>

/**
 * @brief 相机焦点调整动作与实际命令的映射
 * 
 */
static std::unordered_map<uint8_t, uint8_t> focus_map = {
    {1, 0x01},
    {2, 0x08},
    {3, 0x09},
    {4, 0x0e}, //未找到对应指令，用急性转换代替
    {5, 0x0a},
    {6, 0x0b},
    {7, 0x1a},
    {8, 0x19},
};
