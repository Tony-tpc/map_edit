#include "map_edit/tool_manager.h"

namespace map_edit
{

ToolManager& ToolManager::getInstance()
{
    static ToolManager instance;
    return instance;
}

void ToolManager::registerMapEraserTool(MapEraserTool* tool)
{
    map_eraser_tool_ = tool;
}

MapEraserTool* ToolManager::getMapEraserTool() const
{
    return map_eraser_tool_;
}

} // end namespace map_edit 