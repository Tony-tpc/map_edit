#ifndef TOOL_MANAGER_H
#define TOOL_MANAGER_H

#include "map_eraser_tool.h"

namespace map_edit
{

    class ToolManager
    {
    public:
        static ToolManager &getInstance();

        void registerMapEraserTool(MapEraserTool *tool);

        MapEraserTool *getMapEraserTool() const;

    private:
        ToolManager() : map_eraser_tool_(nullptr) {}
        virtual ~ToolManager() = default;
        ToolManager(const ToolManager &) = delete;
        ToolManager &operator=(const ToolManager &) = delete;

        MapEraserTool *map_eraser_tool_;
    };

} // end namespace map_edit

#endif // TOOL_MANAGER_H