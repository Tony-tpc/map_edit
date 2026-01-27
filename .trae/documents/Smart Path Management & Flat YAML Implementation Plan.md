# Refined Plan: Smart Path Management & Flat YAML Structure

I have analyzed the codebase and prepared the following plan to implement your requirements.

## 1. Project Dependencies
*   **Action**: Add `yaml-cpp` dependency since the project currently only uses `jsoncpp`.
*   **Files**:
    *   `package.xml`: Add `<depend>yaml-cpp</depend>`.
    *   `CMakeLists.txt`: Add `find_package(yaml-cpp REQUIRED)` and link it to the target.

## 2. Smart Path Management (Global Improvement)
*   **Goal**: Eliminate hardcoded paths and remember the last used location.
*   **Files**:
    *   `include/map_edit/map_edit_panel.h`
    *   `src/map_edit_panel.cpp`
*   **Implementation**:
    *   Add member `QString last_opened_path_;` to `MapEditPanel`.
    *   Initialize `last_opened_path_ = QDir::currentPath();` in the constructor.
    *   Update **ALL** file operations (`openMap`, `saveAllFiles`, `openRegionYaml`, `saveRegionYaml`):
        *   Use `last_opened_path_` in `QFileDialog` calls.
        *   Update `last_opened_path_` to the selected directory upon success.

## 3. Region Data Structure
*   **Goal**: Support custom key-value attributes.
*   **File**: `include/map_edit/region_tool.h`
*   **Implementation**:
    *   Add `std::map<std::string, std::string> attributes;` to the `Region` struct.

## 4. Flat YAML Implementation
*   **Goal**: Replace JSON with a flat YAML structure for regions.
*   **File**: `src/map_edit_panel.cpp`
*   **Implementation**:
    *   **Saving (`saveRegionYaml`)**:
        *   Rename `saveJSONFiles` to `saveRegionYaml`.
        *   Use `yaml-cpp` to write standard fields (`id`, `type`, `points`).
        *   Iterate `attributes` map and write fields at the same level as standard fields (Flat Structure).
    *   **Loading (`openRegionYaml`)**:
        *   Rename `openRegion` to `openRegionYaml`.
        *   Use `yaml-cpp` to parse the file.
        *   Standard fields go to struct members; unknown keys go to `attributes` map.

## 5. UI Refactoring
*   **Goal**: Support editing custom attributes and rename UI elements.
*   **File**: `src/map_edit_panel.cpp`
*   **Implementation**:
    *   **Edit Dialog**:
        *   In `refreshRegionListUI` (where the dialog is created), add a `QTableWidget` with 2 columns (Key, Value).
        *   Add "Add Attribute" and "Remove Attribute" buttons.
        *   Load existing `attributes` into the table when opening the dialog.
        *   Save table contents back to `attributes` map upon confirmation.
    *   **General**:
        *   Rename `save_json_btn_` to `save_yaml_btn_` and update label to "保存到yaml文件".
        *   Update file filters in `QFileDialog` to `YAML files (*.yaml)`.
