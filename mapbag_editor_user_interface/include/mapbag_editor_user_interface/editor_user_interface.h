#ifndef MAPBAG_EDITOR_USER_INTERFACE_EDITOR_USER_INTERFACE_H
#define MAPBAG_EDITOR_USER_INTERFACE_EDITOR_USER_INTERFACE_H

#include <hector_rviz_overlay/displays/qml_overlay_display.h>
#include <hector_world_heightmap/io.h>

namespace mapbag_editor_user_interface
{

class EditorUserInterface : public hector_rviz_overlay::QmlOverlayDisplay
{
Q_OBJECT

protected:
  QString getPathToQml() override;
  
};
} // namespace mapbag_editor_user_interface


#endif //MAPBAG_EDITOR_USER_INTERFACE_EDITOR_USER_INTERFACE_H