import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.3
import Hector.Controls 1.0
import Hector.Icons 1.0
import Hector.InternalControls 1.0
import Hector.Utils 1.0
import Hector.Style 1.0
import QtQuick.Controls.Material 2.2
import Ros 1.0

Item {
    id: editorUserInterface
    anchors.fill: parent

    property var style: Style.activeStyle
    property bool editor_visible: false
    property bool pri_editor_visible: true
    property bool wrong_info_visible: false

    property var polygonpointTool: {
        if ( !rviz || !rviz.toolManager || !rviz.toolManager.tools ) return null
        for ( let tool of rviz.toolManager.tools ) {
            if ( tool.classId === "mapbag_editor_user_interface/PolygonTool" ) return tool
        }
        return null
    }

    property var polygonpoints: {
        if (!polygonpointTool) return []
        return polygonpointTool.tool.polygonpoints
    }

    property bool outsideClicked

    TimeRectangle {
        id: timeRectangle
    }

    FullscreenButton {
        id: fullscreenButton
    }

    RvizToolBar
    {
        id: toolBar
        anchors.bottom: basicController.top
        anchors.bottomMargin: Units.pt(3)
        anchors.right: parent.right
        anchors.rightMargin: Units.pt(150)
        height: Units.pt(20)
        iconMargin: Units.pt(5)
        buttonRadius: Units.pt(4)
        shortcutFont: Qt.font({pointSize: 8})
        editable: false
    }

    Image {
        id: hectorLogo
        anchors.right: parent.right
        anchors.rightMargin: Units.pt(4)
        anchors.bottom: toolBar.bottom
        // anchors.bottom: basicController.top
        source: "../media/logo.png"
        height: sourceSize.height * scale
        width: sourceSize.width * scale
        visible: true
        property real scale: 0.22
    }

    BasicController {
        id: basicController
        z: 2
        anchors.right: parent.right
        anchors.rightMargin: -Units.pt(2)
        anchors.bottom: parent.bottom
        anchors.bottomMargin: -Units.pt(2)
        polygonpointTool: editorUserInterface.polygonpointTool

        onEditorMode: { editor_visible = true }
        onPolygonMode: { 
            controlPanel.curr_index = 1 
            pri_editor_visible = true 
            wrong_info_visible = false
        }
        onPri_editorMode: { 
            controlPanel.curr_index = 0 
            pri_editor_visible = true    
        }
        onWrongTransfer: {
            wrong_info_visible = true
        }
    }

    ControlPanel {
        id: controlPanel
        anchors.left: parent.left
        anchors.right: basicController.left
        anchors.bottom: parent.bottom
        anchors.margins: -Units.pt(2)
        polygonpointTool: editorUserInterface.polygonpointTool
        height: Units.pt(120)
        style: Style.activeStyle
        visible: pri_editor_visible
        wrongInfo_visible: wrong_info_visible
        onSave: {
            menu.simulateSaveClick()
        }
        onClear: {
            menu.simulateClearClick()
        }
    }

    EditorMenu {
        id: menu
        width: Units.pt(183)
        height: Units.pt(24)
        anchors.left: parent.left
        anchors.leftMargin: -Units.pt(2)
        anchors.top: parent.top
        anchors.topMargin: -Units.pt(2)
        polygonpointTool: editorUserInterface.polygonpointTool
        onEdit_nichtvisible: { 
            pri_editor_visible = false 
        }
        onEditvisible: { 
            pri_editor_visible = true
        }
        onClearmap: { 
            basicController.visible_bc = false 
            basicController.height = Units.pt(205)
        }
    }
}
