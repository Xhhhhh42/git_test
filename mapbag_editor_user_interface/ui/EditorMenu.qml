import QtQuick 2.1
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.3
// import QtQuick.Dialogs 1.3
import Hector.Controls 1.0
import Hector.Icons 1.0
import Hector.InternalControls 1.0
import Hector.Utils 1.0
import Hector.Style 1.0
import QtQuick.Controls.Material 2.2
import Ros 1.0

import Qt.labs.platform 1.1


Rectangle {
    id: root
    property var style_eins: Style.background.container
    property var style_zwei: Style.activeStyle
    property var polygonpointTool

    signal editvisible()
    signal edit_nichtvisible()
    signal clearmap()

    function simulateSaveClick() {
        menueins_zwei.clicked()
    }

    function simulateClearClick() {
        menueins_vier.clicked()
    }

    ColumnLayout{
        Rectangle {
            id: menuBar
            color: style_eins
            width: Units.pt(183)
            height: Units.pt(24)
            border {color: style_zwei.primary.color; width: Units.pt(2)}

            RowLayout  {
                Layout.fillWidth: true
                Layout.preferredHeight: Units.pt(24)
                spacing: 0

                MenuButton {
                    id: menuBar_eins
                    Layout.preferredWidth: Units.pt(60)
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: Units.pt(2)

                    contentItem: Rectangle {
                        anchors.fill: parent
                        color: "transparent"

                        RowLayout {
                            anchors.fill: parent
                            spacing: Units.pt(6)

                            Text {
                                Layout.alignment: Qt.AlignLeft
                                Layout.leftMargin: Units.pt(15)                
                                text: qsTr("Maps")
                                font { pointSize: 13; weight: Font.Bold }
                                color: "black"
                            }
                        }
                    }

                    onClicked: { 
                        if( menuBar_eins.isClicked ) menuBar_eins.isClicked = false
                        else menuBar_eins.isClicked = true
                        menuBar_zwei.isClicked = false
                        menuBar_drei.isClicked = false
                    }
                }

                MenuButton {
                    id: menuBar_zwei
                    Layout.preferredWidth: Units.pt(60)
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: Units.pt(2)

                    contentItem: Rectangle {
                        anchors.fill: parent
                        color: "transparent"

                        RowLayout {
                            anchors.fill: parent
                            spacing: Units.pt(6)

                            Text {
                                Layout.alignment: Qt.AlignLeft
                                Layout.leftMargin: Units.pt(12)                
                                text: qsTr("Editor")
                                font { pointSize: 13; weight: Font.Bold }
                                color: "black"
                            }
                        }
                    }

                    onClicked: { 
                        if( menuBar_zwei.isClicked ) menuBar_zwei.isClicked = false
                        else menuBar_zwei.isClicked = true
                        menuBar_eins.isClicked = false  
                        menuBar_drei.isClicked = false
                    }
                }

                MenuButton {
                    id: menuBar_drei
                    Layout.preferredWidth: Units.pt(60)
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: Units.pt(2)

                    contentItem: Rectangle {
                        anchors.fill: parent
                        color: "transparent"

                        RowLayout {
                            anchors.fill: parent
                            spacing: Units.pt(6)

                            Text {
                                Layout.alignment: Qt.AlignLeft
                                Layout.leftMargin: Units.pt(16)                
                                text: qsTr("Help")
                                font { pointSize: 13; weight: Font.Bold }
                                color: "black"
                            }
                        }
                    }

                    onClicked: { 
                        if( menuBar_drei.isClicked ) menuBar_drei.isClicked = false
                        else menuBar_drei.isClicked = true
                        menuBar_eins.isClicked = false
                        menuBar_zwei.isClicked = false
                    }
                }
            }
        }

        RowLayout{
            Rectangle {
                id: menu_eins
                Layout.alignment: Qt.AlignLeft | Qt.AlignTop 
                Layout.leftMargin: Units.pt(2) 
                Layout.topMargin: -Units.pt(3.5) 
                color: style_eins
                width: Units.pt(105)
                height: Units.pt(96)
                border {color: Material.color(Material.Grey, Material.Shade500); width: Units.pt(0.4)}
                visible: menuBar_eins.isClicked

                ColumnLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(24)
                    spacing: 0

                    MenuButton {
                        id: menueins_eins
                        Layout.preferredWidth: Units.pt(103)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.topMargin: Units.pt(1)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Load Map")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_eins.isClicked = false
                            mapFileDialog.open()
                            // console.log(Ros.package.getPath("mapbag_editor_server"))
                        }
                    }

                    MenuButton {
                        id: menueins_zwei
                        Layout.preferredWidth: Units.pt(103)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Save Map")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_eins.isClicked = false
                            Service.callAsync("/mapbag_editor_server_node/save_map", "hector_std_msgs/StringService", { param: "save" })
                        }
                    }

                    MenuButton {
                        id: menueins_drei
                        Layout.preferredWidth: Units.pt(103)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Save Map As")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_eins.isClicked = false
                            // saveMapFileDialog.open()
                            newfileDialog.open()
                        }
                    }

                    MenuButton {
                        id: menueins_vier
                        Layout.preferredWidth: Units.pt(103)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Clear Maps")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_eins.isClicked = false
                            clearMapbagDialog.open()
                        }
                    }
                }
            }

            Rectangle {
                id: menu_zwei
                Layout.alignment: Qt.AlignLeft | Qt.AlignTop 
                Layout.leftMargin: Units.pt(61) 
                Layout.topMargin: -Units.pt(3.5) 
                color: style_eins
                width: Units.pt(135)
                height: Units.pt(72)
                border {color: Material.color(Material.Grey, Material.Shade500); width: Units.pt(0.4)}
                visible: menuBar_zwei.isClicked

                ColumnLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(24)
                    spacing: 0

                    MenuButton {
                        id: menuzwei_eins
                        Layout.preferredWidth: Units.pt(133)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.topMargin: Units.pt(1)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Save Modifications")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_zwei.isClicked = false
                            polygonpointTool.tool.publishToServer()
                            polygonpointTool.tool.clearPolygonpoints()
                        }
                    }

                    MenuButton {
                        id: menuzwei_zwei
                        Layout.preferredWidth: Units.pt(133)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Editor Visible")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_zwei.isClicked = false
                            root.editvisible()
                        }
                    }

                    MenuButton {
                        id: menu9
                        Layout.preferredWidth: Units.pt(133)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Editor Hide")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_zwei.isClicked = false
                            root.edit_nichtvisible()
                        }
                    }
                }
            }

            Rectangle {
                id: menu_drei
                Layout.alignment: Qt.AlignLeft | Qt.AlignTop 
                Layout.leftMargin: Units.pt(121) 
                Layout.topMargin: -Units.pt(3.5) 
                color: style_eins
                width: Units.pt(155)
                height: Units.pt(25)
                border {color: Material.color(Material.Grey, Material.Shade500); width: Units.pt(0.4)}
                visible: menuBar_drei.isClicked

                ColumnLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(24)
                    spacing: 0

                    MenuButton {
                        id: menudrei_eins
                        Layout.preferredWidth: Units.pt(153)
                        Layout.preferredHeight: Units.pt(23)
                        Layout.topMargin: Units.pt(1)
                        Layout.leftMargin: Units.pt(1)

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(10)                
                                    text: qsTr("Instructions for Plug-in")
                                    font { pointSize: 13 }
                                    color: "black"
                                }
                            }
                        }

                        onClicked: { 
                            menuBar_drei.isClicked = false 
                            var url = "https://git.sim.informatik.tu-darmstadt.de/hector/mapbag_editor" 
                            Qt.openUrlExternally(url) // Open the URL in the default browser
                        }
                    }
                }
            }
        }
    }

    FileDialog {
        id: mapFileDialog
        title: "Please choose a map"
        folder: Ros.package.getPath("mapbag_editor_server")+"/maps"
        // folder: "/home/xhhh/hector/src/mapbag_editor/mapbag_editor_server/maps"
        nameFilters: [ "Map file (*.whm *.mapbag)", "All files (*)" ]
        onAccepted: {
            polygonpointTool.tool.clearmap()
            Service.callAsync("/mapbag_editor_server_node/clearmapbag", "hector_std_msgs/StringService", { param: "" })
            // Service.callAsync("/mapbag_editor_server_node/load_map", "hector_std_msgs/StringService", {param: fileUrl.toString().substr(7)})
            Service.callAsync("/mapbag_editor_server_node/load_map", "hector_std_msgs/StringService", {param: file.toString().substr(7)})
        }
    }

    FileDialog {
        id: newfileDialog
        title: "Save Map as"
        folder: Ros.package.getPath("mapbag_editor_server")+"/maps"
        nameFilters: [ "Map file (*.whm *.mapbag)", "All files (*)" ]
        fileMode: FileDialog.SaveFile
        onAccepted: {
        }
    }

    FileDialog {
        id: saveMapFileDialog
        title: "Save Map"
        folder: Ros.package.getPath("mapbag_editor_server") + "/maps"
        nameFilters: [ "Map file (*.whm *.mapbag)", "All files (*)" ]
        onAccepted: {
            // Service.callAsync("/mapbag_editor_server_node/save_map", "hector_std_msgs/StringService", { param: fileUrl.toString().substr(7) })
            Service.callAsync("/mapbag_editor_server_node/save_map", "hector_std_msgs/StringService", { param: file.toString().substr(7) })
        }
    }

    ConfirmationDialog {
        id: clearMapbagDialog
        polygonpointTool: root.polygonpointTool
        height: Units.pt(120)
        title: "Clear Mapbag?!!"
        onAccepted: {
            polygonpointTool.tool.clearmap()
            Service.callAsync("/mapbag_editor_server_node/clearmapbag", "hector_std_msgs/StringService", { param: "" })
            clearMapbagDialog.close()
            polygonpointTool.tool.changeEditorMode( 3 )
            root.clearmap()
        }
        dialogtext: "Do you really want to discard modifications and clear the current mapbag ? "
    }
}