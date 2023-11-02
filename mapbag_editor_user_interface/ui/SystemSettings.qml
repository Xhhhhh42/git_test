import QtQuick 2.12
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.1
import Hector.Controls 1.0
import Hector.Style 1.0
import Hector.Utils 1.0
import Hector.Icons 1.0
import Ros 1.0

Rectangle {
    id: root

    property string undo: iconFromCharCode(0xf054d)
    property string redo: iconFromCharCode(0xf044f)
    property string config: iconFromCharCode(0xf08bb)
    property string save: iconFromCharCode(0xf0818) 
    property string clear: iconFromCharCode(0xf0a7a) 
    property int polygon_config: 0
    property int intepolation_config: 0
    property int smooth_config: 0
    property bool wronginfo

    //https://pictogrammers.github.io/@mdi/font/6.1.95/

    signal changeSaved()
    signal saveMap()
    signal clearMap()

    QtObject {
    id: pTool
    property var current: (rviz && rviz.toolManager && rviz.toolManager.currentTool ) || {classId: ''} 
    property var polygonpointTool: {
        if (!rviz || !rviz.toolManager) return
        for (var tool of rviz.toolManager.tools) {
            if (tool.classId  == "qml_class/PolygonTool") return tool
        }
        return null
    }
    }

    function iconFromCharCode (codePt) {
    if (codePt > 0xFFFF) {
        codePt -= 0x10000;
        return String.fromCharCode(0xD800 + (codePt >> 10), 0xDC00 + (codePt & 0x3FF));
    }
    return String.fromCharCode(codePt);
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.topMargin: Units.pt(8)
            spacing: 0

            RowLayout  {
                Layout.fillWidth: true
                Layout.preferredHeight: Units.pt(20)
                Layout.topMargin: -Units.pt(14)

                SectionHeader {
                    Layout.columnSpan: 2
                    Layout.fillWidth: true
                    text: config
                    font { pointSize: 18 }
                }
            }

            RowLayout  {
                Layout.leftMargin: Units.pt(10)
                Layout.preferredHeight: Units.pt(24)
                Layout.topMargin: -Units.pt(4)
                spacing: 0

                StyledButton {
                    Layout.preferredWidth: Units.pt(20)
                    Layout.preferredHeight: Units.pt(20)
                    Layout.alignment: Qt.AlignRight
                            
                    contentItem: AutoSizeText {
                        anchors.fill: parent
                        text: undo
                        font { family: Style.iconFontFamily }
                    }
                    onClicked: {
                        Service.callAsync("/mapbag_editor_server_node/undo", "hector_std_msgs/StringService",
                                                    { param: ""})
                        polygonpointTool.tool.clearPolygonpoints()
                    }
                }

                StyledButton {
                    Layout.preferredWidth: Units.pt(20)
                    Layout.preferredHeight: Units.pt(20)
                    Layout.alignment: Qt.AlignRight

                    contentItem: AutoSizeText {
                        anchors.fill: parent
                        text: redo
                        font { family: Style.iconFontFamily }
                    }
                    onClicked: {
                        Service.callAsync("/mapbag_editor_server_node/redo", "hector_std_msgs/StringService",
                                                    { param: ""})
                    }
                }

                StyledButton {
                    Layout.preferredWidth: Units.pt(20)
                    Layout.preferredHeight: Units.pt(20)
                    Layout.alignment: Qt.AlignRight

                    contentItem: AutoSizeText {
                        anchors.fill: parent
                        text: save
                        font { family: Style.iconFontFamily }
                    }
                    onClicked: {
                        root.saveMap()
                    }
                }

                StyledButton {
                    Layout.preferredWidth: Units.pt(20)
                    Layout.preferredHeight: Units.pt(20)
                    Layout.alignment: Qt.AlignRight

                    contentItem: AutoSizeText {
                        anchors.fill: parent
                        text: clear
                        font { family: Style.iconFontFamily }
                    }
                    onClicked: {
                        root.clearMap()
                    }
                }

                Text {
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: Units.pt(6)
                    Layout.leftMargin: Units.pt(10)
                    text: "Wrong Konkavhull Points"
                    color: "red"
                    visible: wronginfo
                }
            } 

            RowLayout  {
                Layout.leftMargin: Units.pt(12)
                Layout.preferredHeight: Units.pt(20)
                spacing: Units.pt(6)

                Text {
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: Units.pt(4)
                    text: "Polygon Selection :"
                }

                ComboBox {
                    id: selectionComboBox
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: -Units.pt(2)
                    Layout.leftMargin: Units.pt(4)
                    model: [ "Konkavhull", "Konvexhull", "Room_Walls" ]

                    onActivated: {
                        if( polygon_config != currentIndex ) {
                            polygon_config = currentIndex
                            Service.callAsync("/mapbag_editor_server_node/systemsettings", "mapbag_editor_msgs/Settings",
                                             { polygon_mode: polygon_config, intepolation_mode: intepolation_config, smooth_mode: smooth_config })
                        }
                    }
                }
            }

            RowLayout  {
                Layout.leftMargin: Units.pt(12)
                Layout.preferredHeight: Units.pt(20)
                spacing: Units.pt(6)

                Text {
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: Units.pt(3)
                    text: "Intepolation Mode :"
                }

                ComboBox {
                    id: intepolationComboBox
                    Layout.preferredHeight: Units.pt(20)
                    Layout.preferredWidth: Units.pt(91)
                    Layout.topMargin: -Units.pt(2)
                    Layout.leftMargin: Units.pt(2)
                    model: [ "Global", "Addtiv" ]

                    onActivated: {
                        if( intepolation_config != currentIndex ) {
                            intepolation_config = currentIndex
                            Service.callAsync("/mapbag_editor_server_node/systemsettings", "mapbag_editor_msgs/Settings",
                                                    { polygon_mode: polygon_config, intepolation_mode: intepolation_config, smooth_mode: smooth_config })
                        }
                    }
                }
            }

            RowLayout  {
                Layout.leftMargin: Units.pt(12)
                Layout.preferredHeight: Units.pt(20)
                spacing: Units.pt(6)

                Text {
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: Units.pt(4)
                    text: "Smooth Mode :"
                }

                ComboBox {
                    id: smoothComboBox
                    Layout.preferredHeight: Units.pt(20)
                    Layout.preferredWidth: Units.pt(124)
                    Layout.leftMargin: Units.pt(8)
                    model: [ "Adaptive Filter", "Spline Curve" ]

                    onActivated: {
                        if( smooth_config != currentIndex ) {
                            smooth_config = currentIndex
                            Service.callAsync("/mapbag_editor_server_node/systemsettings", "mapbag_editor_msgs/Settings",
                                                    { polygon_mode: polygon_config, intepolation_mode: intepolation_config, smooth_mode: smooth_config })
                        }
                    }
                }
            }
        }

        ColumnLayout {
            Layout.preferredWidth: Units.pt(2)
            Layout.fillHeight: true
            Layout.rightMargin: -Units.pt(1)

            Rectangle {
                z: 2
                width: Units.pt(2) 
                Layout.fillHeight: true 
                color: style.primary.color
            }
        }

    }

        
}

