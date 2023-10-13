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

    //https://pictogrammers.github.io/@mdi/font/6.1.95/

    signal changeSaved()

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
                        // Service.callAsync("/mapbag_editor_server_node/redo", "hector_std_msgs/StringService",
                        //                             { param: ""})
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
                        // Service.callAsync("/mapbag_editor_server_node/redo", "hector_std_msgs/StringService",
                        //                             { param: ""})
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
                    text: "Polygon Selection :"
                }

                ComboBox {
                    id: selectionComboBox
                    Layout.preferredHeight: Units.pt(20)
                    Layout.topMargin: -Units.pt(2)
                    Layout.leftMargin: Units.pt(4)
                    model: [ "Konvexhull", "Sequentiell" ]
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
                    Layout.preferredWidth: Units.pt(70)
                    Layout.topMargin: -Units.pt(2)
                    Layout.leftMargin: Units.pt(2)
                    model: [ "Global", "Addtiv" ]
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
                    Layout.preferredWidth: Units.pt(100)
                    Layout.leftMargin: Units.pt(24)
                    model: [ "Spline Curve", "Sequentiell" ]
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

