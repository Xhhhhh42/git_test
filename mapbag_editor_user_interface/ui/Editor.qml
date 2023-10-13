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
    property var polygonpointTool

    signal changeSaved()

    function getMsg() {
        if (!polygonpointTool) return []

        var polygonpoints = polygonpointTool.tool.polygonpoints
        var tempo = polygonpointTool.tool.tempo
        var msgArray = []

        for (var i = 0; i < polygonpoints.length; ++i) {
            let pos = polygonpoints[i].position
            var msg_item = {}
            msg_item.x = pos.x
            msg_item.y = pos.y
            msg_item.z = pos.z
            msgArray.push(msg_item)
        }
        return msgArray
    }   

    ConfirmationDialog {
        id: deleteSubmapDialog
        polygonpointTool: root.polygonpointTool
        title: "Confirm the delete ?!!"
        onAccepted: {
            Service.callAsync("/mapbag_editor_server_node/modechange", "hector_std_msgs/StringService", {param: "2"})
            deleteSubmapDialog.polygonpointTool.tool.deleteSubmap()
            deleteSubmapDialog.close()
        }
        dialogtext: "Do you really want to delete submap ? "
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        Rectangle {
            Layout.fillHeight: true
            Layout.preferredWidth: Units.pt(150)
            Layout.topMargin: -Units.pt(4.5)
            color: Style.background.container

            ColumnLayout {
                anchors.fill: parent
                anchors.leftMargin: Units.pt(4)
                anchors.topMargin: -Units.pt(3)
                spacing: -Units.pt(10)

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(22)

                    StyledButton {
                        Layout.preferredWidth: Units.pt(140)
                        style: Style.activeStyle
                                
                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                Text {
                                    Layout.alignment: Qt.AlignHCenter
                                    text: qsTr("Integrate to Mapbag")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }
                        onClicked: {
                            polygonpointTool.tool.publishToServer()
                            polygonpointTool.tool.clearPolygonpoints()
                            root.changeSaved()
                        }
                    }
                }

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(22)
                    spacing: 0

                    StyledButton {
                        Layout.preferredWidth: Units.pt(105)
                        Layout.preferredHeight: Units.pt(20)
                        style: Style.activeStyle

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(6)                
                                    text: qsTr("Delete Submap")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }
                        onClicked: { 
                            deleteSubmapDialog.open() 
                        }
                    }
                }

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(22)
                    spacing: 0

                    StyledButton {
                        Layout.preferredWidth: Units.pt(86)
                        Layout.preferredHeight: Units.pt(20)
                        Layout.topMargin: Units.pt(2)
                        style: Style.activeStyle

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(6)                
                                    text: qsTr("Intepolation")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }
                        onClicked: { 
                            var msgArray = getMsg()
                            Service.callAsync("/mapbag_editor_server_node/interpolation", "hector_std_msgs/StringService", 
                                            {})
                        }
                    }
                }

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(22)

                    StyledButton {
                        Layout.preferredWidth: Units.pt(60)
                        style: Style.activeStyle
                                
                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                Text {
                                    Layout.alignment: Qt.AlignHCenter
                                    text: qsTr("Smooth")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }
                        onClicked: {}
                    }
                }
            }
        }

        ColumnLayout {
            Layout.fillHeight: true
            Layout.fillWidth: true
            spacing: 0

            RowLayout {
                Layout.fillWidth: true

                Rectangle {
                    Layout.fillHeight: true
                    Layout.topMargin: -Units.pt(4.5)
                    width: Units.pt(2) 
                    color: style.primary.color
                }

                Rectangle {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    Layout.topMargin: -Units.pt(4.5)
                    Layout.leftMargin: -Units.pt(3.5)
                    color: Style.background.container

                    Text {
                        anchors.centerIn: parent
                        text: qsTr("Property Height  ")
                        color: "black"
                    }
                }
            }                

            RowLayout  {
                Layout.fillWidth: true
                Layout.preferredHeight: Units.pt(24)
                Layout.leftMargin: Units.pt(12)
                spacing: 0

                Text {
                    Layout.alignment: Qt.AlignLeft
                    Layout.leftMargin: Units.pt(6)                
                    anchors.fill: parent.center
                    text: "Heightvalue of change:"
                    font { family: Style.iconFontFamily }
                }

                Text {
                    Layout.alignment: Qt.AlignLeft
                    Layout.leftMargin: Units.pt(6)                
                    text: ( polygonpointTool && polygonpointTool.tool.tempo.z.toFixed(3) || "null")        
                    font { pointSize: 12; weight: Font.Bold }
                    color: "black"
                }
            }

            RowLayout  {
                Layout.fillWidth: true
                Layout.preferredHeight: Units.pt(24)
                Layout.leftMargin: Units.pt(18)
                spacing: 0

                TextField {
                    id: heightvalueTextField
                    Layout.preferredHeight: Units.pt(24)
                    Layout.preferredWidth: Units.pt(100)
                    placeholderText: qsTr("Height input...")
                    validator: DoubleValidator{}
                }

                StyledButton {
                    Layout.preferredWidth: Units.pt(60)
                    Layout.preferredHeight: Units.pt(24)
                    Layout.rightMargin: Units.pt(12)
                    Layout.alignment: Qt.AlignRight
                    style: Style.activeStyle

                    contentItem:  Rectangle {
                        anchors.fill: parent
                        color: "transparent"

                        RowLayout {
                            anchors.fill: parent
                            spacing: Units.pt(6)

                            Text {
                                Layout.alignment: Qt.AlignHCenter
                                text: qsTr("Apply")
                                font { pointSize: 13; weight: Font.Bold }
                                color: "white"
                            }
                        }
                    }
                    onClicked: { 
                        // Service.callAsync("/mapbag_editor_server_node/set_heightvalue", "hector_std_msgs/StringService", {param: heightvalueTextField.text})
                        if ( !heightvalueTextField.text ) return
                        var height_service = {}
                        height_service.x = parseFloat(heightvalueTextField.text)
                        height_service.y = add_switch.checked? 1 : 0
                        heightvalueTextField.clear()
                        polygonpointTool.tool.setHeight( height_service.y, height_service.x )
                    }
                }

                Switch {
                    id: add_switch
                    text: qsTr("Additiv")
                    checked: true 

                    indicator: Rectangle {
                        implicitWidth: 40
                        implicitHeight: 20
                        x: add_switch.leftPadding
                        y: parent.height / 2 - height / 2
                        radius: 13
                        color: add_switch.checked ? style.primary.color : "#ffffff"
                        border.color: add_switch.checked ? style.primary.color : "#cccccc"

                        Rectangle {
                            x: add_switch.checked ? parent.width - width : 0
                            width: 20
                            height: 20
                            radius: 10
                            color: add_switch.down ? "#cccccc" : "#ffffff"
                            border.color: add_switch.checked ? (add_switch.down ? style.primary.color : "grey") : "#999999"
                        }
                    }

                    contentItem: Text {
                        text: add_switch.text
                        font: add_switch.font
                        opacity: enabled ? 1.0 : 0.3
                        // color: add_switch.down ? style.primary.color : "white"
                        color: "black"
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: add_switch.indicator.width + add_switch.spacing
                    }
                }
            }

            RowLayout  {
                Layout.preferredHeight: Units.pt(24)
                Layout.leftMargin: Units.pt(10)
                Layout.topMargin: Units.pt(6)
                spacing: 0

                Text {
                    id: tip
                    Layout.alignment: Qt.AlignHCenter
                    Layout.preferredWidth: Units.pt(270)
                    Layout.preferredHeight: Units.pt(24)
                    text: "( Drag mouse to change the height of Submap )"
                    wrapMode: Text.WordWrap
                    color: "black"
                    width: 40
                }
            }
        }
    }
}

