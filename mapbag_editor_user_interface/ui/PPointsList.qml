import QtQuick 2.12
import QtQml 2.12
import QtQuick.Layouts 1.1
import QtQuick.Controls 2.2
import Hector.Controls 1.0
import Hector.Utils 1.0
import Hector.Style 1.0
import Hector.Icons 1.0
import Ros 1.0

Rectangle {
    id: plist

    property bool locked: false
    property string list_remove: iconFromCharCode(0xf0413)
    property string list_plus: iconFromCharCode(0xf0412)
    property int clicked_nummer: 0
    property int polygonmode: 2
    property var polygonpointTool

    signal confirmPolygon()
    signal updatePolygon()
    signal wrongInput()
    signal changeMode()
    
    color: Style.background.container
    width: Units.pt(200)
    border {color: style.primary.color; width: Units.pt(2)}

    // QtObject {
    //     id: pTool
    //     property var current: (rviz && rviz.toolManager && rviz.toolManager.currentTool ) || {classId: ''} 
    //     property var polygonpointTool: {
    //         if (!rviz || !rviz.toolManager) return
    //         for (var tool of rviz.toolManager.tools) {
    //             if (tool.classId  == "qml_class/PolygonTool") return tool
    //         }
    //         return null
    //     }
    // }

    // function getPointList() {
    //     if (!pTool.polygonpointTool) return []

    //     var polygonpoints = pTool.polygonpointTool.tool.polygonpoints
    //     var dataArray = []

    //     for (var i = 0; i < polygonpoints.length; ++i) {
    //         let pos = polygonpoints[i].position
    //         let n = polygonpoints[i].nummer
    //         var list_item = {}
    //         list_item.i = n
    //         list_item.pos_data = "[x: " + pos.x.toFixed(2) + ", y: " + pos.y.toFixed(2) + ", z: " + pos.z.toFixed(2) + "]"
    //         dataArray.push(list_item)
    //     }
    //     return dataArray
    // }
    function getPointList() {
        if (!root.polygonpointTool) return []

        var polygonpoints = root.polygonpointTool.tool.polygonpoints
        var dataArray = []

        for (var i = 0; i < polygonpoints.length; ++i) {
            let pos = polygonpoints[i].position
            let n = polygonpoints[i].nummer
            var list_item = {}
            list_item.i = n
            list_item.pos_data = "[x: " + pos.x.toFixed(2) + ", y: " + pos.y.toFixed(2) + ", z: " + pos.z.toFixed(2) + "]"
            dataArray.push(list_item)
        }
        return dataArray
    }

    function iconFromCharCode (codePt) {
        if (codePt > 0xFFFF) {
            codePt -= 0x10000;
            return String.fromCharCode(0xD800 + (codePt >> 10), 0xDC00 + (codePt & 0x3FF));
        }
        return String.fromCharCode(codePt);
    }

    function getMsg() {
        if (!polygonpointTool) return []

        var polygonpoints = polygonpointTool.tool.polygonpoints
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

    AddPpointDialog {
        id: addPpointDialog
        polygonpointTool: root.polygonpointTool
    }

    ColumnLayout {
        anchors.fill: plist
        anchors.leftMargin: Units.pt(6) 
        anchors.rightMargin: Units.pt(6) 
        anchors.topMargin: Units.pt(4) 
        spacing: 0

        RowLayout  {
            Layout.fillWidth: true
            Layout.preferredHeight: Units.pt(24)
            spacing: 0

            Rectangle {
                Layout.fillWidth: true
                
                Text {
                id: ppointlist
                anchors.centerIn: parent
                text: "P-points List"
                font { pointSize: 12; weight: Font.Bold }
                leftPadding: Units.pt(20)
                rightPadding: Units.pt(20)        
                }
            }

            StyledButton {
                Layout.preferredWidth: Units.pt(20)
                Layout.preferredHeight: Units.pt(20)
                Layout.rightMargin: Units.pt(16)
                Layout.alignment: Qt.AlignRight

                contentItem: AutoSizeText {
                    anchors.fill: parent
                    text: plist.locked ? Style.icons.lockClosed : Style.icons.lockOpen
                    font { family: Style.iconFontFamily }
                }
                onClicked: plist.locked = !plist.locked
            }
        }

        RowLayout {
            spacing: 0
            Layout.fillWidth: true
            Layout.preferredHeight: Units.pt(24)

            Rectangle {
                Layout.fillWidth: true

                Text {
                    anchors.centerIn: parent
                    font { pointSize: 12; weight: Font.Bold }
                    text: (polygonpointTool && polygonpoints.length || 0) + " Polygonpoints"
                    leftPadding: Units.pt(8)
                    rightPadding: Units.pt(8)
                }
            }

            StyledButton {
                Layout.preferredWidth: Units.pt(24)
                Layout.preferredHeight: Units.pt(24)
                Layout.rightMargin: Units.pt(12)
                visible: !locked

                contentItem: AutoSizeText {
                    anchors.fill: parent
                    text: list_remove
                    font { family: Style.iconFontFamily }
                }
                onClicked: polygonpointTool.tool.clearPolygonpoints()
            }
        }

        ListView {
            Layout.fillHeight: true
            Layout.fillWidth: true            
            spacing: Units.pt(2)
            clip: true

            model: {
                var dataArray = getPointList()
                return dataArray
            }           

            delegate: Rectangle {
                color: ( clicked_nummer == modelData.i ) ? Style.background.content : Style.background.container
                width: parent.width
                height: Units.pt(28)

                Text {
                    anchors.left: parent.left
                    anchors.leftMargin: Units.pt(14)
                    text: "P-point " + modelData.i + " : \n" + modelData.pos_data
                }
                
                MouseArea {
                    anchors.fill: parent
                    acceptedButtons: Qt.LeftButton
                    hoverEnabled: true
                    onClicked: {
                        clicked_nummer = modelData.i
                        polygonpointTool.tool.setMarker(clicked_nummer)
                    }
                    onDoubleClicked: {
                        clicked_nummer = 0
                    }
                }

                Button {
                    height: Units.pt(22)
                    width: Units.pt(22)
                    anchors.right: parent.right
                    anchors.rightMargin: Units.pt(4)
                    anchors.verticalCenter: parent.verticalCenter
                    visible: !locked
                    hoverEnabled: true    

                    contentItem: AutoSizeText {
                        anchors.fill: parent
                        text: parent.hovered ? Style.icons.trashOpen : Style.icons.trash
                        font { family: Style.iconFontFamily }
                    }

                    onClicked: polygonpointTool.tool.removePolygonpoint( modelData.i - 1 )          
                    
                    background: Rectangle {
                        implicitWidth: parent.width
                        implicitHeight: parent.height
                        color: Style.background.content
                    }          
                }        
            }

            footer: ColumnLayout {
                spacing: Units.pt(2)

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(24)
                    spacing: 0
                    visible: !locked

                    StyledButton {
                        Layout.preferredWidth: Units.pt(114)
                        Layout.preferredHeight: Units.pt(20)
                        Layout.topMargin: Units.pt(2)
                        Layout.leftMargin: Units.pt(32)
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
                                    text: qsTr("Confirm Polygon")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }

                        onClicked: {                             
                            plist.locked = !plist.locked
                            plist.confirmPolygon()
                            polygonpointTool.tool.changeEditorMode( 1 )
                            var msgArray = getMsg()
                            Service.callAsync( "/mapbag_editor_server_node/polygongridmap", "mapbag_editor_msgs/Submap", 
                                                { datas: msgArray, mode: polygonmode }, function(response) { 
                                                    if( response.result != 0 ) { plist.wrongInput() }
                                                } )
                        }
                    }
                }

                RowLayout {
                    width: parent.width
                    height: Units.pt(28)
                    spacing: Units.pt(5)
                    visible: !locked

                    StyledButton {
                        Layout.fillHeight: true
                        Layout.preferredWidth: Units.pt(95)
                        Layout.leftMargin: Units.pt(11)
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
                                text: qsTr("Delete all")
                                font { pointSize: 13; weight: Font.Bold }
                                color: "white"
                            }

                            Text {
                                Layout.rightMargin: Units.pt(6)              
                                text: list_remove
                                font { pointSize: 20; weight: Font.Bold }
                                color: "white"
                            }
                            }
                        }

                        onClicked: polygonpointTool.tool.clearPolygonpoints()
                    }

                    StyledButton {
                        Layout.fillHeight: true
                        Layout.preferredWidth: Units.pt(60) 
                        Layout.topMargin: Units.pt(2)
                        style: Style.activeStyle
                        
                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.fillWidth: true 
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(6)                
                                    text: "Add"
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }

                                Text {
                                    Layout.rightMargin: Units.pt(6)               
                                    text: list_plus
                                    font { pointSize: 20; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }

                        onClicked: { addPpointDialog.open() }
                    }
                }

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.topMargin: Units.pt(6) 
                    Layout.bottomMargin: Units.pt(3) 
                    Layout.leftMargin: Units.pt(13)
                    spacing: 0
                    visible: !locked

                    SectionHeader {
                        Layout.columnSpan: 2
                        Layout.fillWidth: true
                        text: "Mode Change As"
                    }
                }

                RowLayout {
                    width: parent.width
                    height: Units.pt(28)
                    spacing: Units.pt(5)
                    visible: !locked

                    StyledButton {
                        Layout.fillHeight: true
                        Layout.preferredWidth: Units.pt(105)
                        Layout.leftMargin: Units.pt(34)
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
                                    text: qsTr("Primitive Mode")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }

                        onClicked: {
                            polygonpointTool.tool.changeEditorMode( 2 )
                            polygonpointTool.tool.clearPolygonpoints()
                            plist.changeMode()
                        }
                    }
                }
                
                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(24)
                    spacing: 0
                    visible: locked

                    StyledButton {
                        Layout.preferredWidth: Units.pt(114)
                        Layout.preferredHeight: Units.pt(20)
                        Layout.topMargin: Units.pt(2)
                        Layout.leftMargin: Units.pt(28)
                        style: Style.activeStyle

                        contentItem: Rectangle {
                            anchors.fill: parent
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: Units.pt(6)

                                Text {
                                    Layout.alignment: Qt.AlignLeft
                                    Layout.leftMargin: Units.pt(8)                
                                    text: qsTr("Update Polygon")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }

                        onClicked: { 
                            plist.locked = !plist.locked
                            plist.updatePolygon()
                            polygonpointTool.tool.changeEditorMode( 0 )
                            polygonpointTool.tool.dropChanges()
                        }
                    }
                }
            }
        }
    }
}