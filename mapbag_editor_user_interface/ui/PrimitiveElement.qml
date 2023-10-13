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

    PrimitiveDialog {
        id: primitiveDialog
        polygonpointTool: root.polygonpointTool
        onAdd: {
            pro_drawer.drawerOpen = !pro_drawer.drawerOpen
        }
    }

    RowLayout {
        anchors.fill: root
        spacing: 0

        Rectangle {
            Layout.fillHeight: true
            Layout.preferredWidth: Units.pt(74)
            Layout.topMargin: -Units.pt(4.5)
            Layout.rightMargin: -Units.pt(10)
            color: Style.background.container 

            ColumnLayout {
                anchors.fill: parent
                anchors.leftMargin: Units.pt(6)
                anchors.topMargin: -Units.pt(3)
                spacing: -Units.pt(12)
                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(22)
                    visible: !pro_drawer.drawerOpen

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
                                    text: qsTr("Add")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }

                        onClicked: { primitiveDialog.open() }
                    }
                }

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(12)

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
                                    text: qsTr("Save")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }

                        onClicked: { 
                            polygonpointTool.tool.primitiveSave()
                            polygonpointTool.tool.primitiveClear()
                            primitiveDialog.modepub = -1
                            pro_drawer.drawerOpen = !pro_drawer.drawerOpen
                        }
                    }
                } 
                RowLayout  {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(22)
                    visible: pro_drawer.drawerOpen

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
                                    text: qsTr("Clear")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }

                        onClicked: {
                            polygonpointTool.tool.primitiveClear()
                            primitiveDialog.modepub = -1
                            pro_drawer.drawerOpen = !pro_drawer.drawerOpen
                        }
                    }
                } 
            }
        }

        // ColumnLayout {
        //     Layout.preferredWidth: Units.pt(20)
        //     Layout.fillHeight: true

        //     Rectangle {
        //         Layout.alignment: Qt.AlignHCenter 
        //         width: Units.pt(1) 
        //         Layout.fillHeight: true 
        //         Layout.topMargin: -Units.pt(3)
        //         color: "black"
        //     }
        // }

        ColumnLayout {
            Layout.preferredWidth: Units.pt(100)
            Layout.fillHeight: true
            Layout.topMargin: -Units.pt(4)
            Layout.leftMargin: Units.pt(9)

            Item {
                id: pro_drawer
                Layout.fillHeight: true
                Layout.fillWidth: true 

                property bool drawerOpen : false

                Rectangle {
                    width: parent.width
                    height: parent.height
                    color: !pro_drawer.drawerOpen ? "white" : "transparent"
                    // color: !pro_drawer.drawerOpen ? Style.background.container : "transparent"
                    y: !pro_drawer.drawerOpen ?  parent.height - openButton.height : 0

                    StyledButton {
                        id: openButton
                        anchors.left: parent.left
                        anchors.leftMargin: Units.pt(0.3)
                        anchors.right: parent.right
                        anchors.top: parent.top
                        style: Style.activeStyle
                        text: !pro_drawer.drawerOpen ? "All Properties of Primitive Element" : "Close"
                        onClicked: pro_drawer.drawerOpen = !pro_drawer.drawerOpen
                    }

                    Behavior on y {
                        NumberAnimation {
                            duration: 300
                        }
                    }
                }

                Rectangle {
                    id: drawer
                    width: pro_drawer.width
                    height: parent.height
                    color: "transparent"
                    visible: pro_drawer.drawerOpen

                    Behavior on height {
                        NumberAnimation {
                            duration: 300
                        }
                    }

                    Rectangle {
                        width: parent.width
                        height: parent.height
                        color: "transparent"
                        visible: pro_drawer.drawerOpen

                        RowLayout {
                            anchors.left: property_pri.left
                            anchors.right: property_pri.right
                            anchors.bottom: property_pri.top
                            anchors.bottomMargin: Units.pt(12)

                            Text {
                                text: (primitiveDialog.modepub == 0)? "Sphere" : (primitiveDialog.modepub == 1)? "Cuboid" : 
                                        (primitiveDialog.modepub == 2)? "Cube" : (primitiveDialog.modepub == 3)? "Triangular Prism" : ""
                            }

                            Text {
                                text: "    Raidus :  " + primitiveDialog.radiuspub
                                visible: primitiveDialog.radiuspub > 0
                            }

                            Text {
                                text: "    Winkel :  " + primitiveDialog.winkelpub + " Gard"
                                visible: primitiveDialog.winkelpub > 0
                                
                            }
                        }

                        Text {
                            id: property_pri
                            text: "Center :  ( " + primitiveDialog.xpub + " , " + primitiveDialog.ypub + " , " + primitiveDialog.zpub + " )                         "
                            anchors.centerIn: parent
                        }

                        RowLayout {
                            anchors.left: property_pri.left
                            anchors.right: property_pri.right
                            anchors.top: property_pri.bottom
                            anchors.topMargin: Units.pt(12)

                            Text {
                                text: "Length :  " + primitiveDialog.lengthpub 
                                visible: primitiveDialog.lengthpub > 0                    
                            }

                            Text {
                                text: "    Width :  " + primitiveDialog.widthpub
                                visible: primitiveDialog.widthpub > 0
                            }

                            Text {
                                text: "    Height :  " + primitiveDialog.heightpub
                                visible: primitiveDialog.heightpub > 0
                            }
                        }

                        MouseArea {
                            anchors.fill: parent
                            onClicked: {
                                pro_drawer.drawerOpen = !pro_drawer.drawerOpen;
                            }
                        }
                    }
                }

                transitions: Transition {
                    NumberAnimation {
                        target: drawer
                        property: "height"
                        duration: 300
                    }
                }

                states: [
                    State {
                        when: pro_drawer.drawerOpen
                        PropertyChanges {
                            target: drawer
                            height: pro_drawer.height + openButton.height
                        }
                    },
                    State {
                        when: !pro_drawer.drawerOpen
                        PropertyChanges {
                            target: drawer
                            height: 0
                        }
                    }
                ]
            }
        }
    }    
}

