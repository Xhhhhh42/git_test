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

    RowLayout {
        anchors.fill: root
        spacing: 0

        Rectangle {
            Layout.fillHeight: true
            Layout.preferredWidth: Units.pt(74)
            Layout.topMargin: -Units.pt(4.5)
            Layout.leftMargin: -Units.pt(1)
            Layout.rightMargin: -Units.pt(20)
            color: Style.background.container

            ColumnLayout {
                anchors.fill: parent
                anchors.leftMargin: Units.pt(6)
                anchors.topMargin: -Units.pt(3)
                spacing: -Units.pt(12)

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
                                    text: qsTr("Save")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }
                        onClicked: polygonpointTool.tool.submapVerschiebenSave()
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
                                    text: qsTr("Clear")
                                    font { pointSize: 13; weight: Font.Bold }
                                    color: "white"
                                }
                            }
                        }
                        onClicked: polygonpointTool.tool.submapVerschiebenClear()
                    }
                }
            }
        }

        // ColumnLayout {
        //     Layout.preferredWidth: Units.pt(20)
        //     Layout.fillHeight: true

        //     Rectangle {
        //         Layout.alignment: Qt.AlignLeft 
        //         width: Units.pt(1) 
        //         Layout.fillHeight: true 
        //         Layout.topMargin: -Units.pt(3)
        //         color: "black"
        //     }
        // }

        RowLayout  {
            Layout.alignment: Qt.AlignHCenter
            Layout.fillHeight: true

            ColumnLayout {
                Text {
                    id: tip_eins
                    // Layout.alignment: Qt.AlignLeft
                    Layout.fillHeight: true
                    Layout.preferredWidth: Units.pt(250)
                    Layout.topMargin: Units.pt(4)
                    Layout.leftMargin: -Units.pt(40)
                    text: "Tips"
                    wrapMode: Text.WordWrap
                    color: "black"
                    width: 40
                }

                Text {
                    id: tip_zwei
                    Layout.fillHeight: true
                    Layout.preferredWidth: Units.pt(250)
                    Layout.topMargin: -Units.pt(4)
                    text: "1. Use PolygonTool to select the Submap to be dragged. Switch to Interact Tool."
                    wrapMode: Text.WordWrap
                    color: "black"
                    width: 40
                }

                Text {
                    id: tip_drei
                    Layout.fillHeight: true
                    Layout.preferredWidth: Units.pt(250)
                    text: "2. Check the InteractiveMarkers tool in the Display panel on the left, and use it to move Sub-map."
                    wrapMode: Text.WordWrap
                    color: "black"
                    width: 40
                }
            }           
        }       
    }
}


