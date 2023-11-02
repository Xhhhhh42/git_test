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
    id: root

    property bool visible_bc: false
    property bool pri_instr: false
    property bool poly_instr: false
    property var polygonpointTool

    signal editorMode()
    signal polygonMode()
    signal pri_editorMode()
    signal wrongTransfer()
    
    color: Style.background.container
    width: Units.pt(200)
    height: Units.pt(205)
    border {color: style.primary.color; width: Units.pt(2)}

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons
        onWheel: wheel.accepted = true
    }

    ResizeHandle {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        resizeBorder: ResizeHandle.Border.Top
        resizeMode: ResizeHandle.SizeOnly
        height: Units.pt(4)
        target: root
        z: 2
        minimumHeight: Units.pt(120)
    }

    ColumnLayout {
        spacing: Units.pt(8)
        anchors.top: parent.top
        anchors.left: parent.left
        width: Units.pt(200)
        visible: !visible_bc

        RowLayout  {
            Layout.preferredWidth: Units.pt(154)
            Layout.topMargin: Units.pt(26) 
            Layout.bottomMargin: -Units.pt(1) 
            Layout.leftMargin: Units.pt(8)
            Layout.rightMargin: Units.pt(8)
            spacing: 0
            visible: !visible_bc

            SectionHeader {
                Layout.columnSpan: 2
                Layout.fillWidth: true
                text: "Select Mode"
            }
        }

        RowLayout  {
            Layout.fillWidth: true
            Layout.preferredHeight: Units.pt(24)
            spacing: 0
            visible: !visible_bc

            StyledButton {
                Layout.preferredWidth: Units.pt(114)
                Layout.preferredHeight: Units.pt(20)
                Layout.topMargin: Units.pt(12)
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
                            text: qsTr("Polygon Mode")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "white"
                        }
                    }
                }

                onClicked: { 
                    polygonpointTool.tool.changeEditorMode( 0 )
                    visible_bc = !visible_bc
                    root.height = Units.pt(244)
                }
            }

            MenuButton {
                id: poly_instruction
                Layout.preferredWidth: Units.pt(47)
                Layout.preferredHeight: Units.pt(20)
                Layout.topMargin: Units.pt(12)

                contentItem: Rectangle {
                    anchors.fill: parent
                    color: "transparent"

                    RowLayout {
                        anchors.fill: parent
                        spacing: Units.pt(6)

                        Text {
                            Layout.alignment: Qt.AlignLeft
                            Layout.leftMargin: Units.pt(12)                
                            text: qsTr("tips")
                            color: "black"
                        }
                    }
                }

                onClicked: { 
                    if( !text_poly1.visible ) {
                        text_poly1.visible = true
                        text_poly2.visible = true
                        test_pri1.visible = false
                        test_pri2.visible = false
                    } else if( text_poly1.visible ) {
                        text_poly1.visible = false
                        text_poly2.visible = false
                        test_pri1.visible = false
                        test_pri2.visible = false
                    }
                }
            }
        }

        RowLayout  {
            Layout.fillWidth: true
            Layout.preferredHeight: Units.pt(24)
            Layout.topMargin: Units.pt(12)
            spacing: 0
            visible: !visible_bc

            StyledButton {
                Layout.preferredWidth: Units.pt(114)
                Layout.preferredHeight: Units.pt(20)
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
                            text: qsTr("Primitive Mode")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "white"
                        }
                    }
                }

                onClicked: { 
                    polygonpointTool.tool.changeEditorMode( 2 )
                    root.pri_editorMode()
                }
            }

            MenuButton {
                id: pri_instruction
                Layout.preferredWidth: Units.pt(47)
                Layout.preferredHeight: Units.pt(20)

                contentItem: Rectangle {
                    anchors.fill: parent
                    color: "transparent"

                    RowLayout {
                        anchors.fill: parent
                        spacing: Units.pt(6)

                        Text {
                            Layout.alignment: Qt.AlignLeft
                            Layout.leftMargin: Units.pt(12)                
                            text: qsTr("tips")
                            color: "black"
                        }
                    }
                }

                onClicked: { 
                    if( !test_pri1.visible ) {
                        test_pri1.visible = true
                        test_pri2.visible = true
                        text_poly1.visible = false
                        text_poly2.visible = false
                    } else if( test_pri1.visible ) {
                        test_pri1.visible = false
                        test_pri2.visible = false
                        text_poly1.visible = false
                        text_poly2.visible = false
                    }
                }
            }
        }

        RowLayout  {
            Layout.preferredWidth: Units.pt(124)
            Layout.preferredHeight: Units.pt(24)
            Layout.topMargin: Units.pt(12)
            Layout.leftMargin: Units.pt(12)
            Layout.rightMargin: Units.pt(14)

            ColumnLayout {
                Text {
                    id: test_pri1
                    Layout.preferredWidth: Units.pt(175)
                    Layout.leftMargin: Units.pt(6)                
                    text: "Goal :  Inserting primitives into the map"
                    wrapMode: Text.WordWrap
                    color: "black"
                    visible: false
                    width: 40
                }

                Text {
                    id: test_pri2
                    Layout.preferredWidth: Units.pt(175)
                    Layout.leftMargin: Units.pt(6)                
                    text: "( e.g., ramps, rectangles, cylinders, etc. )"
                    wrapMode: Text.WordWrap
                    color: "black"
                    visible: false
                    width: 40
                }
            }

            ColumnLayout {
                Text {
                    id: text_poly1
                    Layout.preferredWidth: Units.pt(175)
                    Layout.leftMargin: Units.pt(4)       
                    text: "Edit the submap by selecting it with the polygon tool."
                    wrapMode: Text.WordWrap
                    color: "black"
                    visible: false
                    width: 40
                }

                Text {
                    id: text_poly2
                    Layout.preferredWidth: Units.pt(175)
                    Layout.leftMargin: Units.pt(4)  
                    text: "( Existing functions: Delete, Pan, Change height, Interpolate, Smooth )"
                    wrapMode: Text.WordWrap
                    color: "black"
                    visible: false
                    width: 40
                }
            }
            
        }
    }

    PPointsList {
        id: plist
        anchors.fill: parent
        height: Units.pt(244)
        visible: visible_bc
        polygonpointTool: root.polygonpointTool

        onConfirmPolygon: { root.polygonMode() }
        // onUpdatePolygon: { root.polygonMode() }
        onChangeMode: { 
            visible_bc = !visible_bc
            root.height = Units.pt(205)
        }
        onWrongInput: { root.wrongTransfer() }
    }
}