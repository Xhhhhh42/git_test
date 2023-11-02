import QtQuick 2.12
import QtQml 2.12
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import Hector.Controls 1.0
import Hector.Utils 1.0
import Hector.Style 1.0
import Hector.InternalControls 1.0

Rectangle{
    id: root

    property var style
    property int curr_index : 1
    property var polygonpointTool
    property bool wrongInfo_visible

    signal save()
    signal clear()

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        Rectangle {
            z: 2
            height: Units.pt(2) 
            Layout.fillWidth: true 
            color: style.primary.color
        }

        RowLayout {
            id: controlPanel
            Layout.fillWidth: true
            Layout.topMargin: -Units.pt(1)
            
            SystemSettings {
                z: 2
                Layout.fillHeight: true
                Layout.preferredWidth: Units.pt(240)
                Layout.rightMargin: -Units.pt(3)
                wronginfo: wrongInfo_visible
                onSaveMap: { root.save() }
                onClearMap: { root.clear() }
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true

                TabBar {
                    id: bar
                    Layout.fillWidth: true
                    spacing: 0
                    z: 1
                    currentIndex: curr_index
                    
                    StyledTabButton {
                        style: root.style
                        text: "Primitive Element"
                        font { pointSize: 13; weight: Font.Bold }
                        onClicked: {
                            polygonpointTool.tool.changeEditorMode( 2 )
                        }
                    }
                    StyledTabButton {
                        style: root.style
                        text: "Editor"
                        font { pointSize: 13; weight: Font.Bold }
                        onClicked: {
                            polygonpointTool.tool.changeEditorMode( 1 )
                        }
                    }
                    StyledTabButton {
                        style: root.style
                        text: "Verschieben"
                        font { pointSize: 13; weight: Font.Bold }
                        onClicked: {
                            polygonpointTool.tool.changeEditorMode( 3 )
                        }
                    }
                }

                RowLayout {
                    Layout.preferredHeight: Units.pt(2)
                    Layout.fillWidth: true
                    Layout.topMargin: -Units.pt(4)

                    Rectangle {
                        z: 2
                        height: Units.pt(2) 
                        Layout.fillWidth: true 
                        color: style.primary.color
                    }
                }

                StackLayout {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    currentIndex: bar.currentIndex
                    
                    PrimitiveElement {
                        id: primitiveElement
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        polygonpointTool: root.polygonpointTool
                    }

                    Editor {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        polygonpointTool: root.polygonpointTool
                    }

                    VerschiebenTab {
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        polygonpointTool: root.polygonpointTool
                        onVers_saved: {
                            primitiveElement.simulateClearClick()
                        }
                        onVers_cleared: {
                            primitiveElement.simulateClearClick()
                        }
                    }
                }
            }
        }
    }

}