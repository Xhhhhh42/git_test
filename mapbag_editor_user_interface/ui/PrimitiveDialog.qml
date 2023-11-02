import QtQuick 2.12
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.1
import Hector.Controls 1.0
import Hector.Utils 1.0

Dialog {
    id: root

    property var polygonpointTool
    property bool x_rise: true
    property bool y_rise: false
    property int modepub : -1
    property int lengthpub
    property int widthpub
    property int heightpub
    property int winkelpub
    property int radiuspub
    property var xpub
    property var ypub
    property var zpub

    function clear_Sphere() {
        xTextField_Sphere.clear()
        yTextField_Sphere.clear()
        zTextField_Sphere.clear()
        radius_Sphere.clear()
    }

    function clear_Cuboid() {
        xTextField_Cuboid.clear()
        yTextField_Cuboid.clear()
        zTextField_Cuboid.clear()
        length_Cuboid.clear()
        width_Cuboid.clear()
        height_Cuboid.clear()
    }

    function clear_Cube() {
        xTextField_Cube.clear()
        yTextField_Cube.clear()
        zTextField_Cube.clear()
        length_Cube.clear()
    }

    function clear_Prism() {
        xTextField_Prism.clear()
        yTextField_Prism.clear()
        zTextField_Prism.clear()
        length_Prism.clear()
        width_Prism.clear()
        height_Prism.clear()
    }

    function init() {
        x_rise = true
        y_rise = false
        lengthpub = 0
        widthpub = 0
        heightpub = 0
        winkelpub = 0
        radiuspub = 0
        xpub = 0
        ypub = 0
        zpub = 0
    }

    signal add()

    visible: false
    width: Units.pt(470)
    height: Units.pt(300)
    title: "3D Primitive Creator"
    standardButtons: Dialog.Apply | Dialog.Cancel
    closePolicy: Popup.NoAutoClose
    parent: ApplicationWindow.overlay
    x: parent.x + (parent.width - width) / 2
    y: parent.y + (parent.height - height) / 2
    modal: true
    focus: true

    contentItem: Item {
        Rectangle {
            width: Units.pt(440)
            height: Units.pt(220)

            ColumnLayout {
                spacing: 0
                anchors.fill: parent

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: Units.pt(24)
                    Layout.topMargin: -Units.pt(6) 
                    spacing: Units.pt(6) 

                    Text {
                        Layout.alignment: Qt.AlignLeft
                        Layout.leftMargin: Units.pt(6)                
                        text: qsTr("Primitiv Type")
                        font { pointSize: 13; weight: Font.Bold }
                        color: "black"
                    }

                    ComboBox {
                        id: primitiveTypeComboBox
                        model: ["Sphere", "Cuboid", "Cube", "Tri.Prism"]
                        currentIndex: 0
                    }
                }

                RowLayout  {
                    Layout.fillWidth: true
                    Layout.topMargin: Units.pt(2) 
                    spacing: 0

                    SectionHeader {
                        Layout.columnSpan: 2
                        Layout.fillWidth: true
                        text: "Settings"
                    }
                }

                ColumnLayout {
                    spacing: Units.pt(2)
                    Layout.leftMargin: Units.pt(10)
                    Layout.topMargin: -Units.pt(20)
                    visible: primitiveTypeComboBox.currentIndex == 0

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        spacing: Units.pt(26) 

                        Text {
                            Layout.alignment: Qt.AlignLeft
                            Layout.leftMargin: -Units.pt(5)                
                            text: qsTr("Coordinates of the Sphere Center")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "black"
                        }

                        Text {               
                            text: qsTr("Property")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "black"
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: xTextField_Sphere
                            placeholderText: "X Coordinate"
                            validator: DoubleValidator{}
                        }

                        TextField {
                            Layout.leftMargin: Units.pt(102)
                            id: radius_Sphere
                            placeholderText: "Radius"
                            validator: IntValidator {bottom: 1}
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: yTextField_Sphere
                            placeholderText: "Y Coordinate"
                            validator: DoubleValidator{}
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: zTextField_Sphere
                            placeholderText: "Z Coor.( Default 0 )"
                            validator: DoubleValidator{}
                        }
                    }
                }

                ColumnLayout {
                    spacing: Units.pt(2)
                    Layout.leftMargin: Units.pt(10)
                    Layout.topMargin: -Units.pt(20)
                    visible: primitiveTypeComboBox.currentIndex == 1

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        spacing: Units.pt(26) 

                        Text {
                            Layout.alignment: Qt.AlignLeft
                            Layout.leftMargin: -Units.pt(5)                
                            text: qsTr("Coordinates of the Cuboid Center")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "black"
                        }

                        Text {               
                            text: qsTr("Property")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "black"
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: xTextField_Cuboid
                            placeholderText: "X Coordinate"
                            validator: DoubleValidator{}
                        }

                        TextField {
                            Layout.leftMargin: Units.pt(102)
                            id: length_Cuboid
                            placeholderText: "Length"
                            validator: IntValidator {bottom: 1}
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: yTextField_Cuboid
                            placeholderText: "Y Coordinate"
                            validator: DoubleValidator{}
                        }

                        TextField {
                            Layout.leftMargin: Units.pt(102)
                            id: width_Cuboid
                            placeholderText: "Width"
                            validator: IntValidator {bottom: 1}
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: zTextField_Cuboid
                            placeholderText: "Z Coor.( Default 0 )"
                            validator: DoubleValidator{}
                        }

                        TextField {
                            Layout.leftMargin: Units.pt(102)
                            id: height_Cuboid
                            placeholderText: "Height"
                            validator: IntValidator {bottom: 1}
                        }
                    }
                }

                ColumnLayout {
                    spacing: Units.pt(2)
                    Layout.leftMargin: Units.pt(10)
                    Layout.topMargin: -Units.pt(20)
                    visible: primitiveTypeComboBox.currentIndex == 2

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        spacing: Units.pt(36) 

                        Text {
                            Layout.alignment: Qt.AlignLeft
                            Layout.leftMargin: -Units.pt(5)                
                            text: qsTr("Coordinates of the Cube Center")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "black"
                        }

                        Text {               
                            text: qsTr("Property")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "black"
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: xTextField_Cube
                            placeholderText: "X Coordinate"
                            validator: DoubleValidator{}
                        }

                        TextField {
                            Layout.leftMargin: Units.pt(102)
                            id: length_Cube
                            placeholderText: "Length of Side"
                            validator: IntValidator {bottom: 1}
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: yTextField_Cube
                            placeholderText: "Y Coordinate"
                            validator: DoubleValidator{}
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: Units.pt(6)

                        TextField {
                            id: zTextField_Cube
                            placeholderText: "Z Coor.( Default 0 )"
                            validator: DoubleValidator{}
                        }
                    }
                }

                ColumnLayout {
                    spacing: Units.pt(2)
                    Layout.leftMargin: Units.pt(10)
                    Layout.topMargin: -Units.pt(18)
                    visible: primitiveTypeComboBox.currentIndex == 3

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.preferredHeight: Units.pt(24)
                        Layout.topMargin: -Units.pt(5)
                        spacing: Units.pt(36) 

                        Text {
                            Layout.alignment: Qt.AlignLeft
                            Layout.leftMargin: -Units.pt(8)                
                            text: qsTr("Properties of the Triangular Prism")
                            font { pointSize: 13; weight: Font.Bold }
                            color: "black"
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.bottomMargin: -Units.pt(4)

                        ColumnLayout{
                            Layout.fillWidth: true
                            Layout.leftMargin: Units.pt(10)


                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: Units.pt(24)

                                TextField {
                                    id: xTextField_Prism
                                    placeholderText: "X Coordinate"
                                    validator: DoubleValidator{}
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: Units.pt(24)
                                // Layout.topMargin: Units.pt(6)

                                TextField {
                                    id: yTextField_Prism
                                    placeholderText: "Y Coordinate"
                                    validator: DoubleValidator{}
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: Units.pt(24)
                                // Layout.topMargin: Units.pt(6)

                                TextField {
                                    id: zTextField_Prism
                                    placeholderText: "Z Coor.( Default 0 )"
                                    validator: DoubleValidator{}
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: Units.pt(24)
                                // Layout.topMargin: Units.pt(6)

                                TextField {
                                    id: length_Prism
                                    placeholderText: "Length"
                                    validator: IntValidator {bottom: 1}
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: Units.pt(24)

                                TextField {
                                    id: width_Prism
                                    placeholderText: "Width"
                                    validator: IntValidator {bottom: 1}
                                }
                            }                                                      
                        }

                        ColumnLayout{
                            Layout.fillWidth: true
                            Layout.leftMargin: Units.pt(70)

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: Units.pt(24)
                                Layout.topMargin: -Units.pt(1)

                                TextField {
                                    id: height_Prism
                                    placeholderText: "Height"
                                    validator: DoubleValidator{}
                                }
                            }

                            ColumnLayout {
                                Layout.fillWidth: true

                                RowLayout {
                                    Layout.fillWidth: true
                                    Layout.preferredHeight: Units.pt(24)
                                    Layout.topMargin: Units.pt(6) 
                                    spacing: Units.pt(6) 

                                    Text {
                                        Layout.alignment: Qt.AlignLeft
                                        Layout.leftMargin: Units.pt(6)                
                                        text: qsTr("Effect Type")
                                        font { pointSize: 13; weight: Font.Bold }
                                        color: "black"
                                    }

                                    ComboBox {
                                        id: effectTypeComboBox
                                        model: ["Only X", "Only Y", "Both"]
                                        currentIndex: 0
                                    }
                                }

                                CheckBox {
                                    id: checkBox_x
                                    text: "Rising along the positive x_axis" 
                                    checked: true
                                    enabled: (effectTypeComboBox.currentIndex == 0)? true : (effectTypeComboBox.currentIndex == 2)? true : false

                                    onClicked: {
                                        ////The two options are mutually exclusive
                                        // if( effectTypeComboBox.currentIndex == 0 ) {
                                        //     if( checkBox_y.checked ) {
                                        //         checkBox_x.checked = true
                                        //         x_rise = true
                                        //         checkBox_y.checked = false
                                        //         y_rise = false
                                        //     }
                                        // } 
                                        // if( effectTypeComboBox.currentIndex == 1 )  {
                                        //     x_rise = !x_rise
                                        //     // checkBox_x.checked = !checkBox_x.checked
                                        // } 
                                        x_rise = !x_rise                                  
                                    }
                                }

                                CheckBox {
                                    id: checkBox_y
                                    text: "Rising along the positive y_axis" 
                                    checked: false
                                    enabled: (effectTypeComboBox.currentIndex == 1)? true : (effectTypeComboBox.currentIndex == 2)? true : false

                                    onClicked: {
                                        y_rise = !y_rise                                         
                                    }
                                }
                            }
                        }
                    }
                }

                RowLayout  {
                    id: warnings
                    Layout.fillWidth: true
                    Layout.topMargin: -Units.pt(2) 
                    Layout.bottomMargin: -Units.pt(2) 
                    Layout.rightMargin: Units.pt(5)
                    Layout.preferredHeight: Units.pt(16)
                    visible: false                    

                    Text {
                        Layout.alignment: Qt.AlignLeft                
                        text: qsTr("Please enter all necessary data")
                        color: "red"
                    }
                }
                
            }
        }
    }

    onApplied: {     
        var typeIndex = primitiveTypeComboBox.currentIndex
        modepub = primitiveTypeComboBox.currentIndex
        root.add()

        if( typeIndex == 0 ) {
            if ( !xTextField_Sphere.text || !yTextField_Sphere.text || !radius_Sphere.text ) {
                warnings.visible = true
                return
            }
            var pri_mode = mode(typeIndex)
            var newpoint_position = position(typeIndex)
            var pri_size = psize(typeIndex)
            polygonpointTool.tool.primitive(pri_mode, newpoint_position, pri_size)  
            init()
            xpub = xTextField_Sphere.text
            ypub = yTextField_Sphere.text
            zpub = (zTextField_Sphere.text) || 0
            radiuspub = radius_Sphere.text
            clear_Sphere()
            close()
        }

        if( typeIndex == 1 ) {
            if ( !xTextField_Cuboid.text || !yTextField_Cuboid.text || !length_Cuboid.text || !width_Cuboid.text || !height_Cuboid.text ) {
                warnings.visible = true
                return
            }
            var pri_mode = mode(typeIndex)
            var newpoint_position = position(typeIndex)
            var pri_size = psize(typeIndex)
            polygonpointTool.tool.primitive(pri_mode, newpoint_position, pri_size)  
            init()
            xpub = xTextField_Cuboid.text
            ypub = yTextField_Cuboid.text
            zpub = (zTextField_Cuboid.text) || 0
            lengthpub = length_Cuboid.text
            widthpub = width_Cuboid.text
            heightpub = height_Cuboid.text
            clear_Cuboid()
            close()
        }

        if( typeIndex == 2 ) {
            if ( !xTextField_Cube.text || !yTextField_Cube.text || !length_Cube.text ) {
                warnings.visible = true
                return
            }
            var pri_mode = mode(typeIndex)
            var newpoint_position = position(typeIndex)
            var pri_size = psize(typeIndex)
            polygonpointTool.tool.primitive(pri_mode, newpoint_position, pri_size)  
            init()
            xpub = xTextField_Cube.text
            ypub = yTextField_Cube.text
            zpub = (zTextField_Cube.text) || 0
            lengthpub = length_Cube.text
            clear_Cube()
            close()
        }

        if( typeIndex == 3 ) {
            if ( !xTextField_Prism.text || !yTextField_Prism.text || !length_Prism.text || !width_Prism.text || !height_Prism.text ) {
                warnings.visible = true
                return
            }
            var pri_mode = mode(typeIndex)
            var newpoint_position = position(typeIndex)
            var pri_size = psize(typeIndex)
            polygonpointTool.tool.primitive(pri_mode, newpoint_position, pri_size)  
            init()
            xpub = xTextField_Prism.text
            ypub = yTextField_Prism.text
            zpub = (zTextField_Prism.text) || 0
            lengthpub = length_Prism.text
            widthpub = width_Prism.text
            winkelpub = height_Prism.text
            clear_Prism()
            close()
        }
        warnings.visible = false
    }

    function position(typeIndex) {
        var newpoint_position = []
        if( typeIndex == 0 ) {
            newpoint_position.push(parseFloat(xTextField_Sphere.text)) 
            newpoint_position.push(parseFloat(yTextField_Sphere.text))
            newpoint_position.push(parseFloat(zTextField_Sphere.text) || 0)
        } else if( typeIndex == 1 ) {
            newpoint_position.push(parseFloat(xTextField_Cuboid.text)) 
            newpoint_position.push(parseFloat(yTextField_Cuboid.text))
            newpoint_position.push(parseFloat(zTextField_Cuboid.text) || 0)
        } else if( typeIndex == 2 ) {
            newpoint_position.push(parseFloat(xTextField_Cube.text)) 
            newpoint_position.push(parseFloat(yTextField_Cube.text))
            newpoint_position.push(parseFloat(zTextField_Cube.text) || 0)
        } else if( typeIndex == 3 ) {
            newpoint_position.push(parseFloat(xTextField_Prism.text)) 
            newpoint_position.push(parseFloat(yTextField_Prism.text))
            newpoint_position.push(height_Prism.text)
        }   
        return newpoint_position
    }

    function mode(typeIndex) {
        var pri_mode = []
        pri_mode.push(typeIndex)
        if( typeIndex == 0 ) {
            pri_mode.push(0) 
            pri_mode.push(radius_Sphere.text) 
        } else if( typeIndex == 1 ) {
            pri_mode.push(0)
            pri_mode.push(0)
        } else if( typeIndex == 2 ) {
            pri_mode.push(0) 
            pri_mode.push(0) 
        } else if( typeIndex == 3 ) {
            pri_mode.push(effectTypeComboBox.currentIndex)
            pri_mode.push((x_rise)? 1 : 0)
        }  
        return pri_mode
    }

    function psize(typeIndex) {
        var pri_size = []
        if( typeIndex == 0 ) {
            pri_size.push(0) 
            pri_size.push(0) 
            pri_size.push(0)
        } else if( typeIndex == 1 ) {
            pri_size.push(length_Cuboid.text) 
            pri_size.push(width_Cuboid.text)
            pri_size.push(height_Cuboid.text)
        } else if( typeIndex == 2 ) {
            pri_size.push(length_Cube.text) 
            pri_size.push(0) 
            pri_size.push(0)
        } else if( typeIndex == 3 ) {
            pri_size.push(length_Prism.text) 
            pri_size.push(width_Prism.text)
            pri_size.push((y_rise)? 1 : 0)
        }  
        return pri_size
    }
}
