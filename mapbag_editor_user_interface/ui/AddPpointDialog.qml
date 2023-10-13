import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.1
// import QtQuick.Dialogs 1.3
import Hector.Utils 1.0


Dialog {
    id: root

    property var polygonpointTool

    width: Units.pt(300)
    height: Units.pt(200)
    standardButtons: Dialog.Apply | Dialog.Cancel
    title: "Add new Polygon point"
    closePolicy: Popup.NoAutoClose
    parent: ApplicationWindow.overlay
    x: parent.x + (parent.width - width) / 2
    y: parent.y + (parent.height - height) / 2
    modal: true
    focus: true

    onRejected: {
        if ( xTextField.text || yTextField.text || zTextField.text ) {
            confirmationDialog.open()
            open()
        }
    }

    onApplied: {
        if ( !xTextField.text || !yTextField.text || !zTextField.text ) return
        var newpoint_position = {}
        newpoint_position.x = xTextField.text
        newpoint_position.y = yTextField.text
        newpoint_position.z = zTextField.text
        root.polygonpointTool.tool.addPolygonpoint(newpoint_position.x , newpoint_position.y , newpoint_position.z )
        root.polygonpointTool.tool.polygonpointsChanged()
        xTextField.clear()
        yTextField.clear()
        zTextField.clear()
        close()
    }

    ColumnLayout {
        anchors.fill: parent

        Text {
            Layout.fillWidth: true
            text: "Please enter the coordinates (e.g. 1.00) :"
        }

        TextField {
            id: xTextField
            Layout.fillWidth: true
            placeholderText: qsTr("X ...")
            validator: DoubleValidator{}
        }

        TextField {
            id: yTextField
            Layout.fillWidth: true
            placeholderText: qsTr("Y...")
            validator: DoubleValidator{}
        }

        TextField {
            id: zTextField
            Layout.fillWidth: true
            placeholderText: qsTr("Z...")
            validator: DoubleValidator{bottom: 0}
        }       
    }  

    ConfirmationDialog {
        id: confirmationDialog
    }
}