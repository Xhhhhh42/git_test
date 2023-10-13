import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.1
import Hector.Utils 1.0


Dialog {
    id: root

    property var polygonpointTool
    property string dialogtext
    property bool confirmation
    signal confirmed

    width: Units.pt(200)
    height: Units.pt(110)
    standardButtons: Dialog.Yes | Dialog.Cancel
    // title: "Confirm the action"
    closePolicy: Popup.NoAutoClose
    parent: ApplicationWindow.overlay
    x: parent.x + (parent.width - width) / 2
    y: parent.y + (parent.height - height) / 2
    z: 2
    modal: true
    focus: true

    onRejected: {
    }

    // onAccepted: {
    //     root.confirmed()
    //     // root.confirmation = true        
    //     close()
    // }

    Text {
        width: parent.width
        wrapMode: Text.WordWrap
        // text: "Do you really want to delete all the data ? "
        text: dialogtext
    } 
}

