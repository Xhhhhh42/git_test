import QtQuick 2.12
import Hector.Style 1.0
import Hector.Utils 1.0

Rectangle {
    id: timeRectangle
    // x: Units.pt(24)
    y: Units.pt(42)
    z: 10
    color: Style.background.container
    width: Units.pt(48)
    height: Units.pt(24)
    anchors.right: parent.right


    AutoSizeText {
        function getTime() {
            var date = new Date()
            var hours = date.getHours()
            var minutes = date.getMinutes()
            return (hours < 10 ? "0" + hours : hours) + ":" + (minutes < 10 ? "0" + minutes : minutes)
        }

        id: clockText
        color: Style.getTextColor(timeRectangle.color)
        text: getTime()

        Timer {
            interval: 10000; running: true; repeat: true
            onTriggered: clockText.text = clockText.getTime()
        }
    }
}
