import QtQuick 2.12
import QtQuick.Controls 2.5
import Hector.Style 1.0
import Hector.Utils 1.0
import Hector.Icons 1.0

RoundButton {
  anchors.right: parent.right
  anchors.top: parent.top
  anchors.margins: Units.pt(8)
  padding: 0
  width: Units.pt(24)
  height: Units.pt(24)

  Text {
    anchors.centerIn: parent
    font.family: HectorIcons.fontFamily
    font.pointSize: 16
    text: rviz.isFullscreen ? HectorIcons.exitFullscreen : HectorIcons.fullscreen
    color: "#ffffff"
  }

  background: Rectangle {
    anchors.fill: parent
    radius: width / 2
    color: "#444444"
    opacity: parent.down ? 1 : parent.hovered ? 0.8 : 0.6
  }

  onClicked: rviz.isFullscreen = !rviz.isFullscreen
}


