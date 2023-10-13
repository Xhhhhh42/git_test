import QtQuick 2.12
import QtQuick.Controls 2.2
import Hector.Utils 1.0
import Hector.Style 1.0

Button {
    id: control
    property var style: Style.activeStyle
    property var buttonStyle: Style.buttonStyle(style)
    property bool isClicked: false

    padding: Units.pt(2)
    font: Style.button.font
    hoverEnabled: true

    contentItem: Text {
        text: control.text
        font: control.font
        leftPadding: control.leftPadding
        rightPadding: control.rightPadding
        topPadding: control.topPadding
        bottomPadding: control.bottomPadding
        opacity: enabled ? 1 : 0.3
        color: control.flat ? (control.checkable ? (control.checked ? control.buttonStyle.checkedColor : control.buttonStyle.uncheckedColor) : control.buttonStyle.color)
                            : Style.getTextColor(control.background.color)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }

    background: Rectangle {
        implicitWidth: Units.pt(50)
        implicitHeight: Style.button.defaultHeight
        visible: !control.flat
        opacity: enabled ? 1 : 0.3
        color: handleMouseStates(Style.background.container) 
    }

    function handleMouseStates(baseColor)
    {
        var color = baseColor       
        if (control.isClicked) return control.buttonStyle.color
        else if (control.hovered) return Qt.darker(color, 1.15)
        return color
    }

}

