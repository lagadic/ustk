/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

import QtQuick 2.5
// import QtQuick.Controls 2.1 // Not working on lunar with Ubuntu 16.04
import QtQuick.Controls 1.4
Item {
  width: 400
  height: 200
  property var selection: undefined
  property bool userSelection: true
  property bool trackingActive: false
  property bool servoingActive: false

  signal startTracking()
  signal stopTracking()
  signal startServoing()
  signal stopServoing()

  MouseArea {
    anchors.fill: parent
    enabled: userSelection
    onClicked: {
      if(!selection)
        selection = selectionComponent.createObject(parent, {"x": parent.width / 2 - 50, "y": parent.height / 2 - 50, "width": 100, "height": 100})
    }
  }
  Item {
    height: 40
    width: 400
    anchors {
      horizontalCenter: parent.horizontalCenter;
      bottom: parent.bottom;
      bottomMargin: 15
    }
    Button {
      id: trackingButton

      opacity: 1
      text: qsTr("Start tracking")
      width: Text.width
      anchors.left: parent.left
      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.rightMargin: 10

      // Removed to make it working on lunar with Ubuntu 16.04
      /* contentItem: Text {
        text: trackingButton.text
        font: trackingButton.font
        opacity: enabled ? 1.0 : 0.3
        color: "steelblue"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
      } */


      /* background: Rectangle {
        implicitWidth: 100
        implicitHeight: 40
        opacity: enabled ? 1 : 0.3
        color: trackingButton.down ? "#d9d9d9" : "white"
        border.width: 1
        radius: 4
      }
*/
      onClicked: {
        if(selection) {
          if(!trackingActive) {
            startTracking()
            userSelection = false
            trackingActive = true
            trackingButton.text = qsTr("Stop tracking")
          }
          else {
            stopTracking()
            userSelection = true
            trackingActive = false
            trackingButton.text = qsTr("Start tracking")
            if(servoingActive) {
              stopServoing()
              servoingActive = false
              switchServoing.checked = false
            }
          }
        }
      }
    }
    Rectangle {
      id: rectServoing
      width: 200
      visible: trackingActive

      anchors.left: trackingButton.right
      anchors.top: parent.top
      anchors.bottom: parent.bottom

      anchors.leftMargin: 10
      opacity: 1

      implicitHeight: 40
      border.width: 0
      radius: 4
      Text {
        id: switchText
        anchors.left: parent.left
        anchors.rightMargin: 10
        anchors.leftMargin: 10
        anchors.verticalCenter: parent.verticalCenter

        width: Text.width

        text: "Servoing activation"
        color: "steelblue"
      }
      Switch {
        id: switchServoing

        anchors.left:switchText.right
        anchors.right: parent.right

        onClicked: {
          if(checked) {
            startServoing()
            servoingActive = true
          }
          else {
            stopServoing()
            servoingActive = false
          }
        }
      }
    }


  }
  Component {
    id: selectionComponent

    Rectangle {
      id: selComp
      objectName: "selectionRectangle"

      border {
        width: 2
        color: "steelblue"
      }
      color: "#354682B4"

      property int rulersSize: 18

      MouseArea {     // drag mouse area
        anchors.fill: parent
        enabled: userSelection
        drag{
          target: parent
          minimumX: 0
          minimumY: 0
          maximumX: parent.parent.width - parent.width
          maximumY: parent.parent.height - parent.height
          smoothed: true
        }

        onDoubleClicked: {
          parent.destroy()        // destroy component
        }
      }

      Rectangle { // left circle selector
        width: rulersSize
        height: rulersSize
        radius: rulersSize
        color: "steelblue"
        anchors.horizontalCenter: parent.left
        anchors.verticalCenter: parent.verticalCenter
        visible: userSelection

        MouseArea {
          anchors.fill: parent
          enabled: userSelection
          drag{ target: parent; axis: Drag.XAxis }
          onMouseXChanged: {
            if(drag.active){
              selComp.width = selComp.width - mouseX
              selComp.x = selComp.x + mouseX
              if(selComp.width < 30)
                selComp.width = 30
            }
          }
        }
      }

      Rectangle { // right circle selector
        width: rulersSize
        height: rulersSize
        radius: rulersSize
        color: "steelblue"
        anchors.horizontalCenter: parent.right
        anchors.verticalCenter: parent.verticalCenter
        visible: userSelection

        MouseArea {
          anchors.fill: parent
          enabled: userSelection
          drag{ target: parent; axis: Drag.XAxis }
          onMouseXChanged: {
            if(drag.active){
              selComp.width = selComp.width + mouseX
              if(selComp.width < 50)
                selComp.width = 50
            }
          }
        }
      }

      Rectangle {	// top circle selector
        width: rulersSize
        height: rulersSize
        radius: rulersSize
        x: parent.x / 2
        y: 0
        color: "steelblue"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.top
        visible: userSelection

        MouseArea {
          anchors.fill: parent
          enabled: userSelection
          drag{ target: parent; axis: Drag.YAxis }
          onMouseYChanged: {
            if(drag.active){
              selComp.height = selComp.height - mouseY
              selComp.y = selComp.y + mouseY
              if(selComp.height < 50)
                selComp.height = 50
            }
          }
        }
      }


      Rectangle { // bottom circle selector
        width: rulersSize
        height: rulersSize
        radius: rulersSize
        x: parent.x / 2
        y: parent.y
        color: "steelblue"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.bottom
        visible: userSelection

        MouseArea {
          anchors.fill: parent
          enabled: userSelection
          drag{ target: parent; axis: Drag.YAxis }
          onMouseYChanged: {
            if(drag.active){
              selComp.height = selComp.height + mouseY
              if(selComp.height < 50)
                selComp.height = 50
            }
          }
        }
      }
    }
  }
}
