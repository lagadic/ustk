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

import QtQuick 2.6
import QtQuick.Controls 1.4

Item {
    width: parent.width
    height: parent.height

    Row {
        anchors { 
            horizontalCenter: parent.horizontalCenter; 
            bottom: parent.bottom; 
            bottomMargin: 15 
        }
        spacing: 15
        Button {
            text: "Button1"
            onClicked: {
                console.log("Button1 CLICKED!")
            }
        }
        Button {
            text: "Button2"
            onClicked: {
                console.log("Button2 CLICKED!")
            }
        }
        Button {
            text: "Button3"
            onClicked: {
                console.log("Button3 CLICKED!")
            }
        }
    }
}
