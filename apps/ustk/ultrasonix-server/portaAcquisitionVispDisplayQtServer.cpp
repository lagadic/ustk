//std includes
#include <iostream>


//Visp includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpException.h>




//Qt includes
#include <QtWidgets/QApplication>

//local includes
#include "usNetworkServer.h"



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    usNetworkServer w;
    
    return a.exec();
}


