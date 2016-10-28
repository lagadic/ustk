/****************************************************************************
 *
 * Author:
 * Marc Pouliquen
 *
 *****************************************************************************/



#include <visp3/core/vpConfig.h>

#include <iostream>

#include <visp3/core/vpDebug.h>

#include <visp3/ustk_gui/usDisplay2D.h>
#include <visp3/ustk_io/usImageIo.h>

#include <string>
#include <vector>

/* -------------------------------------------------------------------------- */
/*                               MAIN FUNCTION                                */
/* -------------------------------------------------------------------------- */

int main(int argc, const char** argv)
{
    std::cout << "usage : ./tesUsDisplay2D /path/to/usImagePostscan2d" << std::endl;
    if (argc != 2) {
      std::cout << "wrong number of arguments" << std::endl;
      return 1;
    }

    usImagePostScan2D<unsigned char> postscan2D;


    usImageIo::read(postscan2D, argv[1]);
    std::cout << "image read" << std::endl;

    usDisplay2D<usImagePostScan2D<unsigned char> > *display2D;
    display2D = new usDisplay2D<usImagePostScan2D<unsigned char> >;

    std::cout << "display created" << std::endl;
    display2D->setImage(postscan2D);
    std::cout << "image set" << std::endl;
    display2D->display();

    // Wait for a click in the display window
    std::cout << "Wait for a button click..." << std::endl;
    display2D->waitClick();
    delete display2D;

    return 0;
}

