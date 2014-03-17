#include <qapplication.h>
#include "mainwindow.h"

int main ( int argc, char **argv )
{
    QApplication a( argc, argv );

    MainWindow w;
#if 0
    w.resize( 1000, 800 );
#endif
    w.show();

    return a.exec();
}
