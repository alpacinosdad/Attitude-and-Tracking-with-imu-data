#include "widget.h"

#include <QApplication>
#include <QFont>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QFont font(QStringLiteral("Tahoma"));
    font.setPixelSize(18);
    qApp->setFont(font);

    Widget w;
    w.show();

    return a.exec();
}