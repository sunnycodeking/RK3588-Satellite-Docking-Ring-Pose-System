#include <QApplication>
#include <QFont>

#include "app/AppShell.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QFont font = app.font();
    font.setPointSize(10);
    app.setFont(font);

    AppShell shell;
    shell.resize(1400, 860);
    shell.show();

    return app.exec();
}
