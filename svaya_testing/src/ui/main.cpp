#include <svaya_testing/ui/svayaui.h>
#include <QApplication>
//#include "main.moc"

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);

    QApplication a(argc, argv);
    SvayaUi ui;
    ui.show();

    return a.exec();


}
