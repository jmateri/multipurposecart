#include <QCoreApplication>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
int main(int argc, char *argv[])
{
    QString portName;
    QCoreApplication a(argc, argv);

    QList<QSerialPortInfo> info = QSerialPortInfo::availablePorts();

    for(int i=0; i<info.length(); i++)
    {
        qDebug() << info.at(i).portName();
        portName = info.at(i).portName();
    }

    QSerialPort *port = new QSerialPort();

    qDebug() << "Configuring SerialPort" ;
    port->setPortName(portName);
    if(port->isOpen())
    {
        port->close();
    }
    qDebug() << "Open: " << port->open(QIODevice::WriteOnly);
    qDebug() << "SetBaudRate: " << port->setBaudRate(QSerialPort::Baud9600);
    qDebug() << "SetDataBits: " << port->setDataBits(QSerialPort::Data8);
    qDebug() << "SetParity: " << port->setParity(QSerialPort::NoParity);
    qDebug() << "SetStopBits: " << port->setStopBits(QSerialPort::OneStop);
    qDebug() << "SetFlowControl: " << port->setFlowControl(QSerialPort::NoFlowControl);

    QByteArray array;
    array.append(255);



    qDebug() << "Bytes written: " << port->write(array);
   // qDebug() << "Data written: " << QString::number(value).toStdString().c_str();
    qDebug() << "Flush: " << port->flush();

    port->close();
    port->waitForBytesWritten(1000);
    delete port;
    qDebug() << "Port Closed";
//    char tempChar = 'a';
//    port.close();
//    port.open(QIODevice::ReadOnly);


//        if(port.waitForReadyRead(5000))
//        {
//            port.read(&tempChar,1);
//            qDebug() << tempChar;
//        }


//    port.close();

    qDebug() << "End";
    return a.exec();
}
