#include "usercomm.h"
#include <iostream>
#include <QMessageBox>

/*
 * PROTOCOL:
 * HEADER | LEN | DATA                        | CRC
 *   1B   | 1B  |  DATA[0]:CMD, DATA[1]->[n]  | 1B
 */

/* Util functions */
uint8_t
user_checkFlag(uint32_t *status,uint32_t flag)
{
   return (*status) & flag;
}

void
user_setFlag(uint32_t *status, uint32_t flag)
{
   *status |= flag;
}

void
user_clearFlag(uint32_t *status, uint32_t flag)
{
    *status &= (flag ^ 0xFFFFFFFF);
}
const uint8_t crc8_table[256] =
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};
uint8_t
UserComm::mlsUtilsCRC8Calculate(void *buffIn, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t *data = (uint8_t *)buffIn;
    while(len--)
    {
        crc = crc8_table[crc ^ *data];
        data += 1;
    }

    return crc;
}
void
UserComm::reset_uart_controller()
{
    payload_it_ = payload_;
    rcv_crc_ = 0xFF;
    len_ = 0;
    rcvLen_ = 0;
    state_ = PACKET_HEADER;
    status_ = 0x00;
}
/*** Util functions ***/

UserComm::UserComm(QObject *parent) : QObject(parent)
                                    , payload_it_(payload_)
                                    , rcv_crc_(0xFF)
                                    , len_(0)
                                    , rcvLen_(0)
                                    , state_(PACKET_HEADER)
                                    , status_(0x0000)
{
    sp_.setPortName("ttyUSB0");
    sp_.setBaudRate(921600);
    sp_.setDataBits(QSerialPort::Data8);
    sp_.setParity(QSerialPort::NoParity);
    sp_.setStopBits(QSerialPort::OneStop);
    sp_.setFlowControl(QSerialPort::NoFlowControl);

    connect(&sp_, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
            this, &UserComm::handleError);
    connect(&sp_, &QSerialPort::readyRead, this, &UserComm::readData);
}

UserComm::~UserComm()
{

}

void
UserComm::openSerialPort()
{
    std::cout << "usercomm " << __FUNCTION__ << std::endl;
    if(sp_.open(QIODevice::ReadWrite))
    {
        emit(connected());
    }
    else
    {
        emit(errorSig(tr("Can't open %1, error code %2")
                   .arg(sp_.portName()).arg(sp_.error())));
    }
}

void
UserComm::closeSerialPort()
{
   std::cout << "usercomm " << __FUNCTION__ << std::endl;
   if(sp_.isOpen())
   {
       sp_.close();
       emit(disconnected());
   }
}

//void
//UserComm::writeData(const QByteArray &data)
//{
//    if(sp_.isOpen())
//    {
//        std::cout << "usercomm " << __FUNCTION__ << std::endl;
//        sp_.write(data);
//    }
//    else emit(errorSig("Port is not opened yet"));
//}
void
UserComm::writeData(uint8_t *pData, uint8_t len)
{
    if(pData)
    {
        if(sp_.isOpen())
        {
            std::cout << "usercomm " << __FUNCTION__ << std::endl;
            if(len > MAX_DATA_LENGTH)
            {
                len = MAX_DATA_LENGTH;
                emit(errorSig("Excessive length (Max:128B)"));
            }

            // Add protocol
            uint8_t buf[256];
            buf[0] = HEADER_APP;
            buf[1] = len;
            for(int i = 0; i < len; i++)
                buf[2+i] = pData[i];
            buf[2+len] = mlsUtilsCRC8Calculate(pData,len);

            QByteArray qBuf_ = QByteArray((char*)buf,len+OVERLOAD_SIZE);
            sp_.write(qBuf_);
            // push all the data immediately
            // since Qt UART driver tends to wait and push data later
            sp_.flush();
        }
        else emit(errorSig("Port is not opened yet"));

        free(pData);
    }
}

void
UserComm::readData()
{
    std::cout << "usercomm " << __FUNCTION__ << std::endl;

    QByteArray data = sp_.readAll();
    // Somehow not all data is read, must wait
    while(sp_.waitForReadyRead(UART_TIMEOUT_MS))
        data += sp_.readAll();

    for(auto d : data)
        uart_rx_handler(d);

    if(user_checkFlag(&status_,UART_CHECKCRC))
    {
       uint8_t calCrc = mlsUtilsCRC8Calculate(payload_,rcvLen_);
       if(calCrc == rcv_crc_)
           emit(dataReceived(data));
    }

    reset_uart_controller();
}

void
UserComm::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError)
    {
        emit(errorSig(tr("Critical Error: %1").arg(sp_.errorString())));
        closeSerialPort();
    }
}

void
UserComm::uart_rx_handler(uint8_t d)
{
    switch(state_)
    {
    case PACKET_HEADER:
    {
        if(d == HEADER_STM32)
           {
               state_++;
           }
    }
        break;
    case PACKET_LEN:
    {
        if(d > MAX_DATA_LENGTH)
        {
            state_ = PACKET_HEADER;
            status_ |= UART_EXCEEDLENGTH;
        }
        else
        {
            len_ = d;
            rcvLen_ = len_;
            state_++;
        }
    }
        break;
    case PACKET_DATA:
    {
        *payload_it_++ = d;
        if(--len_) {}
        else state_++;
    }
        break;
    case PACKET_CRC:
    {
        rcv_crc_ = d;
        status_ |= UART_CHECKCRC;

        //Reset structure
        state_ = PACKET_HEADER;
        payload_it_ = payload_;
    }
        break;
    default:
        break;
    }
}
