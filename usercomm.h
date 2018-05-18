#ifndef USERCOMM_H
#define USERCOMM_H

#include <QObject>
#include <QSerialPort>

#define HEADER_STM32   					0xCC
#define HEADER_APP     					0xBB

#define MAX_DATA_LENGTH 				128
#define UART_TIMEOUT_MS					1


#define  UART_CHECKCRC 			    	0x0001
#define  UART_BADCRC            	    0x0002
#define  UART_EXCEEDLENGTH	            0x0004
#define  UART_NEEDRESTART		    	0x0008

#define  OVERLOAD_SIZE 					3
#define  TRUE 							1
#define  FALSE							0


#define	PACKET_HEADER 					0x00
#define	PACKET_LEN 						0x01
#define	PACKET_DATA 					0x02
#define	PACKET_CRC 						0x03

enum PROTOCOL_CMD
{
  CMD_MOVETOXYZ     = 0x00,
  CMD_GRAB          = 0x01,
  CMD_DROP          = 0x02,
  CMD_STOP          = 0x03,
  CMD_ERROR         = 0xFF,
};
/*
 * @brief: util function
 */
uint8_t
user_checkFlag(uint32_t *status,uint32_t flag);

void
user_setFlag(uint32_t *status, uint32_t flag);

void
user_clearFlag(uint32_t *status, uint32_t flag);
/*
 * @brief: util function
 */
class UserComm : public QObject
{
    Q_OBJECT
public:
    explicit UserComm(QObject *parent = nullptr);
    ~UserComm();

    void
    setPortname(const QString &portName){
        sp_.setPortName(portName);
    }
    void
    setBaudrate(const qint32 baudrate){
        sp_.setBaudRate(baudrate);
    }
    QString
    getPortname() {
        return sp_.portName();
    }
    qint32
    getBaudrate() {
        return sp_.baudRate();
    }
    bool
    isOpen(){
        return sp_.isOpen();
    }

    uint8_t
    mlsUtilsCRC8Calculate(void *buffIn, uint8_t len) ;

    void
    reset_uart_controller();

public slots:
    void
    openSerialPort();
    void
    closeSerialPort();
    void
    writeData(const QByteArray &data);

private slots:
    void
    readData();
    void
    handleError(QSerialPort::SerialPortError error);

signals:
    void
    dataReceived(const QByteArray &bytes);
    void
    errorSig (const QString &s);
    void
    connected();
    void
    disconnected();

private:
    void
    uart_rx_handler(uint8_t d);
    QSerialPort sp_;
    uint8_t payload_[MAX_DATA_LENGTH];
    uint8_t *payload_it_;
    uint8_t rcv_crc_;
    uint8_t len_;
    uint8_t rcvLen_;
    uint8_t state_;
    uint32_t status_;
};

#endif // USERCOMM_H
