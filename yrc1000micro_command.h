#ifndef YRC1000MICRO_COMMAND_H
#define YRC1000MICRO_COMMAND_H

#include <QObject>
#include <QByteArray>

#define HEADER_SIZE             32  //32 byte header

#define CMD_ID_SERVO_ON                     '\x83'      // command RSTATS_Data2_6
#define CMD_ID_READ_ROBOT_POS               '\x75'
#define CMD_ID_MOVE_ROBOT_CARTESIAN         '\x8A'
#define CMD_ID_MOVE_ROBOT_PULSE             '\x8B'
#define CMD_ID_STATUS_READING               '\x72'
#define CMD_ID_JOB_SELECT                   '\x87'
#define CMD_ID_JOB_START                    '\x86'
#define CMD_ID_GET_VAR_POSITION             '\x7F'
#define CMD_ID_GET_AND_WRITE_BYTE           '\x7A'


#define CMD_DATA_SIZE           6
#define CMD_PROCESS_DIVTION     9
#define CMD_ACK                 10
#define CMD_REQUEST_ID          11
#define CMD_BLOCK_NUM           12
#define CMD_ID_ADDRESS          24
#define CMD_INSTANCE            26
#define CMD_ATTRIBUTE           28
#define CMD_SERVICE             29

#define RES_STATUS              25
#define RES_STATUS_SIZE         26
#define RES_STATUS_ERROC        28

#define DATA_BYTE0              32
#define DATA_BYTE1              33
#define DATA_BYTE2              34
#define DATA_BYTE3              35

#define DATA_X_AXIS             52
#define DATA_Y_AXIS             56
#define DATA_Z_AXIS             60
#define DATA_ROLL_ANGLE         64
#define DATA_PITCH_ANGLE        68
#define DATA_YAW_ANGLE          72

#define DATA_MOVE_ROBOT_NUMBER      0
#define DATA_MOVE_STATION_NUM       4
#define DATA_MOVE_SPEED_TYPE        8
#define DATA_MOVE_SPEED             12
#define DATA_MOVE_COORDINATE        16
#define DATA_MOVE_X_CARTESIAN       20
#define DATA_MOVE_Y_CARTISIAN       24
#define DATA_MOVE_Z_CARTESIAN       28
#define DATA_MOVE_ROLL_CARTESIAN    32
#define DATA_MOVE_PITCH_CARTESIAN   36
#define DATA_MOVE_YAW_CARTESIAN     40
#define DATA_MOVE_USER_COORDINATE   64

#define CMD_HEADER_ACK_REQUEST              '\x00'
#define CMD_HEADER_ACK_NOT_REQUEST          '\x01'
#define CMD_HEADER_PROCESS_ROBOT_CONTROL    '\x01'
#define CMD_HEADER_PROCESS_FILE_CONTROL     '\x02'

#define CMD_HEADER_SERVO                    '\x83'
#define CMD_HEADER_SERVO_DATA_SIZE          '\x04'
#define CMD_HEADER_SERVO_INSTANCE_HOLD      0x01
#define CMD_HEADER_SERVO_INSTANCE_ON        0x02
#define CMD_HEADER_SERVO_INSTANCE_HLOCK     0x03
#define CMD_HEADER_SERVO_ATTRIBUTE          0x01
#define CMD_HEADER_SERVO_SERVICE            0x10

#define CMD_HEADER_READ_POS_DATA_SIZE               '\x00'
#define CMD_HEADER_READ_POS_INSTANCE_CARTESIAN      '\x65'
#define CMD_HEADER_READ_POS_INSTANCE_PULSE          '\x01'
#define CMD_HEADER_READ_POS_ATTRIBUTE_DATA_TYPE     '\x01'
#define CMD_HEADER_READ_POS_ATTRIBUTE_ALL           '\x00'
#define CMD_HEADER_READ_POS_ATTRIBUTE_6TH_AXIS      '\x0B'
#define CMD_HEADER_READ_POS_SERVICE_SINGLE          '\x0E'
#define CMD_HEADER_READ_POS_SERVICE_ALL             '\x01'

#define CMD_HEADER_WRITE_HOME_PULSE_ATTRIBUTE       01
#define CMD_HEADER_WRITE_HOME_PULSE_SERVICE         '\x02'

#define CMD_CONTROL_GROUP_ROBOT         01
#define CMD_CONTROL_GROUP_STATION       00
#define CMD_CLASSIFICATION_IN_SPEED     00

#define CMD_HEADER_MOVE_CARTESIAN_DATA_SIZE             '\x68'
#define CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE          '\x01'
#define CMD_HEADER_MOVE_INSTANCE_STRAIGHT_ABSOLUTE      '\x02'
#define CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT     '\x03'
#define CMD_HEADER_MOVE_ATTRIBUTE                       '\x01'
#define CMD_HEADER_MOVE_SERVICE_ALL                     '\x02'

#define CMD_DATA_SERVO_ON           0x01
#define CMD_DATA_SERVO_OFF          0x02

#define CMD_DATA_MOVE_SPEED_TYPE_LINK           '\x00'
#define CMD_DATA_MOVE_SPEED_TYPE_V_SPEED        '\x01'
#define CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED       '\x02'
#define CMD_DATA_MOVE_COORDINATE_BASE           '\x10'
#define CMD_DATA_MOVE_COORDINATE_ROBOT          '\x11'
#define CMD_DATA_MOVE_COORDINATE_USER           '\x12'
#define CMD_DATA_MOVE_COORDINATE_TOOL           '\x13'

#define RES_VALUE_SERVO_STATUS_NORMAL           '\x00'
#define RES_VALUE_SERVO_STATUS_SIZE_ONE_WORD    0x01
#define RES_VALUE_SERVO_STATUS_SIZE_TWO_WORD    0x02

#define READ_POSITION_PULSE     0
#define READ_POSITION_XYZ       1

#define CMD_REQUEST_ID_ON_SERVO         '\x00'
#define CMD_REQUEST_ID_OFF_SERVO        '\x01'
#define CMD_REQUEST_ID_GET_POSITION     '\x02'
#define CMD_REQUEST_ID_GET_PULSE        '\x03'
#define CMD_REQUEST_ID_WRITE_POSITION   '\x04'
#define CMD_REQUEST_ID_WRITE_PULSE      '\x05'
#define CMD_REQUEST_ID_SELECT_JOB       '\x06'
#define CMD_REQUEST_ID_START_JOB        '\x07'
#define CMD_REQUEST_ID_HOME_POS         '\x08'
#define CMD_REQUEST_ID_SAVE_FILE        '\x09'
#define CMD_REQUEST_ID_LOAD_FILE        '\x0A'
#define CMD_REQUEST_ID_DELETE_FILE      '\X0B'
#define CMD_REQUEST_ID_GET_VARPOS       '\x0C'
#define CMD_REQUEST_ID_WRITE_VARPOS     '\x0D'
#define CMD_REQUEST_ID_WRITE_BYTE       '\x0E'
#define CMD_REQUEST_ID_GET_BYTE         '\x0F'


class YRC1000micro_command : public QObject
{
private:
    QByteArray *header_to_send;
    QByteArray data_to_send;
    QByteArray move_cartesian_data;
    QByteArray position_variable_data;

    struct TxDataWriteVarPosition;

    Q_OBJECT
public:
    explicit YRC1000micro_command(QObject *parent = nullptr);
    ~YRC1000micro_command();

    QByteArray setServoOn();
    QByteArray setServoOff();

    QByteArray readRobotPosition();
    QByteArray readRobotPulse();

    void initRobot(quint8 robot_number);

    QByteArray setRobotPositionCartesian (quint8 coordinate, quint8 move_type, quint8 speed_type,
                                         double speed, QVector<double>* position);

    QByteArray HomePos();

    void printCMD(QByteArray cmd);

    QByteArray selectJob(char *jobName);
    QByteArray startJob();
    QByteArray getVarPosition(uint16_t index);
    QByteArray writeVarPosition(uint16_t index, QVector<int32_t> pos);

    QByteArray getByte(uint16_t index);
    QByteArray writeByte(uint16_t index, uint8_t data);
    void printTxDataWriteVarPosition(const YRC1000micro_command::TxDataWriteVarPosition &pos);
    QByteArray writeVarPositionAgain(uint16_t index, QVector<double>* position);

signals:

public slots:
};

#endif // YRC1000MICRO_COMMAND_H
