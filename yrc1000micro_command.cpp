#include "yrc1000micro_command.h"
#include <QVector>
#include <QDebug>

YRC1000micro_command::YRC1000micro_command(QObject *parent) : QObject(parent)
{
    // Init header
    const char char_header[] = {'\x59','\x45','\x52','\x43','\x20','\x00','\x04',
                                '\x00','\x03','\x01','\x00','\x01','\x00','\x00','\x00','\x00','\x39','\x39',
                                '\x39','\x39','\x39','\x39','\x39','\x39','\x83','\x00','\x02','\x00','\x01',
                                '\x10','\x00','\x00'};

    this->header_to_send = new QByteArray(char_header, HEADER_SIZE);

    // Init data
    data_to_send.append('\x00');
    data_to_send.append('\x00');
    data_to_send.append('\x00');
    data_to_send.append('\x00');

    // Init move data
    for(int i=0;i<104;i++)
    {
        move_cartesian_data.append('\x00');
    }
    move_cartesian_data[DATA_MOVE_ROBOT_NUMBER]     = '\x01';
    move_cartesian_data[DATA_MOVE_USER_COORDINATE]  = '\x01';

    // Init move data
    for(int i=0;i<52;i++)
    {
        position_variable_data.append('\x00');
    }

}

/*59 45 52 43 20 00 04 00 03 01 00 0.0 00 00 00 00 39 39 39 39
i  1  2 3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20
  39 39 39 39 83 00 02 00 01 10 00 00 01 00 00 00 -> serVoOn
i 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 */

YRC1000micro_command::~YRC1000micro_command(){
    delete this->header_to_send;
}
struct YRC1000micro_command::TxDataWriteVarPosition
{
    int32_t data_type;
    const int32_t figure = 0;
    const int32_t tool_no = 0;
    const int32_t user_coodirnate_no    = 0;
    const int32_t extended_figure       = 0;
    int32_t first_axis_position;
    int32_t second_axis_position;
    int32_t third_axis_position;
    int32_t fourth_axis_position;
    int32_t fifth_axis_position;
    int32_t sixth_axis_position;
    const int32_t seventh_axis_position  = 0;
    const int32_t eighth_axis_position   = 0;
};

QByteArray YRC1000micro_command::setServoOn(){
    //    qDebug() << *this->header_to_send->toHex();
    QByteArray cmd      = *header_to_send + data_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_ON_SERVO;
    cmd[CMD_ID_ADDRESS] = CMD_ID_SERVO_ON;
    cmd[CMD_INSTANCE]   = CMD_HEADER_SERVO_INSTANCE_ON;
    cmd[CMD_ATTRIBUTE]  = CMD_HEADER_SERVO_ATTRIBUTE;
    cmd[CMD_SERVICE]    = CMD_HEADER_SERVO_SERVICE;
    cmd[DATA_BYTE0]     = CMD_DATA_SERVO_ON;
    return cmd;
}

QByteArray YRC1000micro_command::setServoOff(){
    //qDebug() <<this-> header_to_send;
    QByteArray cmd = *header_to_send + data_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_OFF_SERVO;
    cmd[CMD_ID_ADDRESS] = CMD_ID_SERVO_ON;
    cmd[CMD_INSTANCE] = CMD_HEADER_SERVO_INSTANCE_ON;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_SERVO_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_SERVO_SERVICE;
    cmd[DATA_BYTE0] = CMD_DATA_SERVO_OFF;
    //qDebug() << cmd[11];
    return cmd;
}

QByteArray YRC1000micro_command::readRobotPosition(){
    QByteArray cmd = *header_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_GET_POSITION;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_READ_POS_DATA_SIZE;
    cmd[CMD_ID_ADDRESS] = CMD_ID_READ_ROBOT_POS;
    cmd[CMD_INSTANCE] = CMD_HEADER_READ_POS_INSTANCE_CARTESIAN;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_READ_POS_ATTRIBUTE_ALL;
    cmd[CMD_SERVICE] = CMD_HEADER_READ_POS_SERVICE_ALL;
    return cmd;
}

QByteArray YRC1000micro_command::readRobotPulse(){
    QByteArray cmd = *header_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_GET_PULSE;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_READ_POS_DATA_SIZE;
    cmd[CMD_ID_ADDRESS] = CMD_ID_READ_ROBOT_POS;
    cmd[CMD_INSTANCE] = 0x01;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_READ_POS_ATTRIBUTE_ALL;
    cmd[CMD_SERVICE] = CMD_HEADER_READ_POS_SERVICE_ALL;
    return cmd;
}

void YRC1000micro_command::initRobot(quint8 robot_number){
    move_cartesian_data[DATA_MOVE_ROBOT_NUMBER] = robot_number;
}

QByteArray YRC1000micro_command::setRobotPositionCartesian(quint8 coordinate, quint8 move_type,quint8 speed_type,
                                                           double speed, QVector<double>* position)
{
    QByteArray cmd = *header_to_send + move_cartesian_data;
    //    qDebug() << "cmd_1= " <<cmd;
    //    qDebug() << "cmd_1_1= " <<cmd.toHex();
    cmd[CMD_REQUEST_ID]     = CMD_REQUEST_ID_WRITE_POSITION;
    cmd[CMD_DATA_SIZE]      = CMD_HEADER_MOVE_CARTESIAN_DATA_SIZE;
    cmd[CMD_ID_ADDRESS]     = CMD_ID_MOVE_ROBOT_CARTESIAN;
    cmd[CMD_INSTANCE]       = move_type;
    cmd[CMD_ATTRIBUTE]      = CMD_HEADER_MOVE_ATTRIBUTE;
    cmd[CMD_SERVICE]        = CMD_HEADER_MOVE_SERVICE_ALL;

    if(speed < 0)
        speed = 0;

    quint32 speed_u     = (quint32)(speed*10);

    quint32 x_u         = (quint32)(position->at(0)*1000);
    quint32 y_u         = (quint32)(position->at(1)*1000);
    quint32 z_u         = (quint32)(position->at(2)*1000);
    quint32 roll_u      = (quint32)(position->at(3)*10000);
    quint32 pitch_u     = (quint32)(position->at(4)*10000);
    quint32 yaw_u       = (quint32)(position->at(5)*10000);

    cmd[DATA_MOVE_SPEED_TYPE+HEADER_SIZE]   = speed_type;           //cmd 40
    cmd[DATA_MOVE_COORDINATE+HEADER_SIZE]   = coordinate;           //cmd 48

    cmd[DATA_MOVE_SPEED+HEADER_SIZE+3]  = (quint8)(speed_u>>24);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+2]  = (quint8)(speed_u>>16);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+1]  = (quint8)(speed_u>>8);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE]    = (quint8)(speed_u);

    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+3]    = (quint8)(x_u>>24);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+2]    = (quint8)(x_u>>16);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+1]    = (quint8)(x_u>>8);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE]      = (quint8)(x_u);

    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+3]    = (quint8)(y_u>>24);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+2]    = (quint8)(y_u>>16);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+1]    = (quint8)(y_u>>8);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE]      = (quint8)(y_u);

    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+3]    = (quint8)(z_u>>24);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+2]    = (quint8)(z_u>>16);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+1]    = (quint8)(z_u>>8);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE]      = (quint8)(z_u);

    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+3]     = (quint8)(roll_u>>24);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+2]     = (quint8)(roll_u>>16);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+1]     = (quint8)(roll_u>>8);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE]       = (quint8)(roll_u);

    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+3]    = (quint8)(pitch_u>>24);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+2]    = (quint8)(pitch_u>>16);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+1]    = (quint8)(pitch_u>>8);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE]      = (quint8)(pitch_u);

    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+3]      = (quint8)(yaw_u>>24);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+2]      = (quint8)(yaw_u>>16);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+1]      = (quint8)(yaw_u>>8);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE]        = (quint8)(yaw_u);
    //    qDebug() << "cmd move caterisan: " << cmd.toHex();
    return cmd;
}

QByteArray YRC1000micro_command::HomePos()
{
    QByteArray data_move_pulse;
    for (int i = 0; i < 13*4+1; ++i)
    {
        data_move_pulse.append('\x00');
    }
    QByteArray cmd = *header_to_send + data_move_pulse;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_HOME_POS;
    cmd[CMD_ID_ADDRESS] = CMD_ID_MOVE_ROBOT_PULSE;
    cmd[CMD_INSTANCE] = CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_WRITE_HOME_PULSE_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_WRITE_HOME_PULSE_SERVICE;
    //data part
    cmd[DATA_BYTE0+4*0] = CMD_CONTROL_GROUP_ROBOT;
    cmd[DATA_BYTE0+4*1] = CMD_CONTROL_GROUP_STATION;
    cmd[DATA_BYTE0+4*2] = CMD_CLASSIFICATION_IN_SPEED;
    cmd[DATA_BYTE0+4*3] = 10*1000;  //speed fix cứng luôn

    cmd[DATA_BYTE0+4*4] = 0;    //robot 1st axis pulse value
    cmd[DATA_BYTE0+4*5] = 0;    //robot 2st axis pulse value
    cmd[DATA_BYTE0+4*6] = 0;    //robot 3st axis pulse value
    cmd[DATA_BYTE0+4*7] = 0;    //robot 4st axis pulse value
    cmd[DATA_BYTE0+4*8] = 0;    //robot 5st axis pulse value
    cmd[DATA_BYTE0+4*9] = 0;    //robot 6st axis pulse value
    cmd[DATA_BYTE0+4*10] = 0;    //robot 7st axis pulse value
    cmd[DATA_BYTE0+4*11] = 0;    //robot 8st axis pulse value
    cmd[DATA_BYTE0+4*12] = 0;    //Tool no
    qDebug() << cmd.toHex();
    return cmd;
}

QByteArray YRC1000micro_command::selectJob(char* jobName)
{
    QByteArray cmd = *header_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_SELECT_JOB;
    cmd[CMD_ID_ADDRESS] = CMD_ID_JOB_SELECT;
    cmd[CMD_DATA_SIZE]  = 36;
    cmd[CMD_DATA_SIZE]  = 0x24;
    cmd[CMD_INSTANCE]   = 0x01;
    cmd[CMD_ATTRIBUTE]  = 0x00;
    cmd[CMD_SERVICE]    = 0x02;


    //    qDebug() << "cmd size of jobSelect: " << cmd.size();
    // Tính kích thước của mảng jobName
    int jobNameSize = strlen(jobName);
    //    qDebug() << "string length: "<< jobNameSize;
    // Thêm dữ liệu từ mảng jobName vào cuối cmd
    cmd.append(jobName, jobNameSize);

    while (cmd.size() < 68) {
        cmd.append('\x00');
    }


    //    qDebug() << "command jobselect: " << cmd;
    //    qDebug() << "cmd size: " << cmd.size();
    //    printCMD(cmd);

    return cmd;
}

QByteArray YRC1000micro_command::startJob()
{
    QByteArray cmd = *header_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_START_JOB;
    cmd[CMD_ID_ADDRESS] = CMD_ID_JOB_START;
    cmd[CMD_INSTANCE]   = 0x01;
    cmd[CMD_ATTRIBUTE]  = 0x01;
    cmd[CMD_SERVICE]    = 0x10;
    uint32_t data       = 0x01;
    cmd[CMD_DATA_SIZE]  =sizeof (data);

    cmd.append('\x01');
    cmd.append('\x00');
    cmd.append('\x00');
    cmd.append('\x00');

    //    qDebug() << "cmd of startJob: "<< cmd.size();
    //    qDebug() << "command startJob: " << cmd.toHex();

    return cmd;

}

//---------------------------------- Read and Write Var Position----------------------
QByteArray YRC1000micro_command::getVarPosition(uint16_t index)
{
    QByteArray cmd = *header_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_GET_VARPOS;
    cmd[CMD_ID_ADDRESS] = CMD_ID_GET_VAR_POSITION;
    cmd[CMD_INSTANCE]   = index;
    cmd[CMD_ATTRIBUTE]  = 16;
    cmd[CMD_SERVICE]    = 0x01;
    cmd[CMD_DATA_SIZE]  = 0;

    //    qDebug() << "cmd of getVarPosition: "<< cmd.size();
    //    printCMD(cmd);

    return cmd;
}

QByteArray YRC1000micro_command::writeVarPosition(uint16_t index, QVector<int32_t> pos)
{
    QByteArray cmd = *header_to_send;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_WRITE_VARPOS;
    cmd[CMD_ID_ADDRESS] = CMD_ID_GET_VAR_POSITION;
    cmd[CMD_INSTANCE]   = index;
    cmd[CMD_SERVICE]    = 0x02;
    cmd[CMD_ATTRIBUTE]  = 0x00;       // DE MAC DINH BANG 0

    //    qDebug() << "cmd writeVarPosition size before add data part: "<< cmd.size();
    //    qDebug() << "value move at yrc1000micro_command: " << *pos;

    TxDataWriteVarPosition position;
    cmd[CMD_DATA_SIZE] = sizeof (position);
    position.data_type              = 0x11;      //catersian coordinates

    position.first_axis_position    = (int32_t)pos.at(0);
    position.second_axis_position   = (int32_t)pos.at(1);
    position.third_axis_position    = (int32_t)pos.at(2);
    position.fourth_axis_position   = (int32_t)pos.at(3);
    position.fifth_axis_position    = (int32_t)pos.at(4);
    position.sixth_axis_position    = (int32_t)pos.at(5);

    //    qDebug() << "--------------------------------------------------";
    //    qDebug() << "value move at yrc1000micro_command: ";
    //    printTxDataWriteVarPosition(position);
    //    qDebug() << "-----------------------------"<< sizeof(position);
    //    qDebug() << "positon size: "<< sizeof(position);

    // Appending bytes of the position struct
    cmd.append(reinterpret_cast<const char*>(&position), sizeof(position));
    //    qDebug() << "cmd writeVarPosition size shoule be 84 byte: "<< cmd.size();
    //    printCMD(cmd);
    //    qDebug() << "data of write var position: " << cmd.toHex();
    return cmd;
}
//---------------------------------- Read and Write Var Position----------------------
QByteArray YRC1000micro_command::writeVarPositionAgain(uint16_t index, QVector<double>* position)
{
    qDebug() << "positio at rewrite: " << position;
    QByteArray cmd = *header_to_send + position_variable_data;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_WRITE_VARPOS;
    cmd[CMD_ID_ADDRESS] = CMD_ID_GET_VAR_POSITION;
    cmd[CMD_INSTANCE]   = index;
    qDebug() << "index " << index;
    cmd[CMD_SERVICE]    = 0x10;
    cmd[CMD_ATTRIBUTE]  = 0x00;       // catersian coordinates
    cmd[CMD_DATA_SIZE] = 52; /*sizeof(position_variable_data);*/
    //    qDebug() << "size of positio at rewrite: " << position_variable_data.size();

    quint32 x_u         = (quint32)(position->at(0));
    quint32 y_u         = (quint32)(position->at(1));
    quint32 z_u         = (quint32)(position->at(2));
    quint32 roll_u      = (quint32)(position->at(3));
    quint32 pitch_u     = (quint32)(position->at(4));
    quint32 yaw_u       = (quint32)(position->at(5));

    //    qDebug() << "data to send: " << x_u;

    cmd [32]            = 0x11; // data type;

    cmd [/*52*/DATA_MOVE_X_CARTESIAN + HEADER_SIZE+ 3] =(quint8)(x_u>>24);             //first coordinate data
    cmd [DATA_MOVE_X_CARTESIAN + HEADER_SIZE+ 2] =(quint8)(x_u>>16);
    cmd [DATA_MOVE_X_CARTESIAN + HEADER_SIZE+ 1] =(quint8)(x_u>>8);             //first coordinate data
    cmd [DATA_MOVE_X_CARTESIAN + HEADER_SIZE   ] =(quint8)(x_u);

    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+3]    = (quint8)(y_u>>24);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+2]    = (quint8)(y_u>>16);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+1]    = (quint8)(y_u>>8);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE]      = (quint8)(y_u);

    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+3]    = (quint8)(z_u>>24);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+2]    = (quint8)(z_u>>16);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+1]    = (quint8)(z_u>>8);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE]      = (quint8)(z_u);

    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+3]     = (quint8)(roll_u>>24);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+2]     = (quint8)(roll_u>>16);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+1]     = (quint8)(roll_u>>8);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE]       = (quint8)(roll_u);

    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+3]    = (quint8)(pitch_u>>24);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+2]    = (quint8)(pitch_u>>16);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+1]    = (quint8)(pitch_u>>8);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE]      = (quint8)(pitch_u);

    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+3]      = (quint8)(yaw_u>>24);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+2]      = (quint8)(yaw_u>>16);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+1]      = (quint8)(yaw_u>>8);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE]        = (quint8)(yaw_u);

    qDebug() << "size of writeVarPositionAgain: " << cmd.size();
    qDebug() << "data of rewrite var position: " << cmd.toHex();
    return cmd;
}
//--------------------------------- Read and Write Byte-------------------------------
QByteArray YRC1000micro_command::getByte(uint16_t index)
{
    QByteArray cmd = *header_to_send;
    cmd[CMD_ID_ADDRESS] = CMD_ID_GET_AND_WRITE_BYTE;
    cmd[CMD_INSTANCE]   = index;
    cmd[CMD_ATTRIBUTE]  = 0x01;
    cmd[CMD_SERVICE]    = 0x0E;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_GET_BYTE;
    cmd[CMD_DATA_SIZE]  = 0;

    qDebug() << "GetByte:" <<index ;
    //    qDebug() << "cmd size getByte: "<< cmd.size();
    //    qDebug() << "cmd getByte: "<< cmd;
    //    qDebug() << "cmd of getByte: ";
    //    printCMD(cmd);

    return cmd;
}

QByteArray YRC1000micro_command::writeByte(uint16_t index, uint8_t data)
{
    QByteArray cmd = *header_to_send;
    cmd[CMD_ID_ADDRESS] = CMD_ID_GET_AND_WRITE_BYTE;
    cmd[CMD_INSTANCE]   = index;
    cmd[CMD_ATTRIBUTE]  = 0x01;
    cmd[CMD_SERVICE]    = 0x10;
    cmd[CMD_REQUEST_ID] = CMD_REQUEST_ID_WRITE_BYTE;

    //    qDebug() << "cmd size writeByte: "<< cmd.size();
    //    qDebug() << "cmd writeByte: "<< cmd;

    cmd[CMD_DATA_SIZE]  = sizeof(data);
    cmd.append(reinterpret_cast<const char*>(&data), sizeof(data));

    //    qDebug() << "cmd size writeByte: "<< cmd.size();
    //    qDebug() << "cmd writeByte: "<< cmd;
    //    qDebug() << "cmd of writeByte:" ;
    //    printCMD(cmd);

    return cmd;
}
//--------------------------------- Read and Write Byte-------------------------------
void YRC1000micro_command::printCMD(QByteArray cmd)
{
    // Convert cmd to a string of hexadecimal digits
    QString cmdHex = cmd.toHex();

    // Insert a backslash character between each pair of digits
    QString cmdBackslashSeparated;
    for (int i = 0; i < cmdHex.length(); i += 2) {
        cmdBackslashSeparated += cmdHex.mid(i, 2) + "\\";
    }
    cmdBackslashSeparated.chop(1); // remove the last backslash character

    qDebug() << "print cmd: " << cmdBackslashSeparated;
}

void YRC1000micro_command::printTxDataWriteVarPosition(const YRC1000micro_command::TxDataWriteVarPosition &pos) {
    qDebug() << "data_type:" << pos.data_type;
    qDebug() << "figure:" << pos.figure;
    qDebug() << "tool_no:" << pos.tool_no;
    qDebug() << "user_coodirnate_no:" << pos.user_coodirnate_no;
    qDebug() << "extended_figure:" << pos.extended_figure;
    qDebug() << "first_axis_position:" << pos.first_axis_position;
    qDebug() << "second_axis_position:" << pos.second_axis_position;
    qDebug() << "third_axis_position:" << pos.third_axis_position;
    qDebug() << "fourth_axis_position:" << pos.fourth_axis_position;
    qDebug() << "fifth_axis_position:" << pos.fifth_axis_position;
    qDebug() << "sixth_axis_position:" << pos.sixth_axis_position;
    qDebug() << "seventh_axis_position:" << pos.seventh_axis_position;
    qDebug() << "eighth_axis_position:" << pos.eighth_axis_position;
}

