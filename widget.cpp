/*
*******************************************************************************
** Copyright (C), 2019-2024, PGS R&D CENTER
** FileName: syschange.c
** <author>: hongming.cui
** Date:
** Description:
**
** Others:
** Function List:
**   1.
** History:
**  < ޶ ???    < ޶ ʱ  >    < ޶     >    < ޶  汾>
*******************************************************************************
*/
//*****************************************************************************
// File Include
//*****************************************************************************

#include "widget.h"
#include "ui_widget.h"
#include <QClipboard>
#include <QDir>
#include <QFileInfo>
extern "C" {
#include "micro_imu_lib.h"
}

// Forward declarations for globals used by IMU export helper.
extern QList<QString> heattimes;
extern QVector<double> LOG_Time;
extern QVector<double> LOG_Accel_x_h;
extern QVector<double> LOG_Accel_y_h;
extern QVector<double> LOG_Accel_z_h;
extern QVector<double> LOG_Gyro_x_h;
extern QVector<double> LOG_Gyro_y_h;
extern QVector<double> LOG_Gyro_z_h;
//*****************************************************************************
// Local Defines
//*****************************************************************************
#define READ_BOARD_STATE        0x10
#define READ_LOG_PROCESS        0x11


#define READ_REAL_TIME_LOG       0x20
#define WRITE_REAL_TIME_LOG      0x21

#define GET_BOARD_BASIC_INFO     0x44
#define SET_BOARD_BASIC_INFO     0x45

#define SYNC_PC_TIMETO_DEVICE    0x4C

#define READ_H_SETTING           0x4D
#define READ_H_TCR_TABLE         0x4F
#define READ_H_PROFILE           0x4E

#define H1_PORT           0x61
#define H2_PORT           0x62
#define H3_PORT           0x63

#define WRITE_H_SETTING           0x50
#define WRITE_H_TCR_TABLE         0x52
#define WRITE_H_PROFILE           0x51

#define BOARD_START_HEAT           0x53
#define BOARD_STOP_HEAT            0x54

#define BOARD_MYSELF_CALI_START           0x55
#define BOARD_MYSELF_CALI_STOP            0x56

#define ASK_EXPORT_LOG         0x5C
#define CLEAN_FLASH_DATA       0x5D



#define TABLE_WIDTH_TCR                   120
#define TABLE_WIDTH_HEATING               80
#define TABLE_TCR_HORIZONTAL_NUMBER       4
#define TABLE_HEATING_HORIZONTAL_NUMBER   5
#define TABLE_TCR_VERTICAL_NUMBER                  50
#define TABLE_HEATING_VERTICAL_NUMBER              20


#define REC_CHECK_TIME              4000


#define QPLAINTEXTEDIT_SET                  0
#define QPLAINTEXTEDIT_APPEND               1









#define THIRD_QCUSTOMPLOT_YAXIS2        10





#define MAX_TIME 4500
#define MIN_P 0
#define MAX_P 90
#define MIN_V 0
#define MAX_V 9
#define MIN_T 0
#define MAX_T 450


#define MAX_RESI 2.5
#define MIN_RESI 0.4
#define MAX_TCR 0.0034
#define MIN_TCR 0.0001


static uint8_t Log_For_PC[36] = {0};
static uint8_t Log_For_Act_PC[36] = {0};
static double Log_For_Act_display[36][1000000];
static uint32_t Log_For_Act_display_count = 0;
static uint16_t display_paintEvent_count = 0;
static uint16_t display_paintEvent_number = 0;
static bool s_heatGraphVisibilityApplied = false;

#define REALTIME_REPLOT_INTERVAL_POINTS  1
#define ACCEL_FILTER_REPLOT_INTERVAL       1
#define HEAT_PLOT_WINDOW_SEC               120.0
#define SERIAL_LOG_UI_INTERVAL             25

static void setGraphDataUpTo(QCPGraph *graph, const QVector<double> &keys,
                             const QVector<double> &values, int count)
{
    if (!graph || count <= 0)
        return;
    graph->setData(keys.mid(0, count), values.mid(0, count));
}



static QString buildImuResultPath(const QString &basePath)
{

    if (!basePath.isEmpty() && basePath.endsWith(".csv", Qt::CaseInsensitive))
    {
        return basePath.left(basePath.size() - 4) + "_imu_result.csv";
    }

    QDateTime now = QDateTime::currentDateTime();
    const QString baseName = "BAT_Heat_Log_Data_" + now.toString("yyyy_MM_dd_HH_mm_ss");
    const QString appPath = QCoreApplication::applicationDirPath();
    return appPath + "/TestData/./" + baseName + "_imu_result.csv";


}



static void SaveImuResultFileInPc(const QString &fileName, uint32_t arrayNumber)
{
    if (fileName.isEmpty() || arrayNumber == 0)
        return;

    QFile file(fileName);
    if (file.exists() && !file.remove())
    {
        qWarning() << "Failed to delete existing imu result file:" << fileName;
        return;
    }
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qWarning() << "Failed to open imu result file:" << fileName;
        return;
    }

    QTextStream out(&file);
    out << "UTC_Time,Time_s,"
        << "q0,q1,q2,q3,"
        << "R11,R12,R13,R21,R22,R23,R31,R32,R33,"
        << "roll,pitch,yaw,"
        << "aax,aay,aaz,"
        << "lax,lay,laz\n";

    MIL_Handle_t imuHandle = {};
    mil_handle_init(&imuHandle);


    //2026.6.10 mike nian 滤波后数据先化为g再称9.8
    const uint32_t validCount = qMin<uint32_t>(arrayNumber, static_cast<uint32_t>(heattimes.size()));
    for (uint32_t i = 0; i < validCount; ++i)
    {
        const float ax = qIsFinite(LOG_Accel_x_h[i]) ? static_cast<float>((LOG_Accel_x_h[i] / 1000) * 9.8) : 0.0f;      // -> m/s^2
        const float ay = qIsFinite(LOG_Accel_y_h[i]) ? static_cast<float>((LOG_Accel_y_h[i] / 1000) * 9.8) : 0.0f;
        const float az = qIsFinite(LOG_Accel_z_h[i]) ? static_cast<float>((LOG_Accel_z_h[i] / 1000) * 9.8) : 0.0f;
        const float gx = qIsFinite(LOG_Gyro_x_h[i]) ? static_cast<float>(LOG_Gyro_x_h[i] * 0.01745) : 0.0f;   // -> rad/s
        const float gy = qIsFinite(LOG_Gyro_y_h[i]) ? static_cast<float>(LOG_Gyro_y_h[i] * 0.01745) : 0.0f;
        const float gz = qIsFinite(LOG_Gyro_z_h[i]) ? static_cast<float>(LOG_Gyro_z_h[i] * 0.01745) : 0.0f;
        const float t = qIsFinite(LOG_Time[i]) ? static_cast<float>(LOG_Time[i]) : 0.0f;

        mil_get_imu(&imuHandle, gx, gy, gz, ax, ay, az, t);
        mil_while_run(&imuHandle);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        float aax = 0.0f, aay = 0.0f, aaz = 0.0f;
        float lax = 0.0f, lay = 0.0f, laz = 0.0f;
        mil_get_ang(&imuHandle, &roll, &pitch, &yaw);
        mil_get_aaccel(&imuHandle, &aax, &aay, &aaz);
        mil_get_alin(&imuHandle, &lax, &lay, &laz);

        out << heattimes.at(static_cast<int>(i)) << ","
            << QString::number(t, 'f', 6) << ","
            << QString::number(imuHandle.quat.w, 'f', 8) << ","
            << QString::number(imuHandle.quat.x, 'f', 8) << ","
            << QString::number(imuHandle.quat.y, 'f', 8) << ","
            << QString::number(imuHandle.quat.z, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R11, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R12, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R13, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R21, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R22, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R23, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R31, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R32, 'f', 8) << ","
            << QString::number(imuHandle.rotMat.R33, 'f', 8) << ","
            << QString::number(roll, 'f', 6) << ","
            << QString::number(pitch, 'f', 6) << ","
            << QString::number(yaw, 'f', 6) << ","
            << QString::number(aax, 'f', 6) << ","
            << QString::number(aay, 'f', 6) << ","
            << QString::number(aaz, 'f', 6) << ","
            << QString::number(lax, 'f', 6) << ","
            << QString::number(lay, 'f', 6) << ","
            << QString::number(laz, 'f', 6) << "\n";
    }
}



uint32_t heatreccount = 0;

QList<QString> heattimes;
QVector<double>LOG_Time(1000000),LOG_Vin(1000000),LOG_Ain(1000000);
QVector<double>LOG_TC1(1000000),LOG_TC2(1000000);
QVector<double>LOG_Rh1(1000000),LOG_Vh1(1000000),LOG_Ah1(1000000),LOG_Acp1(1000000),LOG_Th1(1000000),LOG_Dty1(1000000),LOG_Pf1(1000000),LOG_Err1(1000000);
QVector<double>LOG_Rh2(1000000),LOG_Vh2(1000000),LOG_Ah2(1000000),LOG_Acp2(1000000),LOG_Th2(1000000),LOG_Dty2(1000000),LOG_Pf2(1000000),LOG_Err2(1000000);
QVector<double>LOG_Accel_x(1000000),LOG_Accel_y(1000000),LOG_Accel_z(1000000),LOG_Gyro_x(1000000),LOG_Gyro_y(1000000),LOG_Gyro_z(1000000),LOG_Temp_c(1000000);
QVector<double>LOG_Accel_x_h(1000000),LOG_Accel_y_h(1000000),LOG_Accel_z_h(1000000),LOG_Gyro_x_h(1000000),LOG_Gyro_y_h(1000000),LOG_Gyro_z_h(1000000);


QStringList Qstring_Save_Log_Name = {"UTC_Time",
                                     "Vin",
                                     "TC1",
                                     "TC2",

                                     "Rh1",
                                     "Vh1",
                                     "Ah1",
                                     "Acp1",
                                     "Dty1",
                                     "Err1",

                                     "Rh2",
                                     "Vh2",
                                     "Ah2",
                                     "Acp2",
                                     "Dty2",
                                     "Err2",

                                     "Accel_x",
                                     "Accel_y",
                                     "Accel_z",
                                     "gyro_x",
                                     "gyro_y",
                                     "gyro_z",
                                     "Temp_c",

                                     "Accel_x_h",
                                     "Accel_y_h",
                                     "Accel_z_h",
                                     "gyro_x_h",
                                     "gyro_y_h",
                                     "gyro_z_h",

                                    };


QStringList Qstring_Qcustomplot_display_Name = {
                                    "Vin",

                                    "TC1",
                                    "TC2",


                                    "Rh1",
                                    "Vh1",
                                    "Ah1",
                                    "Dty1",


                                    "Rh2",
                                    "Vh2",
                                    "Ah2",
                                    "Dty2"
                                    };



QStringList Qstring_event_Log_Name = {
                                     "0x01 Beginning of profile",
                                     "0x02 Next segment of profile",
                                     "0x10 CP Target reached",
                                     "0x11 CP Target cannot reached",
                                     "0x20 CT-TC heating up",
                                     "0x21 CT-TC cooling down",
                                     "0x22 CT-TC Target reached",
                                     "0x30 CT-TCR heating up",
                                     "0x31 CT-TCR cooling down",
                                     "0x32 CT-TCR Target reached",
                                     "0x90 Profile is interrupted",
                                     "0x99 Ending of profile"
                                    };

QStringList Qstring_error_Log_Name = {
                                     "0x01 Heater R error",
                                     "0x02 Heater I error",
                                     "0x03 Heater T error",
                                     "0x04 Heater Thermal couple error  C no data",
                                     "0x05 Heater no power ",
                                     "0x06 Temperature Gap out of limit: +-20  ",
                                     "0x07 No heater connected",
                                     "0x08 No PPS Plugged in",
                                     "0x09 No Heater profile stored in the MCU",
                                     "0x10 Calibration Fail(Can't reach the expect temperature and continuous next step)",
                                     "0x11 PPS -in Over voltage",
                                     "0x12 PPS -in Reverse connection",
                                     "0x21 Board UART to PC error ",
                                     "0x22 Board UART to Driven board",
                                     "0x23 Board Over voltage protection",
                                     "0x24 Board Over temperature protection",
                                     "0x25 Board under voltage",
                                     "0x26 Board under Current"
                                    };
uint8_t Log_Flash_Act_Number = 0;
uint8_t Log_Heat_Act_Number = 0;
QString Middle_Num;

//*****************************************************************************
// Local structures
//*****************************************************************************

typedef struct
{
    uint8_t Uart_Tran_state;
    uint8_t Uart_Export_Flash_state;
    uint8_t Uart_Export_Flash_count;


}UART_T;


Heat_Data_Set H1_data;
Heat_Data_Set H2_data;
Heat_Data_Set H3_data;

Board_Data_Base_Set Board_data;
Board_Data_Base_Set Log_display_flag;

static int logSamplingRateHz()
{
    if (1 == Board_data.Sampling_period) return 50;
    if (2 == Board_data.Sampling_period) return 25;
    if (5 == Board_data.Sampling_period) return 10;
    if (10 == Board_data.Sampling_period) return 5;
    return 50;
}

UART_T sys_uart;

//*****************************************************************************
// Global Data
//*****************************************************************************

uint8_t ReceiveDataArray[1000];

uint16_t Table_Input_Tcr_count = 0,Table_Init_Tcr_count = 0,Table_Heating_profiles1_count = 0,Table_Heating_profiles2_count = 0,Table_Heating_profiles3_count = 0;
unsigned int Timer_Count = 0;
QStringList SerialNamePortList;
uint32_t Flash_Rec_Data_Number_ALL = 0;
uint32_t Flash_Rec_Data_Number_Act = 0;


uint32_t Heat_Rec_Data_Number_ALL = 0;
uint32_t Heat_Rec_Data_Number_Act = 0;


QByteArray Uart_Actual_info,Uart_Save_Info,hexdata_test;


QStringList ColorStringList = {"times(s)","Temperature","TCR"};
QStringList CustomNamestringList = {"times(s)","Temperature","TCR"};
QStringList Export_Flash_data_process = {"[",">"," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","]"};




uint16_t Export_Flash_Data_flag = 0;
uint16_t BAT_Device_State_Flag = 0;
uint16_t BAT_Device_State_Count = 0;

float TCR_Temp[50] = {0};
float TCR_Resi[50] = {0};
float TCR_TCR[50] = {0};

float Profile_Heat_Start_Time[20];
float Profile_Heat_Stop_Time[20];
QString Profile_Heat_Type[20];
float Profile_Heat_Number[20];


QString strPath;
QString TcrstrPath;


uint8_t pre_error_code = 0;


//*****************************************************************************
// Local Data
//*****************************************************************************



//*****************************************************************************
// Local Functions Declaration
//*****************************************************************************



//*****************************************************************************
// API Functions
//*****************************************************************************

void castVariant2ListListVariant(const QVariant &var, QList<QList<QVariant> > &res);
void castListListVariant2Variant(const QList<QList<QVariant> > &cells, QVariant &res);





/**************************************************************************
 *       ??
 *         
 *         ???
 *         ???
 *************************************************************************/






Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/Fire.ico"));

    //     QLabel  ʾͼƬ
    QPixmap pixmap2(":/Grey.png"); //  滻Ϊ    ͼƬ·???
    ui->BAT_Device_Connect_light->setPixmap(pixmap2);
    // ȷ  ͼƬ         ŵ QLabel  С
    ui->BAT_Device_Connect_light->setScaledContents(true);

    //     QLabel  ʾͼƬ
    QPixmap pixmap3(":/Grey.png"); //  滻Ϊ    ͼƬ·???
    ui->BAT_Device_State_light->setPixmap(pixmap3);
    // ȷ  ͼƬ         ŵ QLabel  С
    ui->BAT_Device_State_light->setScaledContents(true);



    //     QLabel  ʾͼƬ
    QPixmap pixmap(":/BATMAINTITLE.png"); //  滻Ϊ    ͼƬ·???
    ui->BAT_Label->setPixmap(pixmap);

    // ȷ  ͼƬ         ŵ QLabel  С
    ui->BAT_Label->setScaledContents(true);

  //  QIcon icon_start_heat(":/start.png");
    //   QIcon   ø   ???
  //  ui->BAT_Start_Heat->setIcon(icon_start_heat);


  //  QIcon icon_stop_heat(":/stop.png");
    //   QIcon   ø   ???
  //  ui->BAT_Stop_Heat->setIcon(icon_stop_heat);




    QStringList HeadText;
    QStringList HeadText2;

    HeadText<< "Index"  << "Tempareture" << "R" << "TCR";
    HeadText2 << "Index" << "Begin(0.1s)" << "End(0.1s)" << "Type" << "Target Value";

    a_Init_TableWidget(ui->BAT_Table_Set_Profile1,TABLE_WIDTH_TCR,TABLE_HEATING_HORIZONTAL_NUMBER,Table_Heating_profiles1_count,TABLE_HEATING_VERTICAL_NUMBER,HeadText2);

    a_Init_TableWidget(ui->BAT_Table_Set_Profile2,TABLE_WIDTH_TCR,TABLE_HEATING_HORIZONTAL_NUMBER,Table_Heating_profiles2_count,TABLE_HEATING_VERTICAL_NUMBER,HeadText2);



    a_Init_CustomPlot(ui->BAT_customplot_Heat,CustomNamestringList[0],CustomNamestringList[1],11,3,500,REAL_LOG,Qstring_Qcustomplot_display_Name);
    a_Init_CustomPlot(ui->BAT_Customplot_Set_Profile1,CustomNamestringList[0],CustomNamestringList[1],3,3,500,PROFILE,Qstring_Qcustomplot_display_Name);
    a_Init_CustomPlot(ui->BAT_Customplot_Set_Profile2,CustomNamestringList[0],CustomNamestringList[1],3,3,500,PROFILE,Qstring_Qcustomplot_display_Name);


    ui->BAT_Customplot_Set_Hannwindow->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                     QCP::iSelectLegend | QCP::iSelectPlottables);


    ui->BAT_Customplot_Set_Hannwindow->addGraph();
    ui->BAT_Customplot_Set_Hannwindow->addGraph();
    ui->BAT_Customplot_Set_Hannwindow->addGraph();

    ui->BAT_Customplot_Set_Hannwindow->legend->setVisible(true);

    ui->BAT_Customplot_Set_Hannwindow->graph(0)->setName("Accel_x");
    ui->BAT_Customplot_Set_Hannwindow->graph(0)->setPen(QPen(Qt::blue));

    ui->BAT_Customplot_Set_Hannwindow->graph(1)->setName("Accel_y");
    ui->BAT_Customplot_Set_Hannwindow->graph(1)->setPen(QPen(Qt::green));

    ui->BAT_Customplot_Set_Hannwindow->graph(2)->setName("Accel_z");
    ui->BAT_Customplot_Set_Hannwindow->graph(2)->setPen(QPen(Qt::red));


    QFont axisLabelFont = ui->BAT_Customplot_Set_Hannwindow->xAxis->labelFont();
    if (axisLabelFont.pointSize() <= 0) axisLabelFont.setPointSize(10);
    ui->BAT_Customplot_Set_Hannwindow->xAxis->setLabelFont(axisLabelFont);
    ui->BAT_Customplot_Set_Hannwindow->xAxis->setTickLabelFont(axisLabelFont);

    axisLabelFont = ui->BAT_Customplot_Set_Hannwindow->yAxis->labelFont();
    if (axisLabelFont.pointSize() <= 0) axisLabelFont.setPointSize(10);
    ui->BAT_Customplot_Set_Hannwindow->yAxis->setLabelFont(axisLabelFont);
    ui->BAT_Customplot_Set_Hannwindow->yAxis->setTickLabelFont(axisLabelFont);

    axisLabelFont = ui->BAT_Customplot_Set_Hannwindow->yAxis2->labelFont();
    if (axisLabelFont.pointSize() <= 0) axisLabelFont.setPointSize(10);
    ui->BAT_Customplot_Set_Hannwindow->yAxis2->setLabelFont(axisLabelFont);
    ui->BAT_Customplot_Set_Hannwindow->yAxis2->setTickLabelFont(axisLabelFont);

    ui->BAT_Customplot_Set_Hannwindow->xAxis->setLabel(CustomNamestringList[0]);
    ui->BAT_Customplot_Set_Hannwindow->yAxis->setLabel("Accel");

    ui->BAT_Customplot_Set_Hannwindow->xAxis->setRange(0, 10);
    ui->BAT_Customplot_Set_Hannwindow->yAxis->setRange(-2000, 2000);










    ui->BAT_Customplot_Set_Hannwindow_Gyro->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                     QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->BAT_Customplot_Set_Hannwindow_Gyro->addGraph();
    ui->BAT_Customplot_Set_Hannwindow_Gyro->addGraph();
    ui->BAT_Customplot_Set_Hannwindow_Gyro->addGraph();

    ui->BAT_Customplot_Set_Hannwindow_Gyro->legend->setVisible(true);

    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(0)->setName("Gyro_x");
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(0)->setPen(QPen(Qt::blue));

    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(1)->setName("Gyro_y");
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(1)->setPen(QPen(Qt::green));

    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(2)->setName("Gyro_z");
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(2)->setPen(QPen(Qt::red));


    QFont axisLabelFont1 = ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->labelFont();
    axisLabelFont1.setPointSize(10);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setLabelFont(axisLabelFont1);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setTickLabelFont(axisLabelFont1);


    axisLabelFont = ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->labelFont();

    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setLabelFont(axisLabelFont);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setTickLabelFont(axisLabelFont);

    axisLabelFont = ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis2->labelFont();
    axisLabelFont.setPointSize(10);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis2->setLabelFont(axisLabelFont);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis2->setTickLabelFont(axisLabelFont);

    ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setLabel(CustomNamestringList[0]);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setLabel("gyro");



    ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setRange(0, 10);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setRange(-2000, 2000);















    QStringList SerialNamePort;
    C_Serialport_Bat = new QSerialPort(this);


    CTimer = new QTimer(this);
    Timer_Count = 0;
    CTimer->stop();
    CTimer->setInterval(1);            //   ö ʱ   ڣ   λ      
    CTimer->start();                   //  ʱ    ʼ  ???














    // ????  ??  ???3???????  ?0??3.000
    QDoubleValidator *doubleValidator_R = new QDoubleValidator(this);
    doubleValidator_R->setRange(0.02, 3.0, 3);
    doubleValidator_R->setNotation(QDoubleValidator::StandardNotation);
    ui->BAT_H1_Max_R->setValidator(doubleValidator_R);
    ui->BAT_H1_Min_R->setValidator(doubleValidator_R);

    ui->BAT_H1_Temp_limit->setMaxLength(3);
    ui->BAT_H1_Temp_limit->setValidator(new QIntValidator(0,100, this));



    QDoubleValidator *doubleValidator_PID = new QDoubleValidator(this);
    doubleValidator_PID->setRange(-9.9, 9.9, 1);
    doubleValidator_PID->setNotation(QDoubleValidator::StandardNotation);


    QDoubleValidator *doubleValidator_COMP = new QDoubleValidator(this);
    doubleValidator_COMP->setRange(0.0, 9.0, 1);
    doubleValidator_COMP->setNotation(QDoubleValidator::StandardNotation);


    ui->BAT_H1_Kp->setMaxLength(3);
    ui->BAT_H1_Kp->setValidator(new QIntValidator(0,999, this));
    ui->BAT_H1_Ki->setMaxLength(3);
    ui->BAT_H1_Ki->setValidator(new QIntValidator(0,999, this));
    ui->BAT_H1_Kd->setMaxLength(3);
    ui->BAT_H1_Kd->setValidator(new QIntValidator(0,999, this));







    // ????  ??  ???3???????  ?0??3.000
    ui->BAT_H2_Max_R->setValidator(doubleValidator_R);
    ui->BAT_H2_Min_R->setValidator(doubleValidator_R);


    ui->BAT_H2_Temp_limit->setMaxLength(3);
    ui->BAT_H2_Temp_limit->setValidator(new QIntValidator(0,100, this));

    ui->BAT_H2_Kp->setMaxLength(3);
    ui->BAT_H2_Kp->setValidator(new QIntValidator(0,999, this));
    ui->BAT_H2_Ki->setMaxLength(3);
    ui->BAT_H2_Ki->setValidator(new QIntValidator(0,999, this));
    ui->BAT_H2_Kd->setMaxLength(3);
    ui->BAT_H2_Kd->setValidator(new QIntValidator(0,999, this));





    // ????  ??  ???3???????  ?0??3.000





    QDoubleValidator *doubleValidator_INITTEMP = new QDoubleValidator(this);
    doubleValidator_INITTEMP->setRange(0.0, 40.0, 1);
    doubleValidator_INITTEMP->setNotation(QDoubleValidator::StandardNotation);



    connectSignalSolt();

    Board_data.heat_tcrt_time_flag = 0;
    Board_data.heat_profile_time_flag = 0;


    for(uint8_t middle_count = 0;middle_count < 36;middle_count ++)
    {
        Log_For_PC[middle_count] = 1;
    }



    initFilters();
    initHannWindow();
}


/**************************************************************************
 *       ??connectSignalSolt()
 *         :   Ӻ       UI   ؽӿں                   ???
 *         ???
 *         ???
 *************************************************************************/

void Widget::connectSignalSolt()
{
    connect(C_Serialport_Bat,SIGNAL(readyRead()),this,SLOT(C_serialPortReadYRead_slot()));


    //connect(ui->C_OpenSerialPort, SIGNAL(clicked(bool)), this, SLOT(serialOpen()));

    connect(CTimer, SIGNAL(timeout()), this, SLOT(on_timer_timeout()));

    //connect(ui->C_customplot, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel()));
}

/**************************************************************************
 *       ??on_timer_timeout()
 *         :  ʱ        ÿ  ???s Դ  ڽӿں    б     ˢ £     ???
 *         ???
 *         ???
 *************************************************************************/
void Widget::on_timer_timeout()
{
    Timer_Count++;

    if(Timer_Count % 100 == 0)
    {



        if((Board_data.heat_tcrt_time_flag == 0)&&(Board_data.heat_profile_time_flag == 0))
        {
            Board_data.heat_time_count = 0;
        }
        QStringList SerialNamePort;

        foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
        {
            SerialNamePort<<info.portName();
        }
        std::sort(SerialNamePort.begin(),SerialNamePort.end());
        if(SerialNamePortList.size() != SerialNamePort.size())
        {
            ui->BAT_SerialPortSelect->clear();
            SerialNamePortList = SerialNamePort;
            ui->BAT_SerialPortSelect->addItems(SerialNamePortList);
        }
    }

    if(Timer_Count > 1000)
    {
        Timer_Count = 0;
    }


    if(BAT_Device_State_Count>0)
    {
        BAT_Device_State_Count --;

        if(BAT_Device_State_Count == 0)
        {

            //     QLabel  ʾͼƬ
            QPixmap pixmap2(":/Grey.png"); //  滻Ϊ    ͼƬ·???
            ui->BAT_Device_State_light->setPixmap(pixmap2);
            // ȷ  ͼƬ         ŵ QLabel  С
            ui->BAT_Device_State_light->setScaledContents(true);

            if(0 != Log_For_Act_display_count)
            {
                //C2
                qDebug() << "csv1:";
                Board_Save_Log_File_In_Pc(strPath,Log_For_Act_display_count-1);
                Log_For_Act_display_count = 0;
                Board_data.heat_profile_time_flag = 0;
                Board_data.heat_time_count = 0;
                heattimes.clear();
                qDebug() << "now is error ";
                std::fill(LOG_Time.begin(), LOG_Time.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0

            }

            QStringList SerialNamePort;

            foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
            {
                SerialNamePort<<info.portName();
            }

           if (SerialNamePort.contains(C_Serialport_Bat->portName()))
           {
                 qDebug() << "Found the target port:" << C_Serialport_Bat->portName();
           }
           else
           {
               C_Serialport_Bat->close();
               serialRealStatus = 0;
               ui->BAT_Key_Serial_Control->setText("Open");

               //     QLabel  ʾͼƬ
               QPixmap pixmap1(":/Grey.png"); //  滻Ϊ    ͼƬ·???
               ui->BAT_Device_Connect_light->setPixmap(pixmap1);
               // ȷ  ͼƬ         ŵ QLabel  С
               ui->BAT_Device_Connect_light->setScaledContents(true);
              qDebug() << "The target port is not available:" << C_Serialport_Bat->portName();
           }


        }
    }
}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::QCustomplot_Display_Data_Proc(QTableWidget *TableWidget,QCustomPlot *CustomPlot,uint8_t ArrayNumber)
{
    uint8_t row = TableWidget->rowCount();
    uint8_t col = TableWidget->columnCount();
    H1_data.Middle_count = 0;

    for (uint8_t j = 0; j < row; j++)
    {
        H1_data.Middle_count++;
        TCR_Temp[j] = QString(TableWidget->item(j, 1)->text()).toFloat();
        if((TCR_Temp[j]<1)||(TCR_Temp[j]>5000))
        {
            if(TCR_Temp[j] == 0)
            {
                if(j > 0)
                {
                    H1_data.Middle_count --;
                    qDebug() << "H1_data.Middle_count" << H1_data.Middle_count;
                    break;
                }


            }
        }
    }
    qDebug() << "H1_data.Middle_count" << H1_data.Middle_count;
    if(TCR_T == ArrayNumber)
    {
        QVector<double>TEMPARRAY(100),TCRARRAY(100);
        H1_data.TCRT_count = H1_data.Middle_count;


        float MAX_Y = 0,MAX_X = 0;
        qDebug() << row  << col;



        for (uint8_t i = 1; i < (col); i++)
        {
            for (uint8_t j = 0; j < H1_data.TCRT_count; j++)
            {
                if(i == 1)
                {
                    H1_data.TCR_Temp[j] = TCR_Temp[j] = QString(TableWidget->item(j, i)->text()).toFloat();
                }
                else if(i == 2)
                {
                    H1_data.TCR_Resi[j] = TCR_Resi[j] = QString(TableWidget->item(j, i)->text()).toFloat();
                }
                else if(i == 3)
                {
                    H1_data.TCR_TCR[j] = TCR_TCR[j] = QString(TableWidget->item(j, i)->text()).toFloat();
                }
            }
        }

        for (uint8_t j = 0; j < H1_data.TCRT_count; j++)
        {

            if(TCR_TCR[j] > MAX_Y)
            {
                if(TCR_Temp[j] != 0)
                {
                    MAX_Y = TCR_TCR[j];
                }
            }

            if(TCR_Temp[j] > MAX_X)
            {
                MAX_X = TCR_Temp[j];
            }

            TEMPARRAY[j] = TCR_Temp[j];
            TCRARRAY[j] = TCR_TCR[j];

            if(TCR_Temp[j] == 0)
            {
                TCRARRAY[j] = 0;
            }
            qDebug() << TEMPARRAY[j]<< TCRARRAY[j];


         /*
            uint16_t Init_T0,Init_R0;
            double Double_R0;

            Init_T0 = ui->BAT_Init_TempT0->text().toUInt();

            if((Init_T0<20)||(Init_T0>40))
            {
                QMessageBox::information(this,"State","Please enter the correct range  Beginning of Temp(20---40) ");
                return;
            }
            qDebug() <<" Init_T0"  << Init_T0;
            Double_R0 = ui->BAT_Init_ResiR0->text().toDouble();
            Init_R0 = static_cast<int>(std::round(ui->BAT_Init_ResiR0->text().toDouble()*10000))/10;
            if((Init_R0<400)||(Init_R0>2500))
            {
                QMessageBox::information(this,"State","Please enter the correct range  Beginning of Resi(0.4---2.5) ");
                return;
            }
            qDebug() <<" Init_R0"  << Init_R0;

            Init_R0 = ui->BAT_Init_ResiR0->text().toDouble();

            H1_data.TCR_TCR[j] = TCR_TCR[j] = (TCR_Resi[j] - Double_R0)/Double_R0/(TCR_Temp[j] - Init_T0);

            //TCR_TCR[0] = 0;

            if(TCR_TCR[j] > MAX_Y)
            {
                if(TCR_Temp[j] != 0)
                {
                    MAX_Y = TCR_TCR[j];
                }
            }

            if(TCR_Temp[j] > MAX_X)
            {
                MAX_X = TCR_Temp[j];
            }

            TEMPARRAY[j] = TCR_Temp[j];
            TCRARRAY[j] = TCR_TCR[j];

            if(TCR_Temp[j] == 0)
            {
                TCRARRAY[j] = 0;
            }
            qDebug() << TEMPARRAY[j]<< TCRARRAY[j];*/

        }
        CustomPlot->graph(0)->setData(TEMPARRAY,TCRARRAY);
        //a_Array_Data_Tran_TableWidget(TableWidget,&H1_data,TCR_T);
        CustomPlot->xAxis->setRange(TEMPARRAY[0],MAX_X);
        CustomPlot->yAxis->setRange(0,MAX_Y);
    }
    else if(PROFILE == ArrayNumber)
    {
        QVector<double>POWERARRAY(40),TIME1ARRAY(40),VOLTRRAY(40),TEMPARRAY(40),TIME2ARRAY(40),TIME3ARRAY(40);
        uint8_t row = TableWidget->rowCount();
        uint8_t col = TableWidget->columnCount();
        H1_data.Profile_count = H1_data.Middle_count;


        float MAX_Y = 0,MAX_X = 0;

        for (uint8_t i = 1; i < (col); i++)
        {
            for (uint8_t j = 0; j < H1_data.Profile_count; j++)
            {
                if(i == 1)
                {
                    Profile_Heat_Start_Time[j] = QString(TableWidget->item(j, i)->text()).toFloat()/10;
                }
                else if(i == 2)
                {
                    Profile_Heat_Stop_Time[j] = QString(TableWidget->item(j, i)->text()).toFloat()/10;
                }
                else if(i == 3)
                {                    
                    Profile_Heat_Type[j] = QString(TableWidget->item(j, i)->text());

                }
                else if(i == 4)
                {
                    Profile_Heat_Number[j] = QString(TableWidget->item(j, i)->text()).toFloat();
                }

            }
        }
        uint8_t time1_count = 0,time2_count = 0,time3_count = 0;
        for (uint8_t j = 0; j < H1_data.Profile_count; j++)
        {

            if(Profile_Heat_Number[j] > MAX_Y)
            {
                MAX_Y = Profile_Heat_Number[j];
            }

            if(Profile_Heat_Stop_Time[j] > MAX_X)
            {
                MAX_X = Profile_Heat_Stop_Time[j];
            }

            if(j > 0)
            {
                if(Profile_Heat_Start_Time[j] != 0)
                {
                    if(Profile_Heat_Type[j] == "V")
                    {
                        TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                        VOLTRRAY[time1_count ++] = Profile_Heat_Number[j];
                        TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        VOLTRRAY[time1_count ++] = Profile_Heat_Number[j];

                        TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                        TEMPARRAY[time2_count ++] = 0;
                        TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        TEMPARRAY[time2_count ++] = 0;

                        TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                        POWERARRAY[time3_count ++] = 0;
                        TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        POWERARRAY[time3_count ++] = 0;

                    }
                    else if(Profile_Heat_Type[j] == "T")
                    {
                        TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                        VOLTRRAY[time1_count ++] = 0;
                        TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        VOLTRRAY[time1_count ++] = 0;

                        TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                        TEMPARRAY[time2_count ++] = Profile_Heat_Number[j];
                        TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        TEMPARRAY[time2_count ++] = Profile_Heat_Number[j];


                        TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                        POWERARRAY[time3_count ++] = 0;
                        TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        POWERARRAY[time3_count ++] = 0;

                    }
                    else if(Profile_Heat_Type[j] == "P")
                    {
                        TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                        VOLTRRAY[time1_count ++] = 0;
                        TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        VOLTRRAY[time1_count ++] = 0;

                        TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                        TEMPARRAY[time2_count ++] = 0;
                        TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        TEMPARRAY[time2_count ++] = 0;

                        TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                        POWERARRAY[time3_count ++] = Profile_Heat_Number[j];
                        TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        POWERARRAY[time3_count ++] = Profile_Heat_Number[j];
                    }
                    else
                    {
                        TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                        VOLTRRAY[time1_count ++] = 0;
                        TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        VOLTRRAY[time1_count ++] = 0;

                        TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                        TEMPARRAY[time2_count ++] = 0;
                        TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        TEMPARRAY[time2_count ++] = 0;

                        TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                        POWERARRAY[time3_count ++] = 0;
                        TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                        POWERARRAY[time3_count ++] = 0;

                    }
                }
            }
            else if(j == 0)
            {
                if(Profile_Heat_Type[j] == "V")
                {
                    TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                    VOLTRRAY[time1_count ++] = Profile_Heat_Number[j];
                    TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    VOLTRRAY[time1_count ++] = Profile_Heat_Number[j];

                    TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                    TEMPARRAY[time2_count ++] = 0;
                    TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    TEMPARRAY[time2_count ++] = 0;

                    TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                    POWERARRAY[time3_count ++] = 0;
                    TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    POWERARRAY[time3_count ++] = 0;

                }
                else if(Profile_Heat_Type[j] == "T")
                {
                    TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                    VOLTRRAY[time1_count ++] = 0;
                    TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    VOLTRRAY[time1_count ++] = 0;

                    TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                    TEMPARRAY[time2_count ++] = Profile_Heat_Number[j];
                    TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    TEMPARRAY[time2_count ++] = Profile_Heat_Number[j];

                    TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                    POWERARRAY[time3_count ++] = 0;
                    TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    POWERARRAY[time3_count ++] = 0;

                }
                else if(Profile_Heat_Type[j] == "P")
                {
                    TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                    VOLTRRAY[time1_count ++] = 0;
                    TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    VOLTRRAY[time1_count ++] = 0;

                    TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                    TEMPARRAY[time2_count ++] = 0;
                    TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    TEMPARRAY[time2_count ++] = 0;

                    TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                    POWERARRAY[time3_count ++] = Profile_Heat_Number[j];
                    TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    POWERARRAY[time3_count ++] = Profile_Heat_Number[j];
                }
                else
                {
                    TIME1ARRAY[time1_count] = Profile_Heat_Start_Time[j] + 0.01;
                    VOLTRRAY[time1_count ++] = 0;
                    TIME1ARRAY[time1_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    VOLTRRAY[time1_count ++] = 0;

                    TIME2ARRAY[time2_count] = Profile_Heat_Start_Time[j] + 0.01;
                    TEMPARRAY[time2_count ++] = 0;
                    TIME2ARRAY[time2_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    TEMPARRAY[time2_count ++] = 0;

                    TIME3ARRAY[time3_count] = Profile_Heat_Start_Time[j] + 0.01;
                    POWERARRAY[time3_count ++] = 0;
                    TIME3ARRAY[time3_count] = Profile_Heat_Stop_Time[j] - 0.01;
                    POWERARRAY[time3_count ++] = 0;

                }
            }
        }

        CustomPlot->graph(0)->setData(TIME1ARRAY,POWERARRAY);
        CustomPlot->graph(1)->setData(TIME2ARRAY,TEMPARRAY);
        CustomPlot->graph(2)->setData(TIME3ARRAY,VOLTRRAY);


        CustomPlot->xAxis->setRange(0,MAX_X);
        CustomPlot->yAxis->setRange(0,MAX_Y);
    }
    else if(REAL_LOG == ArrayNumber)
    {

        CustomPlot->xAxis->setRange(0,1000);
        CustomPlot->yAxis->setRange(0,300);
    }


    CustomPlot->replot();
}





/**************************************************************************
 *       ??
 *         
 *         ???
 *         ???
 *************************************************************************/
void Widget::a_Init_CustomPlot(QCustomPlot *CustomPlot,QString xAxis_Label,QString yAxis_Label,uint8_t Graph_Count,uint16_t xAxis_Range,uint16_t yAxis_Range,uint8_t ArrayNumber,QStringList Qcustomplot_display_Name)
{
    CustomPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                     QCP::iSelectLegend | QCP::iSelectPlottables);


    if(PROFILE == ArrayNumber)
    {

            CustomPlot->addGraph();
            CustomPlot->addGraph();
            CustomPlot->addGraph();


            CustomPlot->graph(0)->setName("1-power");

            CustomPlot->graph(0)->setPen(QPen(Qt::blue)); //


            CustomPlot->graph(1)->setName("2-temperature");

            CustomPlot->graph(1)->setPen(QPen(Qt::green)); //


            CustomPlot->graph(2)->setName("3-voltage");

            CustomPlot->graph(2)->setPen(QPen(Qt::red)); //

    }
    else if(REAL_LOG == ArrayNumber)
    {
        QVector<QColor> colorList;
        //         ʹ   κ     Ҫ    ɫ     ߼ 
        //    磬    ʹ  HSV  ɫ ռ      ɾ  ȷֲ     ɫ
        for (int i = 0; i < Graph_Count; ++i) {
            float hue = static_cast<float>(i) / (Graph_Count - 1) * 360.0f; //       ӳ 䵽0-360 ȵ HSVɫ    
            QColor color = QColor::fromHsvF(hue / 360.0f, 1.0f, 1.0f); //     HSV  ɫ
            colorList.append(color);
        }

        for(uint8_t i = 0;i<Graph_Count;i++)
        {
            CustomPlot->addGraph();

            CustomPlot->graph(i)->setPen(QPen(colorList[(Graph_Count- 1 - i) % colorList.size()]));
            //CustomPlot->graph(i)->setPen(QPen(Qt::green));

            CustomPlot->setAntialiasedElements(QCP::aeNone);

             //   û ͼ  ʽ     ٻ   ʱ  
            CustomPlot->graph(i)->setLineStyle(QCPGraph::lsNone);
            CustomPlot->graph(i)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 1));
            CustomPlot->graph(i)->setName(Qcustomplot_display_Name[i]);
        }

    }



    // ??????????????????  
    //QCPTextElement *title = CustomPlot->plotLayout()->title();
    //QFont titleFont = title->font();
    //titleFont.setPointSize(12); // ??????????????  ?????12??
    //title->setFont(titleFont);



    // ????x?????????????????
    QFont axisLabelFont = CustomPlot->xAxis->labelFont(); // ???x?????????????
    axisLabelFont.setPointSize(10); // ??x?????????????  ?????10??
    CustomPlot->xAxis->setLabelFont(axisLabelFont); // ??????????x?????
    CustomPlot->xAxis->setTickLabelFont(axisLabelFont); // ??????????x???????

    // ????y?????????????????
    axisLabelFont = CustomPlot->yAxis->labelFont(); // ???y?????????????
    axisLabelFont.setPointSize(10); // ??y?????????????  ?????10??
    CustomPlot->yAxis->setLabelFont(axisLabelFont); // ??????????y?????
    CustomPlot->yAxis->setTickLabelFont(axisLabelFont); // ??????????y???????

    // ????y?????????????????
    axisLabelFont = CustomPlot->yAxis2->labelFont(); // ???y?????????????
    axisLabelFont.setPointSize(10); // ??y?????????????  ?????10??
    CustomPlot->yAxis2->setLabelFont(axisLabelFont); // ??????????y?????
    CustomPlot->yAxis2->setTickLabelFont(axisLabelFont); // ??????????y???????


    if(REAL_LOG != ArrayNumber)
    {
        // ????????????  ??????
        QFont font;
        font.setPixelSize(13);                  // ?????????????  ?????13????
        CustomPlot->legend->setFont(font);      // ?????????????
        CustomPlot->legend->setVisible(true);   // ???????
    }
   /* else
    {
        QFont font;
        font.setPixelSize(13);                  // ?????????????  ?????13????
        CustomPlot->legend->setFont(font);      // ?????????????
        CustomPlot->legend->setVisible(true);   // ???????

        for(uint8_t i = 0;i<Graph_Count;i++)
        {
            if((Qstring_Qcustomplot_display_Name[i] == "TC1")
                    ||(Qstring_Qcustomplot_display_Name[i] == "TC2")
                    ||(Qstring_Qcustomplot_display_Name[i] == "TC3")
                    ||(Qstring_Qcustomplot_display_Name[i] == "TC4")
                    ||(Qstring_Qcustomplot_display_Name[i] == "TC5")
                    ||(Qstring_Qcustomplot_display_Name[i] == "TC6")
                    ||(Qstring_Qcustomplot_display_Name[i] == "Th1")
                    ||(Qstring_Qcustomplot_display_Name[i] == "Th2")
                    ||(Qstring_Qcustomplot_display_Name[i] == "Th3"))
            {
                CustomPlot->legend->item(i)->setVisible(true);
            }
            else
            {
                CustomPlot->legend->item(i)->setVisible(false);
            }

        }
    }*/




    if(TCR_T == ArrayNumber)
    {
        // ????x????y??????
        CustomPlot->xAxis->setLabel(xAxis_Label); // ????x????????xAxis_Label??????????????????
        CustomPlot->yAxis->setLabel(yAxis_Label); // ????y????????yAxis_Label??????????????????

        // ????x????y?????  
        CustomPlot->xAxis->setRange(0, xAxis_Range); // ????x?????  ??xAxis_Range????    ???????
        CustomPlot->yAxis->setRange(0, yAxis_Range); // ????y?????  ??yAxis_Range????    ???????
    }
    else if(PROFILE == ArrayNumber)
    {
        CustomPlot->yAxis2->setVisible(true);


        CustomPlot->graph(0)->setValueAxis(CustomPlot->yAxis2);
        CustomPlot->graph(2)->setValueAxis(CustomPlot->yAxis2);

        // ????x????y??????
        CustomPlot->xAxis->setLabel(xAxis_Label); // ????x????????xAxis_Label??????????????????
        CustomPlot->yAxis->setLabel(yAxis_Label); // ????y????????yAxis_Label??????????????????
        CustomPlot->yAxis2->setLabel("Power(W)/Voltage(V)");



        // ????x????y?????  
        CustomPlot->xAxis->setRange(0, xAxis_Range); // ????x?????  ??xAxis_Range????    ???????
        CustomPlot->yAxis->setRange(0, yAxis_Range); // ????y?????  ??yAxis_Range????    ???????
        CustomPlot->yAxis2->setRange(0,THIRD_QCUSTOMPLOT_YAXIS2);   //          ѡ  


    }
    else if(REAL_LOG == ArrayNumber)
    {
        CustomPlot->yAxis2->setVisible(true);

        for(uint8_t i = 0;i<Graph_Count;i++)
        {
            if(((Qcustomplot_display_Name[i] != "TC1")
                &&(Qcustomplot_display_Name[i] != "TC2")
                &&(Qcustomplot_display_Name[i] != "Th1")
                &&(Qcustomplot_display_Name[i] != "Th2")))
            {
                CustomPlot->graph(i)->setValueAxis(CustomPlot->yAxis2);
            }

        }

        // ????x????y??????
        CustomPlot->xAxis->setLabel(xAxis_Label); // ????x????????xAxis_Label??????????????????
        CustomPlot->yAxis->setLabel(yAxis_Label); // ????y????????yAxis_Label??????????????????
        CustomPlot->yAxis2->setLabel("Voltage(V)/Current(A)/Resistance(Ohm)/Duty");

        // ????x????y?????  
        CustomPlot->xAxis->setRange(0, xAxis_Range); // ????x?????  ??xAxis_Range????    ???????
        CustomPlot->yAxis->setRange(0, yAxis_Range); // ????y?????  ??yAxis_Range????    ???????
        CustomPlot->yAxis2->setRange(0,THIRD_QCUSTOMPLOT_YAXIS2);   //          ѡ  

    }
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::C_serialPortReadYRead_slot()
{
    QByteArray newData = C_Serialport_Bat->readAll();
    Uart_Actual_info.append(newData);

    while (canExtractFrame(Uart_Actual_info))
    {
        QByteArray frame = extractFrame(Uart_Actual_info);

        // 【关键修复 1】：增加安全边界检查，绝对防止把超过 1000 字节的数据塞入数组导致内存崩溃！
        int copySize = qMin(frame.size(), 1000);
        for (int i = 0; i < copySize; ++i)
        {
            ReceiveDataArray[i] = frame.at(i);
        }

        if (!Uart_Flash_Data_Check(ReceiveDataArray, copySize))
        {
            Rec_Data_Proc(ReceiveDataArray);
        }
        else
        {
            qDebug() << "CRC mismatch, discarding frame.";
        }
    }
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/

void Widget::Rec_Data_Proc(uint8_t *Rec)
{
    switch(Rec[4])
    {
        case READ_BOARD_STATE:
            {
                BAT_Device_State_Count = REC_CHECK_TIME;

                if(Rec[10] != 4)
                {
                    //     QLabel  ʾͼƬ
                    QPixmap pixmap2(":/Green.png"); //  滻Ϊ    ͼƬ·???
                    ui->BAT_Device_State_light->setPixmap(pixmap2);
                    // ȷ  ͼƬ         ŵ QLabel  С
                    ui->BAT_Device_State_light->setScaledContents(true);
                }
                else
                {
                    //     QLabel  ʾͼƬ
                    QPixmap pixmap2(":/Red.png"); //  滻Ϊ    ͼƬ·???
                    ui->BAT_Device_State_light->setPixmap(pixmap2);
                    // ȷ  ͼƬ         ŵ QLabel  С
                    ui->BAT_Device_State_light->setScaledContents(true);
                }

                //QString versionInfo = "HW:" + QString::number(Rec[7]) + "." + QString::number(Rec[8]) + " " +
                                      //"SW:" + QString::number(Rec[5]) + "." + QString::number(Rec[6]);
                QString versionInfo = "HW:" + QString::number(1) + "." + QString::number(0) + " " +
                                      "SW:" + QString::number(1) + "." + QString::number(0) + "." + QString::number(0);
                ui->BAT_Version_Rom->setText(versionInfo);


                if(pre_error_code != Rec[11] && Rec[11] != 0) {
                    // 先把之前的图表清空，防止 X 坐标倒流引发崩溃
                    on_BAT_Clear_Screen_clicked();

                    Board_data.heat_profile_time_flag = 0;
                    Board_data.heat_tcrt_time_flag = 0;
                    Log_For_Act_display_count = 0;

                }
                //Board_data.heat_profile_time_flag = 0;
                //Board_data.heat_tcrt_time_flag = 0;
                //Log_For_Act_display_count = 0;


                Middle_Num.setNum(Rec[9]);


                if(pre_error_code != Rec[11])
                {
                    Middle_Num.setNum(Rec[11],16);

                    //ui->BAT_SerialData_display->appendPlainText("Now error state is " + Middle_Num);

                    if(Rec[11] == (0x01))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[0]);
                    }
                    else if(Rec[11] == (0x02))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[1]);
                    }
                    else if(Rec[11] == (0x03))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[2]);
                    }
                    else if(Rec[11] == (0x04))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[3]);
                    }
                    else if(Rec[11] == (0x05))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[4]);
                    }
                    else if(Rec[11] == (0x06))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[5]);
                    }
                    else if(Rec[11] == (0x07))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[6]);
                    }
                    else if(Rec[11] == (0x08))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[7]);
                    }
                    else if(Rec[11] == (0x09))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[8]);
                    }
                    else if(Rec[11] == (0x10))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[9]);
                    }
                    else if(Rec[11] == (0x11))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[10]);
                    }
                    else if(Rec[11] == (0x12))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[11]);
                    }
                    else if(Rec[11] == (0x21))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[12]);
                    }
                    else if(Rec[11] == (0x22))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[13]);
                    }
                    else if(Rec[11] == (0x23))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[14]);
                    }
                    else if(Rec[11] == (0x24))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[15]);
                    }
                    else if(Rec[11] == (0x25))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[16]);
                    }
                    else if(Rec[11] == (0x26))
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is " + Qstring_error_Log_Name[17]);
                    }
                    else
                    {
                        ui->BAT_SerialData_display->appendPlainText("Now error state is 0x00 now state is normal");
                    }

                }
                pre_error_code = Rec[11];



            }
            break;

        case READ_LOG_PROCESS:
            {
                 if(Board_data.heat_profile_time_flag == 0)
                 {
                     //     QLabel  ʾͼƬ
                     QPixmap pixmap2(":/Green.png"); //  滻Ϊ    ͼƬ·???
                     ui->BAT_Device_State_light->setPixmap(pixmap2);
                     // ȷ  ͼƬ         ŵ QLabel  С
                     ui->BAT_Device_State_light->setScaledContents(true);

                     Board_data.heat_profile_time_flag = 1;
                     a_log_flag_set();
                     a_log_flag_set_ui();
                     Log_For_Act_display_count = 0;
                     Board_data.heat_time_count = 0;
                     heattimes.clear();

                     if(Board_data.heat_tcrt_time_flag != 1)
                     {
                        QDateTime current_date_time =QDateTime::currentDateTime();
                        QString current_date ="BAT_Heat_Log_Data_" + current_date_time.toString("yyyy_MM_dd_HH_mm_ss");
                        QString path = QCoreApplication::applicationDirPath();


                        strPath = path +"/TestData"+"./" + current_date + ".csv";
                        qDebug() << "path2delect = " << strPath;

                     }
                 }

                if(Board_data.heat_tcrt_time_flag == 1)
                {
                    Board_data.heat_time_count ++;

                    if(1 ==Board_data.Sampling_period)
                    {
                        Middle_Num.setNum(((double) Board_data.heat_time_count/50.0));
                    }
                    else if(2 ==Board_data.Sampling_period)
                    {
                        Middle_Num.setNum(((double) Board_data.heat_time_count/25.0));
                    }
                    else if(5 ==Board_data.Sampling_period)
                    {
                        Middle_Num.setNum(((double) Board_data.heat_time_count/10.0));
                    }
                    else if(10 ==Board_data.Sampling_period)
                    {
                        Middle_Num.setNum(((double) Board_data.heat_time_count/5.0));
                    }


                }


                if(Board_data.heat_profile_time_flag == 1)
                {

                    if(Board_data.heat_tcrt_time_flag != 1)
                    {
                        Board_data.heat_time_count ++;

                        if(1 ==Board_data.Sampling_period)
                        {
                            Middle_Num.setNum(((double) Board_data.heat_time_count/50.0));
                        }
                        else if(2 ==Board_data.Sampling_period)
                        {
                            Middle_Num.setNum(((double) Board_data.heat_time_count/25.0));
                        }
                        else if(5 ==Board_data.Sampling_period)
                        {
                            Middle_Num.setNum(((double) Board_data.heat_time_count/10.0));
                        }
                        else if(10 ==Board_data.Sampling_period)
                        {
                            Middle_Num.setNum(((double) Board_data.heat_time_count/5.0));
                        }

                        ui->BAT_profile_heat_time->setText(Middle_Num);

                    }
                }



                BAT_Device_State_Count = REC_CHECK_TIME;

                QString Read_log;
                uint8_t log_count = 5;
                Log_For_Act_display[0][Log_For_Act_display_count] = Log_For_Act_display_count;

                for(uint8_t j = 0;j<23;j++)
                {
                    if(1 == Log_For_PC[j])
                    {
                        if(j == 0)
                        {
                            QDateTime current_date_time =QDateTime::currentDateTime();
                            Read_log =  current_date_time.toString("yyyy_MM_dd_hh_mm_ss.zzz") + tr(",");
                           heattimes.append(current_date_time.toString("yyyy_MM_dd_hh_mm_ss.zzz"));
                           log_count += 8;

                        }
                        else
                        {
                            Read_log =  Read_log +  Qstring_Save_Log_Name[j] + tr("=");

                              double data_rec;
                              if((j>15)&&(j < 24))
                              {
                                  uint16_t recuintdata = (uint16_t)Rec[log_count++]*256;
                                  recuintdata = recuintdata + Rec[log_count++];

                                  int16_t recData = (uint16_t)recuintdata;
                                  data_rec = recData;
                              }
                              else
                              {
                                  data_rec = (int16_t)Rec[log_count++]*256;
                                  data_rec = data_rec + (int16_t)Rec[log_count++];
                              }

                              if((j == 9)||(j == 15))
                              {

                              }
                              else if((j == 2)||(j == 3)||(j == 16)||(j == 17)||(j == 18)||(j == 19)||(j == 20)||(j == 21)||(j == 22)||(j == 23))
                              {
                                  data_rec = data_rec / 10.0;
                              }
                              else if((j == 7)||(j == 13))
                              {
                                  data_rec = data_rec / 100.0;
                              }
                              else
                              {
                                  data_rec = data_rec / 1000.000;
                              }

                              if((j>15)&&(j < 24))
                              {

                              }
                              else
                              {
                                  if(data_rec > 500)
                                  {
                                      data_rec = 0;
                                  }
                              }

                            Read_log =  Read_log + QString("%1").arg(data_rec) + tr(",");



                            Log_For_Act_display[j][Log_For_Act_display_count] = data_rec;
                        }

                    }
                    else
                    {
                        Log_For_Act_display[j][Log_For_Act_display_count] = 0;
                    }
                }


                Log_For_Act_display_count ++;
                if ((SERIAL_LOG_UI_INTERVAL <= 1)
                    || (Log_For_Act_display_count % SERIAL_LOG_UI_INTERVAL == 1))
                {
                    a_BAT_SerialData_display_Set(Read_log,QPLAINTEXTEDIT_APPEND);
                }
                QCustomplot_Display_Log_Proc(ui->BAT_customplot_Heat,Log_For_Act_display_count);

                 if(Board_data.heat_tcrt_time_flag == 1)
                 {
                    if(Log_For_Act_display_count > 2999)
                    {
                        if(Log_For_Act_display_count % 30000 == 1)
                        {
                            ui->BAT_SerialData_display->clear();
                        }
                    }
                 }
                 else
                 {
                     if(Log_For_Act_display_count > 2999)
                     {
                         if(Log_For_Act_display_count % 30000 == 1)
                         {
                             ui->BAT_SerialData_display->clear();
                         }
                     }
                 }


                 /*if(Board_data.heat_tcrt_time_flag == 1)
                 {
                     if(Board_data.heat_time_count % 5 == 0)
                     {
                        Log_For_Act_display_count ++;
                        QCustomplot_Display_Log_Proc(ui->BAT_customplot_Heat,Log_For_Act_display_count);
                        if(Log_For_Act_display_count > 2999)
                        {
                            if(Log_For_Act_display_count % 5000 == 1)
                            {
                                ui->BAT_SerialData_display->clear();
                            }
                        }
                     }

                 }
                 else
                 {
                     //a_BAT_SerialData_display_Set(Read_log,QPLAINTEXTEDIT_APPEND);
                     Log_For_Act_display_count ++;
                     QCustomplot_Display_Log_Proc(ui->BAT_customplot_Heat,Log_For_Act_display_count);
                     if(Log_For_Act_display_count > 2999)
                     {
                         if(Log_For_Act_display_count % 30000 == 1)
                         {
                             ui->BAT_SerialData_display->clear();
                         }
                     }
                 }*/


            }
            break;

        case READ_REAL_TIME_LOG:
            {
                for(uint8_t j = 0;j<17;j++)
                {
                    Log_For_PC[j] = Rec[j+5];

                    if(Log_For_PC[j]!= 0)
                    {
                        if(j == 0)
                        {
                            Log_Heat_Act_Number = Log_Heat_Act_Number + 8;
                        }
                        else
                        {
                            Log_Heat_Act_Number = Log_Heat_Act_Number + 2;
                        }
                    }
                }

                a_log_flag_set_ui();
                a_Read_board_basic_info_Get();


            }
            break;

            case WRITE_REAL_TIME_LOG:
                {
                    if(0x01 == Rec[5])
                    {
                         a_Uart_Write_Board_baisic_info();

                         qDebug()  <<  "Scceed";
                    }
                    else
                    {
                        a_Write_Real_Time_Log_Set();
                        qDebug()  <<  "Fail";
                    }
                }
                break;

            case GET_BOARD_BASIC_INFO:
                {
                    a_Board_Base_Check_And_Set_ui();
                    a_Uart_Read_H1_Setting();
                }
                break;

            case SET_BOARD_BASIC_INFO:
                {
                    if(0x01 == Rec[5])
                    {
                         a_Uart_Write_H1_Setting();
                         qDebug()  <<  "Scceed";
                    }
                    else
                    {
                        qDebug()  <<  "Fail";
                        a_Uart_Write_Board_baisic_info();
                    }

                }
                break;


            case SYNC_PC_TIMETO_DEVICE:
                {

                }
                break;

            case READ_H_SETTING:
                {
                    if(H1_PORT == Rec[5])
                    {
                        a_H1_Data_Check_And_Set_ui();
                        a_Uart_Read_H2_Setting();

                    }
                    if(H2_PORT == Rec[5])
                    {
                        a_H2_Data_Check_And_Set_ui();
                        a_Uart_Read_H3_Setting();

                    }
                    if(H3_PORT == Rec[5])
                    {
                        a_H3_Data_Check_And_Set_ui();
                        a_Uart_Read_H1_Profile();
                    }
                }
                break;



            case READ_H_PROFILE:
                {
                    if(H1_PORT == Rec[5])
                    {
                        a_Uart_Read_H2_Profile();
                        H1_data.Profile_count = Rec[6];


                        for(uint8_t init_count = 0;init_count < H1_data.Profile_count;init_count++)
                        {
                            H1_data.Profile_Heat_Start_Time[init_count] = (Rec[7 + 8*init_count]*256 + Rec[8 + 8*init_count]);
                            H1_data.Profile_Heat_Stop_Time[init_count] = (Rec[9 + 8*init_count]*256 + Rec[10 + 8*init_count]);

                            if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 1)
                            {
                                H1_data.Profile_Heat_Type[init_count] = "P";
                            }
                            else if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 2)
                            {
                                H1_data.Profile_Heat_Type[init_count] = "T";
                            }
                            else if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 3)
                            {
                                H1_data.Profile_Heat_Type[init_count] = "V";
                            }
                            else
                            {
                                H1_data.Profile_Heat_Type[init_count] = "0";
                            }

                            H1_data.Profile_Heat_Number[init_count] = ((Rec[13 + 8*init_count]*256 + Rec[14 + 8*init_count]) / 100.00);
                        }
                        a_Array_Data_Tran_TableWidget(ui->BAT_Table_Set_Profile1,&H1_data,PROFILE);
                        on_BAT_Display_Table_Set_Profile1_clicked();

                        H1_data.Profile_count = 0;

                    }
                    if(H2_PORT == Rec[5])
                    {
                        a_Uart_Read_H3_Profile();
                        H2_data.Profile_count = Rec[6];

                        for(uint8_t init_count = 0;init_count < H2_data.Profile_count;init_count++)
                        {
                            H2_data.Profile_Heat_Start_Time[init_count] = (Rec[7 + 8*init_count]*256 + Rec[8 + 8*init_count]);
                            H2_data.Profile_Heat_Stop_Time[init_count] = (Rec[9 + 8*init_count]*256 + Rec[10 + 8*init_count]);
                            if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 1)
                            {
                                H2_data.Profile_Heat_Type[init_count] = "P";
                            }
                            else if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 2)
                            {
                                H2_data.Profile_Heat_Type[init_count] = "T";
                            }
                            else if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 3)
                            {
                                H2_data.Profile_Heat_Type[init_count] = "V";
                            }
                            else
                            {
                                H2_data.Profile_Heat_Type[init_count] = "0";
                            }
                            H2_data.Profile_Heat_Number[init_count] =  ((Rec[13 + 8*init_count]*256 + Rec[14 + 8*init_count]) / 100.00);
                        }
                        a_Array_Data_Tran_TableWidget(ui->BAT_Table_Set_Profile2,&H2_data,PROFILE);
                        on_BAT_Display_Table_Set_Profile2_clicked();

                        H2_data.Profile_count = 0;

                    }
                    if(H3_PORT == Rec[5])
                    {
                        H3_data.Profile_count = Rec[6];
                        for(uint8_t init_count = 0;init_count < H3_data.Profile_count;init_count++)
                        {
                            H3_data.Profile_Heat_Start_Time[init_count] = (Rec[7 + 8*init_count]*256 + Rec[8 + 8*init_count]);
                            H3_data.Profile_Heat_Stop_Time[init_count] = (Rec[9 + 8*init_count]*256 + Rec[10 + 8*init_count]);
                            if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 1)
                            {
                                H3_data.Profile_Heat_Type[init_count] = "P";
                            }
                            else if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 2)
                            {
                                H3_data.Profile_Heat_Type[init_count] = "T";
                            }
                            else if((Rec[11 + 8*init_count]*256 + Rec[12 + 8*init_count]) == 3)
                            {
                                H3_data.Profile_Heat_Type[init_count] = "V";
                            }
                            else
                            {
                                H3_data.Profile_Heat_Type[init_count] = "0";
                            }
                            H3_data.Profile_Heat_Number[init_count] =  ((Rec[13 + 8*init_count]*256 + Rec[14 + 8*init_count]) / 100.00);
                        }
                        on_BAT_Display_Table_Set_Profile3_clicked();
                        QMessageBox::information(this,"state","Heat data read succeed");

                        H3_data.Profile_count = 0;

                    }

                }
                break;

            case READ_H_TCR_TABLE:
                {


                    /*
                    if(H1_PORT == Rec[5])
                    {
                        ui->BAT_Table_Heat_Seletion->setCurrentIndex(0); //     Ϊ ڶ   ѡ       0  ʼ  
                    }
                    if(H2_PORT == Rec[5])
                    {
                        ui->BAT_Table_Heat_Seletion->setCurrentIndex(1); //     Ϊ ڶ   ѡ       0  ʼ  

                    }
                    if(H3_PORT == Rec[5])
                    {
                        ui->BAT_Table_Heat_Seletion->setCurrentIndex(2); //     Ϊ ڶ   ѡ       0  ʼ  
                    }

                    uint16_t Init_T0,Init_R0,Cali_Start_Temp = 0,Cali_Stop_Temp = 0,Cali_step = 0;
                    double DInit_R0 = 0;

                    H1_data.TCRT_count = Rec[6];
                    qDebug()<< "H1_data.TCRT_count" <<H1_data.TCRT_count;

                    Init_T0 = Rec[7]*256 + Rec[8];
                    Init_R0 = Rec[9]*256 + Rec[10];
                    Cali_Start_Temp = Rec[11]*256 + Rec[12];
                    Cali_Stop_Temp = Rec[13]*256 + Rec[14];
                    Cali_step = Rec[15]*256 + Rec[16];


                    Middle_Num.setNum((((float)Init_T0)/10.0));
                    ui->BAT_Init_TempT0->setText(Middle_Num);
                    qDebug()<< "Init_T0" <<Init_T0;


                    Middle_Num.setNum((((float)Init_R0)/1000.000));
                    ui->BAT_Init_ResiR0->setText(Middle_Num);
                    qDebug()<< "Init_R0" <<Init_R0;

                    DInit_R0 = Init_R0/1000.000;
                    Middle_Num.setNum((Cali_Start_Temp));
                    ui->BAT_Start_Cali_Temp->setText(Middle_Num);

                    Middle_Num.setNum((Cali_Stop_Temp));
                    ui->BAT_Stop_Cali_Temp->setText(Middle_Num);

                    Middle_Num.setNum((Cali_step));
                    ui->BAT_Start_Cali_Step->setText(Middle_Num);


                    for(uint8_t init_count = 0;init_count < H1_data.TCRT_count;init_count++)
                    {
                        H1_data.TCR_Temp[init_count] = (Rec[17 + 4*init_count]*256 + Rec[18 + 4*init_count]);
                        H1_data.TCR_TCR[init_count] = (((Rec[19 + 4*init_count]*256 + Rec[20 + 4*init_count]))/1000000.000000);
                        uint16_t middle_t = H1_data.TCR_Temp[init_count] - Init_T0/10;

                        H1_data.TCR_Resi[init_count] = ((double)(DInit_R0 + (DInit_R0 * middle_t   * (H1_data.TCR_TCR[init_count]))));
                        qDebug()<< "H1_data.TCR_Temp[init_count]" <<H1_data.TCR_Temp[init_count];
                        qDebug()<< "H1_data.TCR_TCR[init_count]" <<H1_data.TCR_TCR[init_count];
                        qDebug()<< "H1_data.TCR_Resi[init_count]" <<H1_data.TCR_Resi[init_count];


                    }

                    H1_data.Middle_count = H1_data.TCRT_count;

                    a_Array_Data_Tran_TableWidget(ui->BAT_Table_Init_Tcr,&H1_data,TCR_T);
                    QCustomplot_Display_Data_Proc(ui->BAT_Table_Init_Tcr,ui->BAT_customplot_Init_Tcr,TCR_T);

                    if(Board_data.heat_tcrt_time_flag == 1)
                    {
                        a_Save_TableWidget_Tcr();
                    }*/

                    H1_data.TCRT_count = 0;
                }
                break;


            case WRITE_H_SETTING:
                {
                    if(H1_PORT == Rec[5])
                    {
                        a_Uart_Write_H2_Setting();
                    }
                    if(H2_PORT == Rec[5])
                    {
                        a_Uart_Write_H3_Setting();
                    }
                    if(H3_PORT == Rec[5])
                    {
                        a_Uart_Write_H1_Profile();
                    }


                }
                break;

            case WRITE_H_TCR_TABLE:
                {
                    if(H1_PORT == Rec[5])
                    {
                        QMessageBox::information(this,"state","H1 write succeed");
                    }
                    if(H2_PORT == Rec[5])
                    {
                        QMessageBox::information(this,"state","H2 write succeed");
                    }
                    if(H3_PORT == Rec[5])
                    {
                        QMessageBox::information(this,"state","H3 write succeed");
                    }
                }
                break;

            case WRITE_H_PROFILE:
                {
                    if(H1_PORT == Rec[5])
                    {
                        a_Uart_Write_H2_Profile();
                    }
                    if(H2_PORT == Rec[5])
                    {
                        a_Uart_Write_H3_Profile();

                    }
                    if(H3_PORT == Rec[5])
                    {
                        QMessageBox::information(this,"state","Heat data write succeed");
                    }

                }
                break;

            case BOARD_START_HEAT:
                {
                    if(1 == Rec[5])
                    {
                         Board_data.heat_profile_time_flag = 1;
                         Board_data.heat_tcrt_time_flag = 0;
                         display_paintEvent_count = 0;

                         Log_For_Act_display_count = 0;
                         Board_data.heat_time_count = 0;
                         heattimes.clear();
                         on_BAT_Clear_Screen_clicked();
                         s_heatGraphVisibilityApplied = false;

                         a_log_flag_set();
                         a_log_flag_set_ui();

                         QDateTime current_date_time =QDateTime::currentDateTime();
                         QString current_date ="BAT_Heat_Log_Data_" + current_date_time.toString("yyyy_MM_dd_HH_mm_ss");

                         QString path = QCoreApplication::applicationDirPath();

                         strPath = path +"/TestData"+"./" + current_date + ".csv";
                         qDebug() << "startheatpath2 = " << strPath;

                        //QMessageBox::information(this,"state","start heat");


                        std::fill(LOG_Time.begin(), LOG_Time.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
                    }
                    else
                    {
                        ui->BAT_SerialData_display->setPlainText("Heater working!!!!!!!!!!!!!!!!");
                    }

                }
                break;

            case BOARD_STOP_HEAT:
                {


                     Board_data.heat_profile_time_flag = 0;
                     display_paintEvent_count = 0;

                     const uint32_t saveCount = Log_For_Act_display_count;
                     if (saveCount > 0)
                     {
                         if (strPath.isEmpty())
                         {
                             QDateTime current_date_time = QDateTime::currentDateTime();
                             QString current_date = "BAT_Heat_Log_Data_"
                                     + current_date_time.toString("yyyy_MM_dd_HH_mm_ss");
                             QString path = QCoreApplication::applicationDirPath();
                             strPath = path + "/TestData/./" + current_date + ".csv";
                         }
                         QDir().mkpath(QFileInfo(strPath).absolutePath());
                         qDebug() << "stopandsavecsv2:";

                         Board_Save_Log_File_In_Pc(strPath, saveCount);
                         SaveImuResultFileInPc(buildImuResultPath(strPath), saveCount);
                     }

                     Board_data.heat_tcrt_time_flag = 0;
                     Log_For_Act_display_count = 0;
                     Board_data.heat_time_count = 0;
                     heattimes.clear();
                     Uart_Actual_info.clear();
                     ui->BAT_SerialData_display->appendPlainText("Stop heat!!!!!!!!!!!!!!!!");

                     std::fill(LOG_Time.begin(), LOG_Time.end(), 0);


                }


                break;

            case BOARD_MYSELF_CALI_START:
                {


                }
                break;

            case BOARD_MYSELF_CALI_STOP:
                {



                }
                break;

            case ASK_EXPORT_LOG:
                {



                }
                break;

        case 0xC0:
            {



            }
            break;

        case 0xD0:
            {



            }
            break;
            default:
            break;
    }
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::QCustomplot_Display_Log_Proc(QCustomPlot *CustomPlot,uint32_t ArrayNumber)
{

     uint8_t log_act_count = 0;

    //LOG_Time[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1]/10;
    display_paintEvent_number = REALTIME_REPLOT_INTERVAL_POINTS;
    const int sampleHz = logSamplingRateHz();
    const double timeRaw = Log_For_Act_display[log_act_count++][ArrayNumber - 1];
    const double tPlotCandidate = (sampleHz > 0) ? (timeRaw / sampleHz) : 0.0;

    LOG_Time[ArrayNumber-1] = tPlotCandidate;
    LOG_Vin[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];


    LOG_TC1[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_TC2[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];


    LOG_Rh1[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Vh1[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Ah1[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Acp1[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Dty1[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Err1[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];


    LOG_Rh2[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Vh2[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Ah2[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Acp2[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Dty2[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Err2[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];

    LOG_Accel_x[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Accel_y[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Accel_z[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];

    //updateWindowedData();

    LOG_Gyro_x[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Gyro_y[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    LOG_Gyro_z[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];
    processData(LOG_Accel_x[ArrayNumber-1], LOG_Accel_y[ArrayNumber-1], LOG_Accel_z[ArrayNumber-1],LOG_Gyro_x[ArrayNumber-1], LOG_Gyro_y[ArrayNumber-1], LOG_Gyro_z[ArrayNumber-1]);

    LOG_Temp_c[ArrayNumber-1] = Log_For_Act_display[log_act_count++][ArrayNumber-1];







    log_act_count = 0;


    const int graphCount = CustomPlot ? CustomPlot->graphCount() : -1;
    const double tPlot = LOG_Time[ArrayNumber - 1];
    const double yValues[] = {
        LOG_Vin[ArrayNumber - 1],
        LOG_TC1[ArrayNumber - 1],
        LOG_TC2[ArrayNumber - 1],
        LOG_Rh1[ArrayNumber - 1],
        LOG_Vh1[ArrayNumber - 1],
        LOG_Ah1[ArrayNumber - 1],
        LOG_Dty1[ArrayNumber - 1],
        LOG_Rh2[ArrayNumber - 1],
        LOG_Vh2[ArrayNumber - 1],
        LOG_Ah2[ArrayNumber - 1],
        LOG_Dty2[ArrayNumber - 1]
    };
    bool badValue = false;

    CustomPlot->graph(0)->addData(tPlot, LOG_Vin[ArrayNumber - 1]);
    CustomPlot->graph(1)->addData(tPlot, LOG_TC1[ArrayNumber - 1]);
    CustomPlot->graph(2)->addData(tPlot, LOG_TC2[ArrayNumber - 1]);
    CustomPlot->graph(3)->addData(tPlot, LOG_Rh1[ArrayNumber - 1]);
    CustomPlot->graph(4)->addData(tPlot, LOG_Vh1[ArrayNumber - 1]);
    CustomPlot->graph(5)->addData(tPlot, LOG_Ah1[ArrayNumber - 1]);
    CustomPlot->graph(6)->addData(tPlot, LOG_Dty1[ArrayNumber - 1]);
    CustomPlot->graph(7)->addData(tPlot, LOG_Rh2[ArrayNumber - 1]);
    CustomPlot->graph(8)->addData(tPlot, LOG_Vh2[ArrayNumber - 1]);
    CustomPlot->graph(9)->addData(tPlot, LOG_Ah2[ArrayNumber - 1]);
    CustomPlot->graph(10)->addData(tPlot, LOG_Dty2[ArrayNumber - 1]);
    for (int g = 0; g <= 10; ++g)
        CustomPlot->graph(g)->data()->removeBefore(tPlot - HEAT_PLOT_WINDOW_SEC);

    display_paintEvent_count++;
    if (display_paintEvent_count > display_paintEvent_number)
    {
        display_paintEvent_count = 0;
        CustomPlot->xAxis->setRange(qMax(0.0, tPlot - HEAT_PLOT_WINDOW_SEC), qMax(1.0, tPlot));
        CustomPlot->yAxis->setRange(0, 500);

        if (!s_heatGraphVisibilityApplied || ArrayNumber <= 1)
        {
            s_heatGraphVisibilityApplied = true;
            uint8_t log_act_count = 0;

        if(true == ui->PPS_Volt_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }




        if(true == ui->TC1_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }

        if(true == ui->TC2_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }

        if(true == ui->HS1_Resi_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }



        if(true == ui->HS1_Vh1_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }

        if(true == ui->HS1_Ah1_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }

        if(true == ui->HS1_Duty_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }


        if(true == ui->HS2_Resi_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }



        if(true == ui->HS2_Vh2_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }

        if(true == ui->HS2_Ah2_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }


        if(true == ui->HS2_Duty_Log_State_act->isChecked())
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
            Log_For_Act_PC[log_act_count++] = 0x01;
        }
        else
        {
            ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
            Log_For_Act_PC[log_act_count++] = 0x00;
        }
        }

        CustomPlot->replot(QCustomPlot::rpQueuedReplot);
    }
}


/**************************************************************************
 * ????????
 * ????  ??
 *
 * ????  ????
 * ????  ????
 *************************************************************************/


void Widget::Board_Save_Log_File_In_Pc(const QString &fileName, uint32_t ArrayNumber)
{



    if (fileName.isEmpty() || ArrayNumber == 0)
        return;

    QFile file(fileName);

    //      ļ  Ƿ    ڣ           ɾ  
    if (file.exists())
    {
        if (!file.remove())
        {
            qWarning() << "Failed to delete existing file:" << fileName;
            return;
        }
        qDebug() << "Deleted existing file:" << fileName;
    }


    //bool OKfile = file.open(QIODevice::WriteOnly|QIODevice::Append);
    bool OKfile = file.open(QIODevice::WriteOnly|QIODevice::Text);

    if(OKfile)
    {
        QTextStream out(&file);

        for(uint8_t Count_Local1 = 0;Count_Local1<29;Count_Local1++)
        {
            out<<Qstring_Save_Log_Name[Count_Local1]<<",";
        }
        out<<"\n";

        for(uint32_t Count_Local3 = 0;Count_Local3 < ArrayNumber;Count_Local3++)
        {
            out<< heattimes.at((0)+Count_Local3) <<",";



            out<<QString("%1").arg(LOG_Vin[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_TC1[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_TC2[Count_Local3])<<",";



            out<<QString("%1").arg(LOG_Rh1[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Vh1[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Ah1[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Acp1[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Dty1[Count_Local3])<<",";




            if(LOG_Err1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                if(LOG_Err1[Count_Local3] == (0x01))
                {
                    out<<Qstring_error_Log_Name[0]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x02))
                {
                    out<<Qstring_error_Log_Name[1]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x03))
                {
                    out<<Qstring_error_Log_Name[2]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x04))
                {
                    out<<Qstring_error_Log_Name[3]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x05))
                {
                    out<<Qstring_error_Log_Name[4]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x06))
                {
                    out<<Qstring_error_Log_Name[5]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x07))
                {
                    out<<Qstring_error_Log_Name[6]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x08))
                {
                    out<<Qstring_error_Log_Name[7]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x09))
                {
                    out<<Qstring_error_Log_Name[8]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x10))
                {
                    out<<Qstring_error_Log_Name[9]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x11))
                {
                    out<<Qstring_error_Log_Name[10]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x12))
                {
                    out<<Qstring_error_Log_Name[11]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x21))
                {
                    out<<Qstring_error_Log_Name[12]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x22))
                {
                    out<<Qstring_error_Log_Name[13]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x23))
                {
                    out<<Qstring_error_Log_Name[14]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x24))
                {
                    out<<Qstring_error_Log_Name[15]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x25))
                {
                    out<<Qstring_error_Log_Name[16]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x26))
                {
                    out<<Qstring_error_Log_Name[17]<<",";
                }
                else
                {
                    out<<QString(" ")<<",";
                }
            }

            out<<QString("%1").arg(LOG_Rh2[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Vh2[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Ah2[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Acp2[Count_Local3])<<",";

            out<<QString("%1").arg(LOG_Dty2[Count_Local3])<<",";



            if(LOG_Err2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                if(LOG_Err2[Count_Local3] == (0x01))
                {
                    out<<Qstring_error_Log_Name[0]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x02))
                {
                    out<<Qstring_error_Log_Name[1]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x03))
                {
                    out<<Qstring_error_Log_Name[2]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x04))
                {
                    out<<Qstring_error_Log_Name[3]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x05))
                {
                    out<<Qstring_error_Log_Name[4]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x06))
                {
                    out<<Qstring_error_Log_Name[5]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x07))
                {
                    out<<Qstring_error_Log_Name[6]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x08))
                {
                    out<<Qstring_error_Log_Name[7]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x09))
                {
                    out<<Qstring_error_Log_Name[8]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x10))
                {
                    out<<Qstring_error_Log_Name[9]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x11))
                {
                    out<<Qstring_error_Log_Name[10]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x12))
                {
                    out<<Qstring_error_Log_Name[11]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x21))
                {
                    out<<Qstring_error_Log_Name[12]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x22))
                {
                    out<<Qstring_error_Log_Name[13]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x23))
                {
                    out<<Qstring_error_Log_Name[14]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x24))
                {
                    out<<Qstring_error_Log_Name[15]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x25))
                {
                    out<<Qstring_error_Log_Name[16]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x26))
                {
                    out<<Qstring_error_Log_Name[17]<<",";
                }
                else
                {
                    out<<QString(" ")<<",";
                }
            }


                out<<QString("%1").arg(LOG_Accel_x[Count_Local3])<<",";




                out<<QString("%1").arg(LOG_Accel_y[Count_Local3])<<",";




                out<<QString("%1").arg(LOG_Accel_z[Count_Local3])<<",";




                out<<QString("%1").arg(LOG_Gyro_x[Count_Local3])<<",";



                out<<QString("%1").arg(LOG_Gyro_y[Count_Local3])<<",";




                out<<QString("%1").arg(LOG_Gyro_z[Count_Local3])<<",";



             out<<QString("%1").arg(LOG_Temp_c[Count_Local3])<<",";






             out<<QString("%1").arg(LOG_Accel_x_h[Count_Local3])<<",";




             out<<QString("%1").arg(LOG_Accel_y_h[Count_Local3])<<",";




             out<<QString("%1").arg(LOG_Accel_z_h[Count_Local3])<<",";




             out<<QString("%1").arg(LOG_Gyro_x_h[Count_Local3])<<",";



             out<<QString("%1").arg(LOG_Gyro_y_h[Count_Local3])<<",";




             out<<QString("%1").arg(LOG_Gyro_z_h[Count_Local3])<<",";














/*
            if(LOG_Vin[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Vin[Count_Local3])<<",";
            }


            if(LOG_TC1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_TC1[Count_Local3])<<",";
            }

            if(LOG_TC2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_TC2[Count_Local3])<<",";
            }




            if(LOG_Rh1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Rh1[Count_Local3])<<",";
            }

            if(LOG_Vh1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Vh1[Count_Local3])<<",";
            }


            if(LOG_Ah1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Ah1[Count_Local3])<<",";
            }

            if(LOG_Acp1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Acp1[Count_Local3])<<",";
            }




            if(LOG_Dty1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Dty1[Count_Local3])<<",";
            }



            if(LOG_Err1[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                if(LOG_Err1[Count_Local3] == (0x01))
                {
                    out<<Qstring_error_Log_Name[0]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x02))
                {
                    out<<Qstring_error_Log_Name[1]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x03))
                {
                    out<<Qstring_error_Log_Name[2]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x04))
                {
                    out<<Qstring_error_Log_Name[3]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x05))
                {
                    out<<Qstring_error_Log_Name[4]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x06))
                {
                    out<<Qstring_error_Log_Name[5]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x07))
                {
                    out<<Qstring_error_Log_Name[6]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x08))
                {
                    out<<Qstring_error_Log_Name[7]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x09))
                {
                    out<<Qstring_error_Log_Name[8]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x10))
                {
                    out<<Qstring_error_Log_Name[9]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x11))
                {
                    out<<Qstring_error_Log_Name[10]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x12))
                {
                    out<<Qstring_error_Log_Name[11]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x21))
                {
                    out<<Qstring_error_Log_Name[12]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x22))
                {
                    out<<Qstring_error_Log_Name[13]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x23))
                {
                    out<<Qstring_error_Log_Name[14]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x24))
                {
                    out<<Qstring_error_Log_Name[15]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x25))
                {
                    out<<Qstring_error_Log_Name[16]<<",";
                }
                else if(LOG_Err1[Count_Local3] == (0x26))
                {
                    out<<Qstring_error_Log_Name[17]<<",";
                }
                else
                {
                    out<<QString(" ")<<",";
                }
            }



            if(LOG_Rh2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Rh2[Count_Local3])<<",";
            }

            if(LOG_Vh2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Vh2[Count_Local3])<<",";
            }


            if(LOG_Ah2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Ah2[Count_Local3])<<",";
            }

            if(LOG_Acp2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Acp2[Count_Local3])<<",";
            }

            if(LOG_Dty2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Dty2[Count_Local3])<<",";
            }


            if(LOG_Err2[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                if(LOG_Err2[Count_Local3] == (0x01))
                {
                    out<<Qstring_error_Log_Name[0]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x02))
                {
                    out<<Qstring_error_Log_Name[1]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x03))
                {
                    out<<Qstring_error_Log_Name[2]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x04))
                {
                    out<<Qstring_error_Log_Name[3]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x05))
                {
                    out<<Qstring_error_Log_Name[4]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x06))
                {
                    out<<Qstring_error_Log_Name[5]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x07))
                {
                    out<<Qstring_error_Log_Name[6]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x08))
                {
                    out<<Qstring_error_Log_Name[7]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x09))
                {
                    out<<Qstring_error_Log_Name[8]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x10))
                {
                    out<<Qstring_error_Log_Name[9]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x11))
                {
                    out<<Qstring_error_Log_Name[10]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x12))
                {
                    out<<Qstring_error_Log_Name[11]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x21))
                {
                    out<<Qstring_error_Log_Name[12]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x22))
                {
                    out<<Qstring_error_Log_Name[13]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x23))
                {
                    out<<Qstring_error_Log_Name[14]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x24))
                {
                    out<<Qstring_error_Log_Name[15]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x25))
                {
                    out<<Qstring_error_Log_Name[16]<<",";
                }
                else if(LOG_Err2[Count_Local3] == (0x26))
                {
                    out<<Qstring_error_Log_Name[17]<<",";
                }
                else
                {
                    out<<QString(" ")<<",";
                }
            }

            if(LOG_Accel_x[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Accel_x[Count_Local3])<<",";
            }


            if(LOG_Accel_y[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Accel_y[Count_Local3])<<",";
            }


            if(LOG_Accel_z[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Accel_z[Count_Local3])<<",";
            }


            if(LOG_Gyro_x[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Gyro_x[Count_Local3])<<",";
            }


            if(LOG_Gyro_y[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Gyro_y[Count_Local3])<<",";
            }


            if(LOG_Gyro_z[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Gyro_z[Count_Local3])<<",";
            }

            if(LOG_Temp_c[Count_Local3] == 0)
            {
                out<<QString(" ")<<",";
            }
            else
            {
                out<<QString("%1").arg(LOG_Temp_c[Count_Local3])<<",";
            }*/
            out<<"\n";

        }
        file.close();
    }
    else
    {

        QMessageBox::information(
                this,
                "State",
                "File error:\n" + file.errorString() + "\n\nPath:\n" + fileName
            );

    }




}





/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Init_TableWidget(QTableWidget *TableWidget, uint16_t Table_Width,uint16_t Table_Horizontal_Number,uint16_t Table_Count,uint16_t Table_Vertical_Number,QStringList HeadText)
{

    TableWidget->setColumnCount(Table_Horizontal_Number);
    for(uint16_t i = 0;i < Table_Vertical_Number;i++)
    {
        TableWidget->setColumnWidth(i, Table_Width);
    }
    TableWidget->verticalHeader()->setDefaultSectionSize(25);

    TableWidget->setHorizontalHeaderLabels(HeadText);
    TableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    TableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    TableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    TableWidget->setAlternatingRowColors(true);
    TableWidget->verticalHeader()->setVisible(false);
    TableWidget->horizontalHeader()->setStretchLastSection(true);

    this->setStyleSheet(QString(u8"QHeaderView::section{background-color: rgb(14, 29, 63); color: white; border:1px solid rgb(255, 255, 255);}"));

    //     и 
    TableWidget->setRowCount(Table_Vertical_Number);

    for (uint16_t i = 0; i < Table_Vertical_Number; i++)
    {
        Table_Count ++;
        TableWidget->setRowHeight(i, 24);
        QTableWidgetItem *ItemData[Table_Horizontal_Number];

        for (uint16_t j = 0; j < Table_Horizontal_Number; j++)
        {
            if(j == 0)
            {
                ItemData[j] = new QTableWidgetItem(QString::number(Table_Count));
            }
            else
            {
                ItemData[j] = new QTableWidgetItem(QString(""));
            }
            TableWidget->setItem(i, j, ItemData[j]);
        }

    }

    TableWidget->setEditTriggers(QAbstractItemView::CurrentChanged);

    for(int i=0; i<TableWidget->rowCount(); i++)
    {
        QTableWidgetItem* item = TableWidget->item(i,0); //  ȡÿ  ??? еĵ Ԫ  ָ???
        item->setFlags(Qt::ItemIsEnabled);//   ø item     ޸ ???
    }
}





/**************************************************************************
 *         castVariant2ListListVariant
 *           QVariantתΪQList<QList<QVariant> >,   ڿ  ٶ     
 *         ???
 *         ???
 *************************************************************************/
void castVariant2ListListVariant(const QVariant &var, QList<QList<QVariant> > &res)
{
    QVariantList varRows = var.toList();
    if(varRows.isEmpty())
    {
        return;
    }

    const int rowCount = varRows.size();
    QVariantList rowData;

    for(int i=0;i<rowCount;++i)
    {
        rowData = varRows[i].toList();
        res.push_back(rowData);
    }
}

/**************************************************************************
 *         castListListVariant2Variant
 *           QList<QList<QVariant> > תΪQVariant,   ڿ   д    
 *         ???
 *         ???
 *************************************************************************/
void castListListVariant2Variant(const QList<QList<QVariant> > &cells, QVariant &res)
{
    QVariantList vars;
    const int rows = cells.size();
    for(int i=0;i<rows;++i)
    {
        vars.append(QVariant(cells[i]));
    }
    res = QVariant(vars);
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Save_TableWidget(QTableWidget *TableWidget)
{
    QDateTime current_date_time =QDateTime::currentDateTime();
    QString excelName = "data_" + current_date_time.toString("yyyy-MM-dd_hh-mm-ss");
    QString filePath = QFileDialog::getSaveFileName(this, "Save Data", excelName, "Microsoft Excel 2013(*.xlsx)");
    int row = TableWidget->rowCount();
    int col = TableWidget->columnCount();

    QAxObject* excel = new QAxObject(this);
    //excel->setControl("ket.Application");//wps
    excel->setControl("Excel.Application"); //Excel
    excel->dynamicCall("SetVisible(bool Visible)", false);
    excel->setProperty("DisplayAlerts", false);
    QAxObject* workbooks = excel->querySubObject("WorkBooks");
    workbooks->dynamicCall("Add");
    QAxObject* workbook = excel->querySubObject("ActiveWorkBook");
    QAxObject* worksheets = workbook->querySubObject("Sheets");
    QAxObject* worksheet = worksheets->querySubObject("Item(int)", 1);
    QList<QList<QVariant>> excel_list;
    QList<QVariant> wfwfwf1;

    for (int j = 0; j < col; j++)
    {
        wfwfwf1.append(TableWidget->horizontalHeaderItem(j)->text());
    }
    excel_list.append((wfwfwf1));
    for (int i = 0; i < (row); i++)
    {
        QList<QVariant> wfwfwf;

        for (int j = 0; j < col; j++)
        {
            wfwfwf.append(TableWidget->item(i, j)->text());
        }
        excel_list.append((wfwfwf));
    }
    QVariant var;
    castListListVariant2Variant(excel_list,var);

    QAxObject *user_range = worksheet->querySubObject("Range(const QString&)","A" + QString("%1").arg(row+1) + ":" + "E"  +  QString("%1").arg(1));
    user_range->setProperty("Value", var);



    workbook->dynamicCall("SaveAs(const QString &)", QDir::toNativeSeparators(filePath));
    if (excel != NULL) {
        excel->dynamicCall("Quit()");
        delete excel;
        excel = NULL;
    }
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Save_TableWidget_Tcr(void)
{




}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Load_TableWidget(QTableWidget *TableWidget,uint8_t ArrayNumber)
{
    QString strFile = QFileDialog::getOpenFileName(this,QStringLiteral("Option Excel file"),"","Excel files(*.xls *.xlsx)");

    if (strFile.isEmpty())
    {
        return;
    }

    QAxObject excel("Excel.Application");
    excel.setProperty("Visible", false);
    QAxObject *work_books = excel.querySubObject("WorkBooks");
    //    ָ   ļ 
    work_books->dynamicCall("Open (const QString&)", strFile);
    QAxObject *work_book = excel.querySubObject("ActiveWorkBook");
    static int row_count = 0;
    QAxObject *work_sheet = work_book->querySubObject("Sheets(int)", 1);
    QAxObject *used_range = work_sheet->querySubObject("UsedRange");

    QVariant var = used_range->dynamicCall("Value"); //   ȡ          ???
    QList<QList<QVariant>> excel_list;
    QVariantList varRows = var.toList();
    if(varRows.isEmpty())
    {
        return;
    }
    row_count = varRows.size();
    QVariantList rowData;

    for(int i=0;i<row_count;++i)
    {
        rowData = varRows[i].toList();
        excel_list.push_back(rowData);
    }//ת      

    //          QTableWidget        
    TableWidget->setRowCount(0);
    //TableWidget->clear();

    //    ñ               
    int rowCount = excel_list.size();
    if (!rowCount) return; //         Ϊ գ  򲻼 ???

    int columnCount = excel_list.first().size(); //            б       ͬ ĳ ???
    //QStringList headText;
    //headText << "Index" << "Begin(s)" << "End(s)" << "Type" << "Target Value";
    //TableWidget->setHorizontalHeaderLabels(headText);
    if(ArrayNumber == TCR_T)
     {
          if(columnCount != 4)
          {
              uint8_t Table_Count = 0;
              TableWidget->setRowCount(51 - 1);
              TableWidget->setColumnCount(4);
              for (uint16_t i = 0; i < 50; i++)
              {
                  Table_Count ++;
                  TableWidget->setRowHeight(i, 24);
                  QTableWidgetItem *ItemData[50];

                  for (uint16_t j = 0; j < 4; j++)
                  {
                      if(j == 0)
                      {
                          ItemData[j] = new QTableWidgetItem(QString::number(Table_Count));
                      }
                      else
                      {
                          ItemData[j] = new QTableWidgetItem(QString(""));
                      }
                      TableWidget->setItem(i, j, ItemData[j]);
                  }

              }

              TableWidget->setEditTriggers(QAbstractItemView::CurrentChanged);

              for(int i=0; i<TableWidget->rowCount(); i++)
              {
                  QTableWidgetItem* item = TableWidget->item(i,0); //  ȡÿ  ??? еĵ Ԫ  ָ???
                  item->setFlags(Qt::ItemIsEnabled);//   ø item     ޸ ???
              }

              work_book->dynamicCall("Close(Boolean)", false);  // ر  ļ 
              excel.dynamicCall("Quit(void)");  //  ???

              QMessageBox::information(this,"State","Please option the correct tcr_t excel ");
              return;
          }

     }
     else
     {
         if(columnCount != 5)
         {
             TableWidget->setRowCount(21 - 1);
             TableWidget->setColumnCount(5);

             uint8_t Table_Count = 0;

             for (uint16_t i = 0; i < 20; i++)
             {
                 Table_Count ++;
                 TableWidget->setRowHeight(i, 24);
                 QTableWidgetItem *ItemData[20];

                 for (uint16_t j = 0; j < 5; j++)
                 {
                     if(j == 0)
                     {
                         ItemData[j] = new QTableWidgetItem(QString::number(Table_Count));
                     }
                     else
                     {
                         ItemData[j] = new QTableWidgetItem(QString(""));
                     }
                     TableWidget->setItem(i, j, ItemData[j]);
                 }

             }

             TableWidget->setEditTriggers(QAbstractItemView::CurrentChanged);

             for(int i=0; i<TableWidget->rowCount(); i++)
             {
                 QTableWidgetItem* item = TableWidget->item(i,0); //  ȡÿ  ??? еĵ Ԫ  ָ???
                 item->setFlags(Qt::ItemIsEnabled);//   ø item     ޸ ???
             }

             work_book->dynamicCall("Close(Boolean)", false);  // ر  ļ 
             excel.dynamicCall("Quit(void)");  //  ???

             QMessageBox::information(this,"State","Please option the correct Profile excel ");

             return;
         }
     }
    TableWidget->setRowCount(rowCount - 1);
    TableWidget->setColumnCount(columnCount);


    //     ÿһ???
    for (int row = 1; row < rowCount; ++row)
    {

        //     ÿһ???
        for (int col = 0; col < columnCount; ++col)
        {
            //        б  л ȡQVariant???
            QVariant value = excel_list.at(row).at(col);

            //     һ  QTableWidgetItem        ֵ    ΪQVariant  toString()    
            // ע ⣺    QVariant       ַ      ͣ toString()   ܲ       ת    ???
            //        ֻ    ڣ         Ҫʹ  toInt()  toDouble()  toDate().toString()???
            QTableWidgetItem *item = new QTableWidgetItem(value.toString());

            //     Ŀ   õ         Ӧ  Ԫ  ???
            TableWidget->setItem((row -1 ), col, item);
        }
    }


    TableWidget->setEditTriggers(QAbstractItemView::CurrentChanged);

    for(int i=0; i<TableWidget->rowCount(); i++)
    {
        QTableWidgetItem* item = TableWidget->item(i,0); //  ȡÿ  ??? еĵ Ԫ  ָ???
        item->setFlags(Qt::ItemIsEnabled);//   ø item     ޸ ???
    }

    work_book->dynamicCall("Close(Boolean)", false);  // ر  ļ 
    excel.dynamicCall("Quit(void)");  //  ???
}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Clean_Table_Init_Tcr_clicked()
{

}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Save_Table_Init_Tcr_clicked()
{
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Load_Table_Init_Tcr_clicked()
{

}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Save_Table_Set_Profile1_clicked()
{
    a_Save_TableWidget(ui->BAT_Table_Set_Profile1);
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Load_Table_Set_Profile1_clicked()
{
    a_Load_TableWidget(ui->BAT_Table_Set_Profile1,PROFILE);
    QCustomplot_Display_Data_Proc(ui->BAT_Table_Set_Profile1,ui->BAT_Customplot_Set_Profile1,PROFILE);

}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Save_Table_Set_Profile2_clicked()
{
    a_Save_TableWidget(ui->BAT_Table_Set_Profile2);
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Load_Table_Set_Profile2_clicked()
{
    a_Load_TableWidget(ui->BAT_Table_Set_Profile2,PROFILE);
    QCustomplot_Display_Data_Proc(ui->BAT_Table_Set_Profile2,ui->BAT_Customplot_Set_Profile2,PROFILE);
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Save_Table_Set_Profile3_clicked()
{
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Load_Table_Set_Profile3_clicked()
{
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Display_Table_Set_Profile1_clicked()
{
    QCustomplot_Display_Data_Proc(ui->BAT_Table_Set_Profile1,ui->BAT_Customplot_Set_Profile1,PROFILE);
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Display_Table_Set_Profile2_clicked()
{
    QCustomplot_Display_Data_Proc(ui->BAT_Table_Set_Profile2,ui->BAT_Customplot_Set_Profile2,PROFILE);

}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Display_Table_Set_Profile3_clicked()
{
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Display_Table_Init_Tcr_clicked()
{

}








/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Sync_time_to_device(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[12] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x07;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x4c;

    QTime current_time = QTime::currentTime();
    QString ct = current_time.toString();
    QDate current_date = QDate::currentDate();
    QString cd = current_date.toString();

    int currentyear = current_date.year();
    int currentmonth = current_date.month();
    int currentday = current_date.day();

    int currenthour = current_time.hour();
    int currentminute = current_time.minute();
    int currentSecond = current_time.second();
    int currentmsec = current_time.msec();
    int write_currentmsec = 0;

    if((currentmsec + 50)>= 1000)
    {
        currentmsec = ((currentmsec + 50) - 1000);
        currentSecond = currentSecond + 1;
        write_currentmsec = (255 -  (currentmsec/40));
    }
    else
    {
        currentmsec = ((currentmsec + 50));
        write_currentmsec = (255 -  (currentmsec/40));

    }


    SendBuffer[i++] = currentyear%100;
    qDebug()<<currentyear%100;

    SendBuffer[i++] = currentmonth;
    qDebug()<<currentmonth;

    SendBuffer[i++] = currentday;
    qDebug()<<currentday;

    SendBuffer[i++] = currenthour;
    qDebug()<<currenthour;

    SendBuffer[i++] = currentminute;
    qDebug()<<currentminute;

    SendBuffer[i++] = currentSecond;
    qDebug()<<currentSecond;

    SendBuffer[i++] = write_currentmsec;
    qDebug()<<write_currentmsec;




    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}








Widget::~Widget()
{
    delete ui;
}
/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Key_Serial_Control_clicked()
{
    QString NowPushButton_text = ui->BAT_Key_Serial_Control->text();

    if(NowPushButton_text == "Open")
    {
        C_Serialport_Bat->setPortName(ui->BAT_SerialPortSelect->currentText());

        if(ui->BAT_SerialPortBaud->currentText() == "9600")
        {
            C_Serialport_Bat->setBaudRate(QSerialPort::Baud9600,QSerialPort::AllDirections);
        }
        else if(ui->BAT_SerialPortBaud->currentText() == "19200")
        {
            C_Serialport_Bat->setBaudRate(QSerialPort::Baud19200,QSerialPort::AllDirections);

        }
        else if(ui->BAT_SerialPortBaud->currentText() == "57600")
        {
            C_Serialport_Bat->setBaudRate(QSerialPort::Baud57600,QSerialPort::AllDirections);
        }
        else if(ui->BAT_SerialPortBaud->currentText() == "115200")
        {
            C_Serialport_Bat->setBaudRate(QSerialPort::Baud115200,QSerialPort::AllDirections);
        }
        C_Serialport_Bat->setDataBits(QSerialPort::Data8);
        C_Serialport_Bat->setStopBits(QSerialPort::OneStop);
        C_Serialport_Bat->setParity(QSerialPort::NoParity);
        if(C_Serialport_Bat->open(QIODevice::ReadWrite) == true)
        {
            //     readBufferSize???56 ֽ 
            C_Serialport_Bat->setReadBufferSize(1000000);
            serialRealStatus = 1;
            ui->BAT_Key_Serial_Control->setText("Close");
            QMessageBox::information(this,"State","Port connected succeed");

            //     QLabel  ʾͼƬ
            QPixmap pixmap2(":/Green.png"); //  滻Ϊ    ͼƬ·???
            ui->BAT_Device_Connect_light->setPixmap(pixmap2);
            // ȷ  ͼƬ         ŵ QLabel  С
            ui->BAT_Device_Connect_light->setScaledContents(true);

            //a_Uart_Sync_time_to_device();
        }
        else
        {
            serialRealStatus = 0;
            QMessageBox::warning(this, "State", "Port connected fail!");

            //     QLabel  ʾͼƬ
            QPixmap pixmap2(":/Grey.png"); //  滻Ϊ    ͼƬ·???
            ui->BAT_Device_Connect_light->setPixmap(pixmap2);
            // ȷ  ͼƬ         ŵ QLabel  С
            ui->BAT_Device_Connect_light->setScaledContents(true);
        }


    }
    else
    {
        C_Serialport_Bat->close();
        serialRealStatus = 0;
        ui->BAT_Key_Serial_Control->setText("Open");
        QMessageBox::information(this,"State","Closed");

        //     QLabel  ʾͼƬ
        QPixmap pixmap2(":/Grey.png"); //  滻Ϊ    ͼƬ·???
        ui->BAT_Device_Connect_light->setPixmap(pixmap2);
        // ȷ  ͼƬ         ŵ QLabel  С
        ui->BAT_Device_Connect_light->setScaledContents(true);
    }
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::Uart_Send_CMD(unsigned char *addr, int num)
{
    if (addr == nullptr || num <= 0)
    {
        qDebug() << "Uart_Send_CMD error: invalid buffer or length";
        return;
    }

    if (C_Serialport_Bat == nullptr)
    {
        qDebug() << "Uart_Send_CMD error: serial port pointer is null";
        return;
    }

    if (!C_Serialport_Bat->isOpen())
    {
        qDebug() << "Uart_Send_CMD error: serial port is not open";
        QMessageBox::warning(this, "Serial Port", "Please open serial port first.");
        return;
    }

    QByteArray C_SendArray1;

    for (int i = 0; i < num; i++)
    {
        C_SendArray1.append(static_cast<char>(addr[i]));
    }

    uint16_t Crc12 = crc16(addr, num, 0xffff);

    C_SendArray1.append(static_cast<char>((Crc12 >> 8) & 0xff));
    C_SendArray1.append(static_cast<char>(Crc12 & 0xff));

    qDebug() << "Send CMD:" << C_SendArray1.toHex(' ').toUpper();

    C_Serialport_Bat->write(C_SendArray1);
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Read_board_basic_info_Get(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[5] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x44;

    Uart_Send_CMD(SendBuffer,i);
}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_clean_Flash(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[5] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x5D;

    Uart_Send_CMD(SendBuffer,i);
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Write_Real_Time_Log_Set(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[43] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x24;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x21;

    SendBuffer[i++] = 0x01;


    SendBuffer[i++] = Board_data.Vin;
    SendBuffer[i++] = Board_data.Ain;

    SendBuffer[i++] = 0x01;

    SendBuffer[i++] = Board_data.Vbd;
    SendBuffer[i++] = Board_data.Abd;



    SendBuffer[i++] = Board_data.TC1;
    SendBuffer[i++] = Board_data.TC2;
    SendBuffer[i++] = Board_data.TC3;
    SendBuffer[i++] = Board_data.TC4;
    SendBuffer[i++] = Board_data.TC5;
    SendBuffer[i++] = Board_data.TC6;

    SendBuffer[i++] = Board_data.Rh1;
    SendBuffer[i++] = Board_data.Vh1;

    SendBuffer[i++] = Board_data.Ah1;
    SendBuffer[i++] = Board_data.Acp1;
    SendBuffer[i++] = Board_data.Th1;
    SendBuffer[i++] = Board_data.Dty1;
    SendBuffer[i++] = Board_data.Pf1;
    SendBuffer[i++] = 0x01;


    SendBuffer[i++] = Board_data.Rh2;
    SendBuffer[i++] = Board_data.Vh2;
    SendBuffer[i++] = Board_data.Ah2;
    SendBuffer[i++] = Board_data.Acp2;
    SendBuffer[i++] = Board_data.Th2;
    SendBuffer[i++] = Board_data.Dty2;
    SendBuffer[i++] = Board_data.Pf2;
    SendBuffer[i++] = 0x01;


    SendBuffer[i++] = Board_data.Rh3;
    SendBuffer[i++] = Board_data.Vh3;
    SendBuffer[i++] = Board_data.Ah3;
    SendBuffer[i++] = Board_data.Acp3;
    SendBuffer[i++] = Board_data.Th3;
    SendBuffer[i++] = Board_data.Dty3;
    SendBuffer[i++] = Board_data.Pf3;
    SendBuffer[i++] = 0x01;



    qDebug() << i;
    Uart_Send_CMD(SendBuffer,i);
}
/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Export_Flash_Data_clicked()
{
    /*uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x01;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x5C;
    SendBuffer[i++] = 0x01;


    Uart_Send_CMD(SendBuffer,i);*/
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Clean_Flash_Data_clicked()
{
    //a_clean_Flash();
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Clear_Screen_clicked()
{
    ui->BAT_SerialData_display->clear();
    s_heatGraphVisibilityApplied = false;
    for (int g = 0; g < ui->BAT_customplot_Heat->graphCount(); ++g)
        ui->BAT_customplot_Heat->graph(g)->data()->clear();
    for (int g = 0; g < ui->BAT_Customplot_Set_Hannwindow->graphCount(); ++g)
        ui->BAT_Customplot_Set_Hannwindow->graph(g)->data()->clear();
    for (int g = 0; g < ui->BAT_Customplot_Set_Hannwindow_Gyro->graphCount(); ++g)
        ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(g)->data()->clear();
    display_paintEvent_count = 0;

    std::fill(LOG_Time.begin(), LOG_Time.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    /*std::fill(LOG_Vin.begin(), LOG_Vin.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Ain.begin(), LOG_Ain.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Batt_Data.begin(), LOG_Batt_Data.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Vbd.begin(), LOG_Vbd.end(),  0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Abd.begin(), LOG_Abd.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_TC1.begin(), LOG_TC1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_TC2.begin(), LOG_TC2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_TC3.begin(), LOG_TC3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_TC4.begin(), LOG_TC4.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_TC5.begin(), LOG_TC5.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_TC6.begin(), LOG_TC6.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0

    std::fill(LOG_Rh1.begin(), LOG_Rh1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Vh1.begin(), LOG_Vh1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Ah1.begin(), LOG_Ah1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Acp1.begin(), LOG_Acp1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Th1.begin(), LOG_Th1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Dty1.begin(), LOG_Dty1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Pf1.begin(), LOG_Pf1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Err1.begin(), LOG_Err1.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0

    std::fill(LOG_Rh2.begin(), LOG_Rh2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Vh2.begin(), LOG_Vh2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Ah2.begin(), LOG_Ah2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Acp2.begin(), LOG_Acp2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Th2.begin(), LOG_Th2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Dty2.begin(), LOG_Dty2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Pf2.begin(), LOG_Pf2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Err2.begin(), LOG_Err2.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0


    std::fill(LOG_Rh3.begin(), LOG_Rh3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Vh3.begin(), LOG_Vh3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Ah3.begin(), LOG_Ah3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Acp3.begin(), LOG_Acp3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Th3.begin(), LOG_Th3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Dty3.begin(), LOG_Dty3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Pf3.begin(), LOG_Pf3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
    std::fill(LOG_Err3.begin(), LOG_Err3.end(), 0); // ʹ   std::fill       Ԫ      Ϊ 0.0
*/

    //for(uint8_t i = 0;i < 10;i++)
    //{
      //  ui->BAT_customplot_Heat->graph(i)->data()->clear();
    //}

    ui->BAT_customplot_Heat->replot();
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Pause_clicked()
{
    QString NowPushButton_text = ui->BAT_Pause->text();

    if(NowPushButton_text == "Pause")
    {
        ui->BAT_Pause->setText("Resume");
    }
    else
    {
        ui->BAT_Pause->setText("Pause");
    }

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_BAT_SerialData_display_Set(QString Display_Data,uint8_t Refresh_State)
{

    QString NowPushButton_text = ui->BAT_Pause->text();
    if(NowPushButton_text == "Pause")
    {
        if(QPLAINTEXTEDIT_SET == Refresh_State)
        {
            ui->BAT_SerialData_display->setPlainText(Display_Data);
        }
        else
        {
            ui->BAT_SerialData_display->appendPlainText(Display_Data);
        }
    }
}
/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Save_TCR_List_Flash_clicked()
{
    //QCustomplot_Display_Data_Proc(ui->BAT_customplot_Init_Tcr,TCR_T);
}








/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/

void Widget::on_BAT_Read_Configration_clicked()
{

    ui->BAT_Table_Set_Profile1->clear();
    ui->BAT_Table_Set_Profile2->clear();
    QStringList HeadText2;

    HeadText2 << "Index" << "Begin(0.1s)" << "End(0.1s)" << "Type" << "Target Value";

    a_Init_TableWidget(ui->BAT_Table_Set_Profile1,TABLE_WIDTH_TCR,TABLE_HEATING_HORIZONTAL_NUMBER,Table_Heating_profiles1_count,TABLE_HEATING_VERTICAL_NUMBER,HeadText2);

    a_Init_TableWidget(ui->BAT_Table_Set_Profile2,TABLE_WIDTH_TCR,TABLE_HEATING_HORIZONTAL_NUMBER,Table_Heating_profiles2_count,TABLE_HEATING_VERTICAL_NUMBER,HeadText2);

    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x20;

    Uart_Send_CMD(SendBuffer,i);
}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Push_Configration_clicked()
{
    bool check_data1 = false,check_data2 = false,check_data3 = false,check_data4 = false,check_data5 = false,check_data6 = false,check_data7 = false;
    check_data1 = a_H1_Data_Check_And_Set();
    check_data2 = a_H2_Data_Check_And_Set();
    check_data3 = true;

    check_data4 = a_TableWidget_Data_Tran_Array(ui->BAT_Table_Set_Profile1,&H1_data,PROFILE);
    check_data5 = a_TableWidget_Data_Tran_Array(ui->BAT_Table_Set_Profile2,&H2_data,PROFILE);
    check_data6 = true;
    check_data7 = a_Board_Base_Check_And_Set();

    if((check_data1 == true)&&
       (check_data2 == true)&&
       (check_data3 == true)&&
       (check_data4 == true)&&
       (check_data5 == true)&&
       (check_data6 == true)&&
       (check_data7 == true))
    {
        a_Write_Real_Time_Log_Set();
    }


}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_H1_Setting(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[30] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x14;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x50;

    SendBuffer[i++] = 0x61;

    SendBuffer[i++] = H1_data.Heater_enabled_state/256;
    SendBuffer[i++] = H1_data.Heater_enabled_state%256;
    SendBuffer[i++] = H1_data.Max_R/256;
    SendBuffer[i++] = H1_data.Max_R%256;
    SendBuffer[i++] = H1_data.Min_R/256;
    SendBuffer[i++] = H1_data.Min_R%256;


    SendBuffer[i++] = H1_data.Temp_Detection;
    SendBuffer[i++] = H1_data.Temp_limit/256;
    SendBuffer[i++] = H1_data.Temp_limit%256;


    SendBuffer[i++] = H1_data.High_TC_Set;
    SendBuffer[i++] = H1_data.Low_TC_Set;

    SendBuffer[i++] = H1_data.Kp/256;
    SendBuffer[i++] = H1_data.Kp%256;
    SendBuffer[i++] = H1_data.Ki/256;
    SendBuffer[i++] = H1_data.Ki%256;
    SendBuffer[i++] = H1_data.Kd/256;
    SendBuffer[i++] = H1_data.Kd%256;

    SendBuffer[i++] = H1_data.Resi_Protection;
    SendBuffer[i++] = H1_data.Temp_Protection;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_H2_Setting(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[30] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x14;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x50;

    SendBuffer[i++] = 0x62;

    SendBuffer[i++] = H2_data.Heater_enabled_state/256;
    SendBuffer[i++] = H2_data.Heater_enabled_state%256;
    SendBuffer[i++] = H2_data.Max_R/256;
    SendBuffer[i++] = H2_data.Max_R%256;
    SendBuffer[i++] = H2_data.Min_R/256;
    SendBuffer[i++] = H2_data.Min_R%256;


    SendBuffer[i++] = H2_data.Temp_Detection;
    SendBuffer[i++] = H2_data.Temp_limit/256;
    SendBuffer[i++] = H2_data.Temp_limit%256;
    SendBuffer[i++] = H2_data.High_TC_Set;
    SendBuffer[i++] = H2_data.Low_TC_Set;


    SendBuffer[i++] = H2_data.Kp/256;
    SendBuffer[i++] = H2_data.Kp%256;
    SendBuffer[i++] = H2_data.Ki/256;
    SendBuffer[i++] = H2_data.Ki%256;
    SendBuffer[i++] = H2_data.Kd/256;
    SendBuffer[i++] = H2_data.Kd%256;

    SendBuffer[i++] = H2_data.Resi_Protection;
    SendBuffer[i++] = H2_data.Temp_Protection;



    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_H3_Setting(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[30] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x14;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x50;

    SendBuffer[i++] = 0x63;

    SendBuffer[i++] = H3_data.Heater_enabled_state/256;
    SendBuffer[i++] = H3_data.Heater_enabled_state%256;
    SendBuffer[i++] = H3_data.Max_R/256;
    SendBuffer[i++] = H3_data.Max_R%256;
    SendBuffer[i++] = H3_data.Min_R/256;
    SendBuffer[i++] = H3_data.Min_R%256;


    SendBuffer[i++] = H3_data.Temp_Detection;
    SendBuffer[i++] = H3_data.Temp_limit/256;
    SendBuffer[i++] = H3_data.Temp_limit%256;

    SendBuffer[i++] = H3_data.High_TC_Set;
    SendBuffer[i++] = H3_data.Low_TC_Set;


    SendBuffer[i++] = H3_data.Kp/256;
    SendBuffer[i++] = H3_data.Kp%256;
    SendBuffer[i++] = H3_data.Ki/256;
    SendBuffer[i++] = H3_data.Ki%256;
    SendBuffer[i++] = H3_data.Kd/256;
    SendBuffer[i++] = H3_data.Kd%256;


    SendBuffer[i++] = H3_data.Resi_Protection;
    SendBuffer[i++] = H3_data.Temp_Protection;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_H1_Setting(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x01;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x4D;

    SendBuffer[i++] = 0x61;


    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_H2_Setting(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x01;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x4D;

    SendBuffer[i++] = 0x62;


    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_H3_Setting(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x01;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x4D;

    SendBuffer[i++] = 0x63;


    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_Heat_Tcr_table(void)
{

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_Heat_Tcr_table(void)
{


}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_H1_Profile(void)
{
    uint8_t i = 0,Send_count = 0;
    uint8_t SendBuffer[256] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0xA1;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x51;
    SendBuffer[i++] = 0x61;

    a_TableWidget_Data_Tran_Array(ui->BAT_Table_Set_Profile1,&H1_data,PROFILE);

    SendBuffer[i++] = H1_data.Profile_count;
    qDebug() << "H1_data.Profile_count" << H1_data.Profile_count;

    for(uint8_t Array_Count = 0; Array_Count < H1_data.Profile_count; Array_Count++)
    {
        SendBuffer[i++] = H1_data.Profile_Heat_Start_Time[Array_Count]/256;
        SendBuffer[i++] = H1_data.Profile_Heat_Start_Time[Array_Count]%256;

        SendBuffer[i++] = H1_data.Profile_Heat_Stop_Time[Array_Count]/256;
        SendBuffer[i++] = H1_data.Profile_Heat_Stop_Time[Array_Count]%256;


        SendBuffer[i++] = 0;

        if(H1_data.Profile_Heat_Type[Array_Count] == "P")
        {
            SendBuffer[i++] = 1;
        }
        else if(H1_data.Profile_Heat_Type[Array_Count] == "T")
        {
            SendBuffer[i++] = 2;
        }
        else if(H1_data.Profile_Heat_Type[Array_Count] == "V")
        {
            SendBuffer[i++] = 3;
        }
        else
        {
            SendBuffer[i++] = 0;
        }

        SendBuffer[i++] = ((uint32_t)(H1_data.Profile_Heat_Number[Array_Count]*1000))/10/256;
        SendBuffer[i++] = (static_cast<int>(std::round(H1_data.Profile_Heat_Number[Array_Count]*1000)))/10%256;

        Send_count++;

    }

    SendBuffer[2] = Send_count*8 + 1 + 1;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

    H1_data.Profile_count = 0;
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_H2_Profile(void)
{
    uint8_t i = 0,Send_count = 0;
    uint8_t SendBuffer[256] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0xA1;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x51;

    SendBuffer[i++] = 0x62;

    a_TableWidget_Data_Tran_Array(ui->BAT_Table_Set_Profile2,&H2_data,PROFILE);

    SendBuffer[i++] = H2_data.Profile_count;

    for(uint8_t Array_Count = 0; Array_Count < H2_data.Profile_count; Array_Count++)
    {
        SendBuffer[i++] = H2_data.Profile_Heat_Start_Time[Array_Count]/256;
        SendBuffer[i++] = H2_data.Profile_Heat_Start_Time[Array_Count]%256;

        SendBuffer[i++] = H2_data.Profile_Heat_Stop_Time[Array_Count]/256;
        SendBuffer[i++] = H2_data.Profile_Heat_Stop_Time[Array_Count]%256;

        SendBuffer[i++] = 0;

        if(H2_data.Profile_Heat_Type[Array_Count] == "P")
        {
            SendBuffer[i++] = 1;
        }
        else if(H2_data.Profile_Heat_Type[Array_Count] == "T")
        {
            SendBuffer[i++] = 2;
        }
        else if(H2_data.Profile_Heat_Type[Array_Count] == "V")
        {
            SendBuffer[i++] = 3;
        }
        else
        {
            SendBuffer[i++] = 0;
        }

        SendBuffer[i++] = ((uint32_t)(H2_data.Profile_Heat_Number[Array_Count]*1000))/10/256;
        SendBuffer[i++] = (static_cast<int>(std::round(H2_data.Profile_Heat_Number[Array_Count]*1000)))/10%256;
        Send_count++;
    }

    SendBuffer[2] = Send_count*8 + 1 + 1;
    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

    H2_data.Profile_count = 0;

}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_H3_Profile(void)
{
    uint8_t i = 0,Send_count = 0;
    uint8_t SendBuffer[256] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0xA1;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x51;
    SendBuffer[i++] = 0x63;


    SendBuffer[i++] = H3_data.Profile_count;

    for(uint8_t Array_Count = 0; Array_Count < H3_data.Profile_count; Array_Count++)
    {
        SendBuffer[i++] = H3_data.Profile_Heat_Start_Time[Array_Count]/256;
        SendBuffer[i++] = H3_data.Profile_Heat_Start_Time[Array_Count]%256;

        SendBuffer[i++] = H3_data.Profile_Heat_Stop_Time[Array_Count]/256;
        SendBuffer[i++] = H3_data.Profile_Heat_Stop_Time[Array_Count]%256;


        SendBuffer[i++] = 0;

        if(H3_data.Profile_Heat_Type[Array_Count] == "P")
        {
            SendBuffer[i++] = 1;
        }
        else if(H3_data.Profile_Heat_Type[Array_Count] == "T")
        {
            SendBuffer[i++] = 2;
        }
        else if(H3_data.Profile_Heat_Type[Array_Count] == "V")
        {
            SendBuffer[i++] = 3;
        }
        else
        {
            SendBuffer[i++] = 0;
        }
        SendBuffer[i++] = ((uint32_t)(H3_data.Profile_Heat_Number[Array_Count]*1000))/10/256;
        SendBuffer[i++] = (static_cast<int>(std::round(H3_data.Profile_Heat_Number[Array_Count]*1000)))/10%256;
        Send_count++;
    }

    SendBuffer[2] = Send_count*8 + 1 + 1;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

    H3_data.Profile_count = 0;

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_H1_Profile(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x01;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x4E;
    SendBuffer[i++] = 0x61;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_H2_Profile(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x01;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x4E;
    SendBuffer[i++] = 0x62;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}

/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_H3_Profile(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x01;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x4E;
    SendBuffer[i++] = 0x63;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
uint8_t Widget::a_H1_Data_Check_And_Set(void)
{
    if(true == ui->BAT_H1_enabled_state->isChecked())
    {
        H1_data.Heater_enabled_state = 1;
    }
    else
    {
        H1_data.Heater_enabled_state = 0;
    }
    qDebug() <<" ui->BAT_H1_enabled_state->isChecked()" << ui->BAT_H1_enabled_state->isChecked() << "H1_data.Heater_enabled_state" << H1_data.Heater_enabled_state;


    H1_data.Max_R = static_cast<int>(std::round(ui->BAT_H1_Max_R->text().toDouble()*10000))/10; // ???std::round????????????
    if((H1_data.Max_R < 400)||(H1_data.Max_R > 2500))
    {
        QMessageBox::information(this,"State","Please enter the correct range  h1_max_r(0.4---2.5) ");
        return false;
    }
    qDebug() <<" H1_data.Max_R"  << H1_data.Max_R;

    H1_data.Min_R = static_cast<int>(std::round(ui->BAT_H1_Min_R->text().toDouble()*10000))/10; // ???std::round????????????
    if((H1_data.Min_R < 400)||(H1_data.Min_R > 2500))
    {
        QMessageBox::information(this,"State","Please enter the correct range h1_min_r(0.4---2.5) ");
        return false;
    }
    qDebug() <<" H1_data.Min_R"  << H1_data.Min_R;





    H1_data.Temp_Detection = 0;




    H1_data.Temp_limit = (uint16_t)ui->BAT_H1_Temp_limit->text().toUInt();
    if((H1_data.Temp_limit<300)||(H1_data.Temp_limit>450))
    {
        QMessageBox::information(this,"State","Please enter the correct range  h1_temp_limit(300---450) ");
        return false;
    }
    qDebug() <<" H1_data.Temp_limit"  << H1_data.Temp_limit;


    H1_data.Kp = (uint16_t)ui->BAT_H1_Kp->text().toUInt();
    if((H1_data.Kp<0)||(H1_data.Kp>999))
    {
        QMessageBox::information(this,"State","Please enter the correct range h1_kp(0---999) ");
        return false;
    }
    qDebug() <<" H1_data.Kp"  << H1_data.Kp;

    H1_data.Ki = (uint16_t)ui->BAT_H1_Ki->text().toUInt();
    if((H1_data.Ki<0)||(H1_data.Ki>999))
    {
        QMessageBox::information(this,"State","Please enter the correct range  h1_ki(0---999) ");
        return false;
    }
    qDebug() <<" H1_data.Ki"  << H1_data.Ki;

    H1_data.Kd = (uint16_t)ui->BAT_H1_Kd->text().toUInt();
    if((H1_data.Kd<0)||(H1_data.Kd>999))
    {
        QMessageBox::information(this,"State","Please enter the correct range  h1_kd(0---999) ");
        return false;
    }
    qDebug() <<" H1_data.Kd"  << H1_data.Kd;




    if(true == ui->BAT_H1_Resi_Protection->isChecked())
    {
        H1_data.Resi_Protection = 1;
    }
    else
    {
        H1_data.Resi_Protection = 0;
    }
    qDebug() <<" ui->BAT_H1_Resi_Protection->isChecked()" << ui->BAT_H1_Resi_Protection->isChecked() << "H1_data.Resi_Protection" << H1_data.Resi_Protection;


    if(true == ui->BAT_H1_Temp_Protection->isChecked())
    {
        H1_data.Temp_Protection = 1;
    }
    else
    {
        H1_data.Temp_Protection = 0;
    }
    qDebug() <<" ui->BAT_H1_Temp_Protection->isChecked()" << ui->BAT_H1_Temp_Protection->isChecked() << "H1_data.Temp_Protection" << H1_data.Temp_Protection;





     H1_data.Low_TC_Set = 1;


    if(true == ui->BAT_H1_TC_Channel1_Check->isChecked())
    {
        if(ui->BAT_H1_TC_Seclect_Channel1->currentText() == "TC1")
        {
            H1_data.High_TC_Set = 1;
        }
        else if(ui->BAT_H1_TC_Seclect_Channel1->currentText() == "TC2")
        {
            H1_data.High_TC_Set = 2;
        }
        else if(ui->BAT_H1_TC_Seclect_Channel1->currentText() == "TC3")
        {
            H1_data.High_TC_Set = 3;
        }
        else if(ui->BAT_H1_TC_Seclect_Channel1->currentText() == "TC4")
        {
            H1_data.High_TC_Set = 4;
        }
        else if(ui->BAT_H1_TC_Seclect_Channel1->currentText() == "TC5")
        {
            H1_data.High_TC_Set = 5;
        }
        else if(ui->BAT_H1_TC_Seclect_Channel1->currentText() == "TC6")
        {
            H1_data.High_TC_Set = 6;
        }

    }
    else
    {
        H1_data.High_TC_Set = 1;
    }
    qDebug() <<" ui->BAT_H1_TC_Seclect_Channel1->currentText()" << ui->BAT_H1_TC_Seclect_Channel1->currentText() << "H1_data.High_TC_Set" << H1_data.High_TC_Set;


    return true;

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
uint8_t Widget::a_H2_Data_Check_And_Set(void)
{

    if(true == ui->BAT_H2_enabled_state->isChecked())
    {
        H2_data.Heater_enabled_state = 1;
    }
    else
    {
        H2_data.Heater_enabled_state = 0;
    }
    qDebug() <<" ui->BAT_H2_enabled_state->isChecked()" << ui->BAT_H2_enabled_state->isChecked() << "H2_data.Heater_enabled_state" << H2_data.Heater_enabled_state;


    H2_data.Max_R = static_cast<int>(std::round(ui->BAT_H2_Max_R->text().toDouble()*10000))/10; // ???std::round????????????
    if((H2_data.Max_R < 400)||(H2_data.Max_R > 2500))
    {
        QMessageBox::information(this,"State","Please enter the correct range h2_max_r(0.4---2.5) ");
        return false;
    }
    qDebug() <<" H2_data.Max_R"  << H2_data.Max_R;

    H2_data.Min_R = static_cast<int>(std::round(ui->BAT_H2_Min_R->text().toDouble()*10000))/10; // ???std::round????????????
    if((H2_data.Min_R < 400)||(H2_data.Min_R > 2500))
    {
        QMessageBox::information(this,"State","Please enter the correct range h2_min_r(0.4---2.5) ");
        return false;
    }
    qDebug() <<" H2_data.Min_R"  << H2_data.Min_R;





    H2_data.Temp_Detection = 0;



    H2_data.Temp_limit = (uint16_t)ui->BAT_H2_Temp_limit->text().toUInt();
    if((H2_data.Temp_limit<300)||(H2_data.Temp_limit>450))
    {
        QMessageBox::information(this,"State","Please enter the correct range h2_temp_limit(300---450) ");
        return false;
    }
    qDebug() <<" H2_data.Temp_limit"  << H2_data.Temp_limit;


    H2_data.Kp = (uint16_t)ui->BAT_H2_Kp->text().toUInt();
    if((H2_data.Kp<0)||(H2_data.Kp>999))
    {
        QMessageBox::information(this,"State","Please enter the correct range h2_kp(0---999) ");
        return false;
    }
    qDebug() <<" H2_data.Kp"  << H2_data.Kp;

    H2_data.Ki = (uint16_t)ui->BAT_H2_Ki->text().toUInt();
    if((H2_data.Ki<0)||(H2_data.Ki>999))
    {
        QMessageBox::information(this,"State","Please enter the correct range h2_ki(0---999) ");
        return false;
    }
    qDebug() <<" H2_data.Ki"  << H2_data.Ki;

    H2_data.Kd = (uint16_t)ui->BAT_H2_Kd->text().toUInt();
    if((H2_data.Kd<0)||(H2_data.Kd>999))
    {
        QMessageBox::information(this,"State","Please enter the correct range h2_kd(0---999) ");
        return false;
    }
    qDebug() <<" H2_data.Kd"  << H2_data.Kd;




    if(true == ui->BAT_H2_Resi_Protection->isChecked())
    {
        H2_data.Resi_Protection = 1;
    }
    else
    {
        H2_data.Resi_Protection = 0;
    }
    qDebug() <<" ui->BAT_H2_Resi_Protection->isChecked()" << ui->BAT_H2_Resi_Protection->isChecked() << "H2_data.Resi_Protection" << H2_data.Resi_Protection;


    if(true == ui->BAT_H2_Temp_Protection->isChecked())
    {
        H2_data.Temp_Protection = 1;
    }
    else
    {
        H2_data.Temp_Protection = 0;
    }
    qDebug() <<" ui->BAT_H2_Temp_Protection->isChecked()" << ui->BAT_H2_Temp_Protection->isChecked() << "H2_data.Temp_Protection" << H2_data.Temp_Protection;




     H2_data.Low_TC_Set = 1;


    if(true == ui->BAT_H2_TC_Channel1_Check->isChecked())
    {
        if(ui->BAT_H2_TC_Seclect_Channel1->currentText() == "TC1")
        {
            H2_data.High_TC_Set = 1;
        }
        else if(ui->BAT_H2_TC_Seclect_Channel1->currentText() == "TC2")
        {
            H2_data.High_TC_Set = 2;
        }
        else if(ui->BAT_H2_TC_Seclect_Channel1->currentText() == "TC3")
        {
            H2_data.High_TC_Set = 3;
        }
        else if(ui->BAT_H2_TC_Seclect_Channel1->currentText() == "TC4")
        {
            H2_data.High_TC_Set = 4;
        }
        else if(ui->BAT_H2_TC_Seclect_Channel1->currentText() == "TC5")
        {
            H2_data.High_TC_Set = 5;
        }
        else if(ui->BAT_H2_TC_Seclect_Channel1->currentText() == "TC6")
        {
            H2_data.High_TC_Set = 6;
        }

    }
    else
    {
        H2_data.High_TC_Set = 1;
    }
    qDebug() <<" ui->BAT_H2_TC_Seclect_Channel1->currentText()" << ui->BAT_H2_TC_Seclect_Channel1->currentText() << "H2_data.High_TC_Set" << H2_data.High_TC_Set;



    return true;

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
uint8_t Widget::a_H3_Data_Check_And_Set(void)
{

    H3_data.Heater_enabled_state = 0;

    H3_data.Max_R = 0; // ???std::round????????????


    H3_data.Min_R = 0; // ???std::round????????????

    H3_data.Temp_Detection = 0;

    H3_data.Temp_limit = 0;

    H3_data.Kp = 0;

    H3_data.Ki = 0;

    H3_data.Kd = 0;


    H3_data.Resi_Protection = 0;

    H3_data.Temp_Protection = 0;


    H3_data.Low_TC_Set = 1;


    H3_data.High_TC_Set = 1;


    return true;

}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
uint8_t Widget::a_TableWidget_Data_Tran_Array(QTableWidget *TableWidget,Heat_Data_Set* Heat_Data,uint8_t ArrayNumber)
{

    uint8_t row = TableWidget->rowCount();
    uint8_t col = TableWidget->columnCount();
    bool check_state = true;
    qDebug() << row  << col;
    Heat_Data->Middle_count = 0;
    Heat_Data->TCR_TCR_count = 0;
    Heat_Data->TCR_Temp_count = 0;
    Heat_Data->TCR_Resi_count = 0;
    Heat_Data->Profile_Heat_Start_count = 0;
    Heat_Data->Profile_Heat_Stop_count = 0;
    Heat_Data->Profile_Heat_Number_count = 0;
    Heat_Data->Profile_Heat_Type_count = 0;


    if(TCR_T == ArrayNumber)
    {

        for (uint8_t j = 0; j < row; j++)
        {
            Heat_Data->TCR_Temp_count++;
            TCR_Temp[j] = QString(TableWidget->item(j, 1)->text()).toFloat();
            qDebug() << "TCR_Temp[j]" << TCR_Temp[j];

            if((TCR_Temp[j]<0.1)||(TCR_Temp[j]>450))
            {
                Heat_Data->TCR_Temp_count --;
                qDebug() << "Heat_Data->TCR_Temp_count" << Heat_Data->TCR_Temp_count;
                break;
            }
        }
        Heat_Data->Middle_count = Heat_Data->TCR_Temp_count;

        for (uint8_t j = 0; j < row; j++)
        {
            Heat_Data->TCR_Resi_count++;
            TCR_Temp[j] = QString(TableWidget->item(j, 2)->text()).toFloat();
            qDebug() << "TCR_Temp[j]" << TCR_Temp[j];

            if((TCR_Temp[j]<0.4)||(TCR_Temp[j]>2.5))
            {
                Heat_Data->TCR_Resi_count --;
                qDebug() << "Heat_Data->TCR_Resi_count" << Heat_Data->TCR_Resi_count;
                break;
            }
        }

        if(Heat_Data->Middle_count < Heat_Data->TCR_Resi_count)
        {
            Heat_Data->Middle_count = Heat_Data->TCR_Resi_count;
        }

        for (uint8_t j = 0; j < row; j++)
        {
            Heat_Data->TCR_TCR_count++;
            TCR_Temp[j] = QString(TableWidget->item(j, 3)->text()).toFloat();
            qDebug() << "TCR_Temp[j]" << TCR_Temp[j];

            if((TCR_Temp[j]<0.00001)||(TCR_Temp[j]>0.0034))
            {
                Heat_Data->TCR_TCR_count --;
                qDebug() << "Heat_Data->TCR_TCR_count" << Heat_Data->TCR_TCR_count;
                break;
            }
        }

        if(Heat_Data->Middle_count < Heat_Data->TCR_TCR_count)
        {
            Heat_Data->Middle_count = Heat_Data->TCR_TCR_count;
        }

        Heat_Data->TCRT_count = Heat_Data->Middle_count;
        for (uint8_t i = 1; i < (col); i++)
        {
            for (uint8_t j = 0; j < Heat_Data->TCRT_count; j++)
            {
                if(i == 1)
                {
                    Heat_Data->TCR_Temp[j] = QString(TableWidget->item(j, i)->text()).toUInt();
                    qDebug() << "test now TCR_Temp"  << Heat_Data->TCR_Temp[j];
                }
                else if(i == 2)
                {
                    Heat_Data->TCR_Resi[j] = TCR_Resi[j] = QString(TableWidget->item(j, i)->text()).toFloat();

                    qDebug() << "test now resi"  << Heat_Data->TCR_Resi[j];
                }
                else if(i == 3)
                {
                    Heat_Data->TCR_TCR[j] = TCR_TCR[j] = QString(TableWidget->item(j, i)->text()).toFloat();

                    qDebug() << "test now tcr"  << Heat_Data->TCR_TCR[j];
                }
            }
        }
        check_state = isValidTcrT(Heat_Data,Heat_Data->TCRT_count);
        Heat_Data->TCR_TCR_count = 0;
        Heat_Data->TCR_Temp_count = 0;
        Heat_Data->TCR_Resi_count = 0;


    }
    else if(PROFILE == ArrayNumber)
    {

        for (uint8_t j = 0; j < row; j++)
        {
            Heat_Data->Profile_Heat_Start_count++;
            TCR_Temp[j] = QString(TableWidget->item(j, 1)->text()).toFloat();
            qDebug() << "TCR_Temp[j]" << TCR_Temp[j];
            if(j > 0)
            {
                if((TCR_Temp[j]<0.1)||(TCR_Temp[j]>4500))
                {
                    if((TCR_Temp[0] == 0)&&(TCR_Temp[1] == 0))
                    {
                        Heat_Data->Profile_Heat_Start_count = 0;
                        qDebug() << "Heat_Data->Profile_Heat_Start_count" << Heat_Data->Profile_Heat_Start_count;
                        break;
                    }
                    else
                    {
                        Heat_Data->Profile_Heat_Start_count --;
                        qDebug() << "Heat_Data->Profile_Heat_Start_count" << Heat_Data->Profile_Heat_Start_count;
                        break;
                    }

                }
            }
        }
        Heat_Data->Middle_count = Heat_Data->Profile_Heat_Start_count;



        for (uint8_t j = 0; j < row; j++)
        {
            Heat_Data->Profile_Heat_Stop_count++;
            TCR_Temp[j] = QString(TableWidget->item(j, 2)->text()).toFloat();
            qDebug() << "TCR_Temp[j]" << TCR_Temp[j];

            if((TCR_Temp[j]<0.1)||(TCR_Temp[j]>4500))
            {
                Heat_Data->Profile_Heat_Stop_count --;
                qDebug() << "Heat_Data->Profile_Heat_Stop_count" << Heat_Data->Profile_Heat_Stop_count;
                break;
            }
        }
        if(Heat_Data->Middle_count < Heat_Data->Profile_Heat_Stop_count)
        {
            Heat_Data->Middle_count = Heat_Data->Profile_Heat_Stop_count;
        }


        for (uint8_t j = 0; j < row; j++)
        {
            Heat_Data->Profile_Heat_Type_count++;
            TCR_Temp[j] = QString(TableWidget->item(j, 3)->text()).toFloat();
            qDebug() << "TCR_Temp[j]" << TCR_Temp[j];

            if(TCR_Temp[j] == 0)
            {
                Heat_Data->Profile_Heat_Type_count --;
                qDebug() << "Heat_Data->Profile_Heat_Type_count" << Heat_Data->Profile_Heat_Type_count;
                break;
            }
        }

        if(Heat_Data->Middle_count < Heat_Data->Profile_Heat_Type_count)
        {
            Heat_Data->Middle_count = Heat_Data->Profile_Heat_Type_count;
        }

        for (uint8_t j = 0; j < row; j++)
        {
            Heat_Data->Profile_Heat_Number_count++;
            TCR_Temp[j] = QString(TableWidget->item(j, 4)->text()).toFloat();
            qDebug() << "TCR_Temp[j]" << TCR_Temp[j];

            if((TCR_Temp[j]<0.1)||(TCR_Temp[j]>450))
            {
                Heat_Data->Profile_Heat_Number_count --;
                qDebug() << "Heat_Data->Profile_Heat_Number_count" << Heat_Data->Profile_Heat_Number_count;
                break;
            }
        }

        if(Heat_Data->Middle_count < Heat_Data->Profile_Heat_Number_count)
        {
            Heat_Data->Middle_count = Heat_Data->Profile_Heat_Number_count;
        }

        Heat_Data->Profile_count = Heat_Data->Middle_count;

        for (uint8_t i = 1; i < (col); i++)
        {
            for (uint8_t j = 0; j < Heat_Data->Profile_count; j++)
            {
                if(i == 1)
                {
                    Heat_Data->Profile_Heat_Start_Time[j] = QString(TableWidget->item(j, i)->text()).toUInt();
                }
                else if(i == 2)
                {
                    Heat_Data->Profile_Heat_Stop_Time[j] = QString(TableWidget->item(j, i)->text()).toUInt();

                }
                else if(i == 3)
                {
                    Heat_Data->Profile_Heat_Type[j] = QString(TableWidget->item(j, i)->text());
                    qDebug() << Heat_Data->Profile_Heat_Type[j];
                }
                else if(i == 4)
                {
                    Heat_Data->Profile_Heat_Number[j] = QString(TableWidget->item(j, i)->text()).toFloat();
                    qDebug() <<"now value is data" <<Heat_Data->Profile_Heat_Number[j];

                    //Heat_Data->Profile_Heat_Number[j] = static_cast<int>(std::round(QString(TableWidget->item(j, i)->text()).toFloat()*100))/10; // ???std::round????????????

                }
            }
        }
        check_state = isValidProfile(Heat_Data,Heat_Data->Profile_count);
        Heat_Data->Profile_Heat_Start_count = 0;
        Heat_Data->Profile_Heat_Stop_count = 0;
        Heat_Data->Profile_Heat_Number_count = 0;
        Heat_Data->Profile_Heat_Type_count = 0;
    }
    return check_state;

}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Array_Data_Tran_TableWidget(QTableWidget *TableWidget,Heat_Data_Set* Heat_Data,uint8_t ArrayNumber)
{
    uint8_t row = TableWidget->rowCount();
    uint8_t col = TableWidget->columnCount();
    qDebug() << row  << col;
    QTableWidgetItem *ItemData[row];

    if(TCR_T == ArrayNumber)
    {
        for (uint8_t i = 1; i < (col); i++)
        {
            for (uint8_t j = 0; j < Heat_Data->TCRT_count; j++)
            {
                if(i == 1)
                {
                    //Heat_Data->TCR_Temp[j] = QString(TableWidget->item(j, i)->text()).toUInt();
                    /*if(Heat_Data->TCR_Temp[j]>500)
                    {
                        if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat1")
                        {
                            QMessageBox::information(this,"State"," TCR_T Heat1 Table Temp data error,Range(0-500)");
                        }
                        else if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat2")
                        {
                            QMessageBox::information(this,"State"," TCR_T Heat2 Table Temp data error,Range(0-500)");
                        }
                        else if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat3")
                        {

                            QMessageBox::information(this,"State"," TCR_T Heat3 Table Temp data error,Range(0-500)");
                        }
                        return;
                    }*/
                    ItemData[j] = new QTableWidgetItem(QString().number(Heat_Data->TCR_Temp[j]));
                    TableWidget->setItem(j, i, ItemData[j]);

                }
                else if(i == 2)
                {
                    //Heat_Data->TCR_Resi[j] = static_cast<int>(std::round(QString(TableWidget->item(j, i)->text()).toFloat()*10000))/10; // ???std::round????????????
                    /*if((Heat_Data->TCR_Resi[j]<0.4)||(Heat_Data->TCR_Resi[j]>2.5))
                    {
                        if(Heat_Data->TCR_Resi[j] != 0)
                        {
                            if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat1")
                            {
                                QMessageBox::information(this,"State"," TCR_T Heat1 Table Resi data error,Range(0.4-2.5)");
                            }
                            else if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat2")
                            {
                                QMessageBox::information(this,"State"," TCR_T Heat2 Table Resi data error,Range(0.4-2.5))");
                            }
                            else if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat3")
                            {
                                QMessageBox::information(this,"State"," TCR_T Heat3 Table Resi data error,Range(0.4-2.5)");
                            }
                            return;
                        }
                    }*/
                    ItemData[j] = new QTableWidgetItem(QString().number(Heat_Data->TCR_Resi[j]));
                    TableWidget->setItem(j, i, ItemData[j]);
                }
                else if(i == 3)
                {
                    //Heat_Data->TCR_Resi[j] = static_cast<int>(std::round(QString(TableWidget->item(j, i)->text()).toFloat()*10000))/10; // ???std::round????????????
                    if(Heat_Data->TCR_TCR[j] > 1)
                    {
                        /*if(Heat_Data->TCR_TCR[j] != 0)
                        {
                            if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat1")
                            {
                                QMessageBox::information(this,"State"," TCR_T Heat1 Table TCR data error,Range(<1)");
                            }
                            else if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat2")
                            {
                                QMessageBox::information(this,"State"," TCR_T Heat2 Table TCR data error,Range<1))");
                            }
                            else if(ui->BAT_Table_Heat_Seletion->currentText() == "Heat3")
                            {
                                QMessageBox::information(this,"State"," TCR_T Heat3 Table TCR data error,Range(<1)");
                            }
                            return;
                        }*/
                    }
                    ItemData[j] = new QTableWidgetItem(QString().number(Heat_Data->TCR_TCR[j]));
                    TableWidget->setItem(j, i, ItemData[j]);
                }
            }
        }
    }
    else if(PROFILE == ArrayNumber)
    {
        for (uint8_t i = 1; i < (col); i++)
        {
            for (uint8_t j = 0; j < Heat_Data->Profile_count; j++)
            {
                if(i == 1)
                {
                    //Heat_Data->Profile_Heat_Start_Time[j] = QString(TableWidget->item(j, i)->text()).toUInt();
                    /*if((Heat_Data->Profile_Heat_Start_Time[j]>5000))
                    {
                        if(TableWidget == ui->BAT_Table_Set_Profile1)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel1 Table Time start data error,Range(0-500)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile2)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel2 Table Time start data error,Range(0-500)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile3)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel3 Table Time start data error,Range(0-500)");
                        }
                        return;
                    }*/
                    ItemData[j] = new QTableWidgetItem(QString().number(Heat_Data->Profile_Heat_Start_Time[j]));
                    TableWidget->setItem(j, i, ItemData[j]);
                }
                else if(i == 2)
                {
                    //Heat_Data->Profile_Heat_Stop_Time[j] = QString(TableWidget->item(j, i)->text()).toUInt();
                    /*if((Heat_Data->Profile_Heat_Stop_Time[j]>5000))
                    {
                        if(TableWidget == ui->BAT_Table_Set_Profile1)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel1 Table Time stop data error,Range(0-500)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile2)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel2 Table Time stop data error,Range(0-500)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile3)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel3 Table Time stop data error,Range(0-500)");
                        }
                        return;
                    }*/
                    ItemData[j] = new QTableWidgetItem(QString().number(Heat_Data->Profile_Heat_Stop_Time[j]));
                    TableWidget->setItem(j, i, ItemData[j]);
                }
                else if(i == 3)
                {
                    //Heat_Data->Profile_Heat_Type[j] = QString(TableWidget->item(j, i)->text()).toUInt();
                    /*if((Heat_Data->Profile_Heat_Type[j]>3))
                    {
                        if(TableWidget == ui->BAT_Table_Set_Profile1)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel1 Table Type data error,Range(0-3)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile2)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel2 Table Type data error,Range(0-3)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile3)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel3 Table Type data error,Range(0-3)");
                        }
                        return;
                    }*/
                    ItemData[j] = new QTableWidgetItem(Heat_Data->Profile_Heat_Type[j]);
                    //ItemData[j] = new QTableWidgetItem(QString().number(Heat_Data->Profile_Heat_Type[j]));

                    TableWidget->setItem(j, i, ItemData[j]);
                }
                else if(i == 4)
                {
                    //Heat_Data->Profile_Heat_Number[j] = static_cast<int>(std::round(QString(TableWidget->item(j, i)->text()).toFloat()*100))/10; // ???std::round????????????
                    /*if((Heat_Data->Profile_Heat_Number[j]>450.0))
                    {
                        if(TableWidget == ui->BAT_Table_Set_Profile1)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel1 Table Target data error,Range(0-450)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile2)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel2 Table Target data error,Range(0-450)");
                        }
                        else if(TableWidget == ui->BAT_Table_Set_Profile3)
                        {
                            QMessageBox::information(this,"State"," Heat Profiel3 Table Target data error,Range(0-450)");
                        }
                        return;
                    }*/
                   ItemData[j] = new QTableWidgetItem(QString().number(Heat_Data->Profile_Heat_Number[j]));


                    TableWidget->setItem(j, i, ItemData[j]);
                }
            }
        }
    }
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
uint8_t Widget::a_Board_Base_Check_And_Set(void)
{

     Board_data.Vin = 0x01;


     Board_data.Ain = 0x01;


    Board_data.Vbd = 0x01;


    Board_data.Abd = 0x01;






    Board_data.TC1 = 0x01;
    Board_data.TC2 = 0x01;
    Board_data.TC3 = 0x01;
    Board_data.TC4 = 0x01;
    Board_data.TC5 = 0x01;
    Board_data.TC6 = 0x01;



    Board_data.Rh1 = 0x01;
    Board_data.Vh1 = 0x01;
    Board_data.Ah1 = 0x01;
    Board_data.Acp1 = 0x01;
    Board_data.Th1 = 0x01;
    Board_data.Dty1 = 0x01;
    Board_data.Pf1 = 0x01;






    Board_data.Rh2 = 0x01;
    Board_data.Vh2 = 0x01;
    Board_data.Ah2 = 0x01;
    Board_data.Acp2 = 0x01;
    Board_data.Th2 = 0x01;
    Board_data.Dty2 = 0x01;
    Board_data.Pf2 = 0x01;





    Board_data.Sampling_period = 1;

    qDebug() <<" Board_data.Sampling_period"  << Board_data.Sampling_period;



    if(ui->BAT_Share_pps->currentText() == "No Parallel")
    {
        Board_data.Priority_selection = 0;
    }
    else if(ui->BAT_Share_pps->currentText() == "Select sector1 H+")
    {
        Board_data.Priority_selection = 1;
    }
    else if(ui->BAT_Share_pps->currentText() == "Select sector2 H+")
    {
        Board_data.Priority_selection = 2;
    }
    else if(ui->BAT_Share_pps->currentText() == "Select sector3 H+")
    {
        Board_data.Priority_selection = 3;
    }

    qDebug() <<" ui->BAT_H_Priority_V->isChecked()" << ui->BAT_Share_pps->currentText() << "Board_data.Priority_selection" << Board_data.Priority_selection;




    Board_data.Pwm_Fre = 50;

    qDebug() <<" Board_data.Pwm_Fre"  << Board_data.Pwm_Fre;



    Board_data.P_comp1 = 0; // ???std::round????????????


    Board_data.T_comp1 = 0; // ???std::round????????????


    Board_data.V_comp1 = 0; // ???std::round????????????




    Board_data.P_comp2 = 0; // ???std::round????????????

    Board_data.T_comp2 = 0; // ???std::round????????????

    Board_data.V_comp2 = 0; // ???std::round????????????





    Board_data.P_comp3 = 0; // ???std::round????????????

    Board_data.T_comp3 = 0; // ???std::round????????????


    Board_data.V_comp3 = 0; // ???std::round????????????




    Board_data.Sampling_mode = 1;

    qDebug() <<" Board_data.Sampling_mode"  << Board_data.Sampling_mode;



    Board_data.Heat_loop_number = 1;


    Board_data.Heat_interval = 1;

    return true;

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Write_TCR_List_Flash_clicked()
{
    a_Uart_Write_Heat_Tcr_table();
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Read_TCR_List_Flash_clicked()
{

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Write_Board_baisic_info(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[35] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x11;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x45;


    SendBuffer[i++] = Board_data.Sampling_period/256;
    SendBuffer[i++] = Board_data.Sampling_period%256;
    SendBuffer[i++] = Board_data.Priority_selection;
    SendBuffer[i++] = Board_data.Pwm_Fre;



    SendBuffer[i++] = Board_data.P_comp1;
    SendBuffer[i++] = Board_data.T_comp1;
    SendBuffer[i++] = Board_data.V_comp1;



    SendBuffer[i++] = Board_data.P_comp2;
    SendBuffer[i++] = Board_data.T_comp2;
    SendBuffer[i++] = Board_data.V_comp2;



    SendBuffer[i++] = Board_data.P_comp3;
    SendBuffer[i++] = Board_data.T_comp3;
    SendBuffer[i++] = Board_data.V_comp3;


    SendBuffer[i++] = Board_data.Sampling_mode;
    SendBuffer[i++] = Board_data.Heat_loop_number;

    SendBuffer[i++] = Board_data.Heat_interval/256;
    SendBuffer[i++] = Board_data.Heat_interval%256;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Uart_Read_Board_baisic_info(void)
{
    uint8_t i = 0;
    uint8_t SendBuffer[5] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x10;

    SendBuffer[i++] = 0x45;

    qDebug()<<i;
    Uart_Send_CMD(SendBuffer,i);

}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Start_Heat_clicked()
{



    //a_Init_CustomPlot(ui->BAT_customplot_Heat,CustomNamestringList[0],CustomNamestringList[1],11,3,500,REAL_LOG,Qstring_Qcustomplot_display_Name);
    ui->BAT_SerialData_display->clear();
    Log_For_Act_display_count = 0;



    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x53;

    Uart_Send_CMD(SendBuffer,i);
}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Stop_Heat_clicked()
{
    uint8_t i = 0;
    uint8_t SendBuffer[6] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x54;


    Uart_Send_CMD(SendBuffer,i);
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Myself_Cali_Start_clicked()
{

}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_BAT_Myself_Cali_Stop_clicked()
{
    uint8_t i = 0;
    uint8_t SendBuffer[5] = {0};

    SendBuffer[i++] = 0xBA;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x00;
    SendBuffer[i++] = 0x10;
    SendBuffer[i++] = 0x56;

    Uart_Send_CMD(SendBuffer,i);
}


/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::on_Log_display_recover_clicked()
{


    ui->PPS_Volt_Log_State_act->setChecked(true);


     ui->TC1_Log_State_act->setChecked(true);


     ui->TC2_Log_State_act->setChecked(true);






     ui->HS1_Resi_Log_State_act->setChecked(true);




    ui->HS1_Vh1_Log_State_act->setChecked(true);
    ui->HS1_Ah1_Log_State_act->setChecked(true);


    ui->HS1_Duty_Log_State_act->setChecked(true);






    ui->HS2_Resi_Log_State_act->setChecked(true);




    ui->HS2_Vh2_Log_State_act->setChecked(true);
    ui->HS2_Ah2_Log_State_act->setChecked(true);

    ui->HS2_Duty_Log_State_act->setChecked(true);

}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/


void Widget::on_Log_display_change_clicked()
{
    uint8_t log_act_count = 0;

    if(true == ui->PPS_Volt_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }

    if(true == ui->TC1_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }

    if(true == ui->TC2_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }

    if(true == ui->HS1_Resi_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }

    if(true == ui->HS1_Vh1_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }

    if(true == ui->HS1_Ah1_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }

    if(true == ui->HS1_Duty_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }


    if(true == ui->HS2_Resi_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }


    if(true == ui->HS2_Vh2_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }

    if(true == ui->HS2_Ah2_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }


    if(true == ui->HS2_Duty_Log_State_act->isChecked())
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(true);
        Log_For_Act_PC[log_act_count++] = 0x01;
    }
    else
    {
        ui->BAT_customplot_Heat->graph(log_act_count)->setVisible(false);
        Log_For_Act_PC[log_act_count++] = 0x00;
    }


    // ui->BAT_customplot_Heat->replot(); // ⚠️ 注释掉同步重绘
        ui->BAT_customplot_Heat->replot(QCustomPlot::rpQueuedReplot); // 改为异步排队重绘

}





/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_Board_Base_Check_And_Set_ui(void)
{
    uint8_t rec_count = 5;
    Board_data.Sampling_period = ReceiveDataArray[rec_count++]*256;
    Board_data.Sampling_period += ReceiveDataArray[rec_count++];


    Board_data.Priority_selection = ReceiveDataArray[rec_count++];


    if(Board_data.Priority_selection == 0)
    {
        ui->BAT_Share_pps->setCurrentIndex(0); //     Ϊ ڶ   ѡ       0  ʼ  
    }
    else if(Board_data.Priority_selection == 1)
    {
        ui->BAT_Share_pps->setCurrentIndex(1); //     Ϊ ڶ   ѡ       0  ʼ  
    }

    rec_count++;



    Board_data.P_comp1 = ReceiveDataArray[rec_count++];
    Board_data.T_comp1 = ReceiveDataArray[rec_count++];
    Board_data.V_comp1 = ReceiveDataArray[rec_count++];
    Board_data.P_comp2 = ReceiveDataArray[rec_count++];
    Board_data.T_comp2 = ReceiveDataArray[rec_count++];
    Board_data.V_comp2 = ReceiveDataArray[rec_count++];


    Board_data.P_comp3 = ReceiveDataArray[rec_count++];
    Board_data.T_comp3 = ReceiveDataArray[rec_count++];
    Board_data.V_comp3 = ReceiveDataArray[rec_count++];

    Board_data.Sampling_mode = ReceiveDataArray[rec_count++];



    Board_data.Heat_loop_number = ReceiveDataArray[rec_count++];





    Board_data.Heat_interval = ReceiveDataArray[rec_count++]*256;
    Board_data.Heat_interval = Board_data.Heat_interval + ReceiveDataArray[rec_count++];


}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_H1_Data_Check_And_Set_ui(void)
{

    H1_data.Heater_enabled_state = ReceiveDataArray[6]*256 + ReceiveDataArray[7];
    if(H1_data.Heater_enabled_state == 1)
    {
        ui->BAT_H1_enabled_state->setChecked(true);
    }
    else
    {
        ui->BAT_H1_enabled_state->setChecked(false);
    }

    H1_data.Max_R = ReceiveDataArray[8]*256 + ReceiveDataArray[9];
    Middle_Num.setNum(((double)H1_data.Max_R/1000.000));
    ui->BAT_H1_Max_R->setText(Middle_Num);

    H1_data.Min_R = ReceiveDataArray[10]*256 + ReceiveDataArray[11];
    Middle_Num.setNum(((double)H1_data.Min_R/1000.000));
    ui->BAT_H1_Min_R->setText(Middle_Num);

    H1_data.Temp_Detection = ReceiveDataArray[12];


    H1_data.Temp_limit = ReceiveDataArray[13]*256 + ReceiveDataArray[14];
    Middle_Num.setNum((H1_data.Temp_limit));
    ui->BAT_H1_Temp_limit->setText(Middle_Num);

    H1_data.High_TC_Set = ReceiveDataArray[15];
    if((H1_data.High_TC_Set == 0)||(H1_data.High_TC_Set == 1))
    {
        ui->BAT_H1_TC_Seclect_Channel1->setCurrentIndex(0); //     Ϊ ڶ   ѡ       0  ʼ  

    }
    else
    {
        ui->BAT_H1_TC_Seclect_Channel1->setCurrentIndex(H1_data.High_TC_Set-1); //     Ϊ ڶ   ѡ       0  ʼ  
    }
    ui->BAT_H1_TC_Channel1_Check->setChecked(true);


    H1_data.Low_TC_Set = ReceiveDataArray[16];


    H1_data.Kp = ReceiveDataArray[17]*256 + ReceiveDataArray[18];
    Middle_Num.setNum((H1_data.Kp));
    ui->BAT_H1_Kp->setText(Middle_Num);

    H1_data.Ki = ReceiveDataArray[19]*256 + ReceiveDataArray[20];
    Middle_Num.setNum((H1_data.Ki));
    ui->BAT_H1_Ki->setText(Middle_Num);

    H1_data.Kd = ReceiveDataArray[21]*256 + ReceiveDataArray[22];
    Middle_Num.setNum((H1_data.Kd));
    ui->BAT_H1_Kd->setText(Middle_Num);



    H1_data.Resi_Protection = ReceiveDataArray[23];
    if(H1_data.Resi_Protection == 1)
    {
        ui->BAT_H1_Resi_Protection->setChecked(true);
    }
    else
    {
        ui->BAT_H1_Resi_Protection->setChecked(false);
    }


    H1_data.Temp_Protection =  ReceiveDataArray[24];
    if(H1_data.Temp_Protection == 1)
    {
        ui->BAT_H1_Temp_Protection->setChecked(true);
    }
    else
    {
        ui->BAT_H1_Temp_Protection->setChecked(false);
    }


}



/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_H2_Data_Check_And_Set_ui(void)
{

    H2_data.Heater_enabled_state = ReceiveDataArray[6]*256 + ReceiveDataArray[7];
    if(H2_data.Heater_enabled_state == 1)
    {
        ui->BAT_H2_enabled_state->setChecked(true);
    }
    else
    {
        ui->BAT_H2_enabled_state->setChecked(false);
    }

    H2_data.Max_R = ReceiveDataArray[8]*256 + ReceiveDataArray[9];
    Middle_Num.setNum(((double)H2_data.Max_R/1000.000));
    ui->BAT_H2_Max_R->setText(Middle_Num);

    H2_data.Min_R = ReceiveDataArray[10]*256 + ReceiveDataArray[11];
    Middle_Num.setNum(((double)H2_data.Min_R/1000.000));
    ui->BAT_H2_Min_R->setText(Middle_Num);

    H2_data.Temp_Detection = ReceiveDataArray[12];
    qDebug() << H2_data.Temp_Detection  <<   "H2_data.Temp_Detection";


    H2_data.Temp_limit = ReceiveDataArray[13]*256 + ReceiveDataArray[14];
    Middle_Num.setNum((H2_data.Temp_limit));
    ui->BAT_H2_Temp_limit->setText(Middle_Num);

    H2_data.High_TC_Set = ReceiveDataArray[15];
    if((H2_data.High_TC_Set == 0)||(H2_data.High_TC_Set == 1))
    {
        ui->BAT_H2_TC_Seclect_Channel1->setCurrentIndex(0); //     Ϊ ڶ   ѡ       0  ʼ  

    }
    else
    {
        ui->BAT_H2_TC_Seclect_Channel1->setCurrentIndex(H2_data.High_TC_Set-1); //     Ϊ ڶ   ѡ       0  ʼ  
    }
    ui->BAT_H2_TC_Channel1_Check->setChecked(true);


    H2_data.Low_TC_Set = ReceiveDataArray[16];





    H2_data.Kp = ReceiveDataArray[17]*256 + ReceiveDataArray[18];
    Middle_Num.setNum((H2_data.Kp));
    ui->BAT_H2_Kp->setText(Middle_Num);

    H2_data.Ki = ReceiveDataArray[19]*256 + ReceiveDataArray[20];
    Middle_Num.setNum((H2_data.Ki));
    ui->BAT_H2_Ki->setText(Middle_Num);

    H2_data.Kd = ReceiveDataArray[21]*256 + ReceiveDataArray[22];
    Middle_Num.setNum((H2_data.Kd));
    ui->BAT_H2_Kd->setText(Middle_Num);



    H2_data.Resi_Protection = ReceiveDataArray[23];
    if(H2_data.Resi_Protection == 1)
    {
        ui->BAT_H2_Resi_Protection->setChecked(true);
    }
    else
    {
        ui->BAT_H2_Resi_Protection->setChecked(false);
    }


    H2_data.Temp_Protection =  ReceiveDataArray[24];
    if(H2_data.Temp_Protection == 1)
    {
        ui->BAT_H2_Temp_Protection->setChecked(true);
    }
    else
    {
        ui->BAT_H2_Temp_Protection->setChecked(false);
    }

}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_H3_Data_Check_And_Set_ui(void)
{


}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_log_flag_set(void)
{
    std::fill(std::begin(Log_For_PC), std::end(Log_For_PC), 0);

    Log_For_PC[0] = 1;
    Log_For_PC[1] = 1;
    Log_For_PC[2] = 1;
    Log_For_PC[3] = 1;
    Log_For_PC[4] = 1;
    Log_For_PC[5] = 1;
    Log_For_PC[6] = 1;
    Log_For_PC[7] = 1;
    Log_For_PC[8] = 1;
    Log_For_PC[9] = 1;
    Log_For_PC[10] = 1;
    Log_For_PC[11] = 1;
    Log_For_PC[12] = 1;
    Log_For_PC[13] = 1;
    Log_For_PC[14] = 1;
    Log_For_PC[15] = 1;
    Log_For_PC[16] = 1;
    Log_For_PC[17] = 1;
    Log_For_PC[18] = 1;
    Log_For_PC[19] = 1;
    Log_For_PC[20] = 1;
    Log_For_PC[21] = 1;
    Log_For_PC[22] = 1;
    Log_For_PC[23] = 1;
    Log_For_PC[24] = 1;
    Log_For_PC[25] = 1;
    Log_For_PC[26] = 1;
    Log_For_PC[27] = 1;
    Log_For_PC[28] = 1;
    Log_For_PC[29] = 1;
    Log_For_PC[30] = 1;
    Log_For_PC[31] = 1;
    Log_For_PC[32] = 1;
    Log_For_PC[33] = 1;
    Log_For_PC[34] = 1;
    Log_For_PC[35] = 1;
}




/**************************************************************************
 * ????????
 * ????  ??
 * ????  ????
 * ????  ????
 *************************************************************************/
void Widget::a_log_flag_set_ui(void)
{





}


/**************************************************************************
*
*
*
*
 *************************************************************************/

bool Widget::canExtractFrame(const QByteArray &buffer)
{
    int pos = 0;
    while (pos <= buffer.size() - 7)
    {
        // 【关键修复 2】：必须查当前的 buffer.at(pos)，不能写死 buffer.at(0)
        if ((uint8_t)buffer.at(pos) == 187)
        {
            bool ok;
            // 从 pos 后面的位置读取长度
            int dataLength = buffer.mid(pos + 1, 2).toHex().toInt(&ok, 16);
            if (ok && pos + 7 + dataLength <= buffer.size())
            {
                return true;
            }
        }
        ++pos;
    }
    return false;
}

#if 0
bool Widget::canExtractFrame(const QByteArray &buffer)
{
    //    黺       Ƿ    㹻            һ            ֡
    //       Ҫ "bb" + 2 ֽڳ    + 1 ֽ ָ   + 2 ֽ CRC = 7 ֽ
    //   ʵ          Ҫ   ࣬  Ϊ   ݳ    ֶ ָ       ݲ  ֵĴ С
    if (buffer.size() < 7) return false;

    int pos = 0;
    while (pos < buffer.size() - 6)
    {

        if ((uint8_t)buffer.at(0) == 187)
        {
            bool ok;
            int dataLength = buffer.mid(1, 2).toHex().toInt(&ok, 16);
            if (ok && pos + 2 + 2 + dataLength + 2 <= buffer.size())
            {
                return true; //  ҵ   һ            ֡
            }
        }
        ++pos;
    }
    return false;
}
#endif


/**************************************************************************
*
*
*
*
 *************************************************************************/

QByteArray Widget::extractFrame(QByteArray &buffer)
{

    int pos = 0;
    while (pos <= buffer.size() - 7)
    {
        // 【关键修复 3】：同上，查当前的 buffer.at(pos)
        if ((uint8_t)buffer.at(pos) == 187)
        {
            bool ok;
            int dataLength = buffer.mid(pos + 1, 2).toHex().toInt(&ok, 16);
            if (ok && pos + 7 + dataLength <= buffer.size())
            {
                int totalLength = 7 + dataLength;
                QByteArray frame = buffer.mid(pos, totalLength);
                // 【关键修复 4】：把包头前面的垃圾错位数据，连同这帧完整数据一起删掉
                buffer.remove(0, pos + totalLength);
                return frame;
            }
        }
        ++pos;
    }
    return QByteArray();
}

#if 0
QByteArray Widget::extractFrame(QByteArray &buffer)
{
    int pos = 0;

    while (pos < buffer.size() - 6)
    {
        if ((uint8_t)buffer.at(0) == 187)
        {
            bool ok;
            int dataLength = buffer.mid(1, 2).toHex().toInt(&ok, 16);
            if (ok && pos + 2 + 2 + dataLength + 2 <= buffer.size())
            {
                QByteArray frame = buffer.mid(pos,1 + 2 + 2 + dataLength + 2);
                buffer.remove(0, 1 + 2 + 2 + dataLength + 2); //  ӻ        Ƴ  Ѵ         ֡
                return frame;
            }
        }
        ++pos;
    }
    //        ﷵ ؿգ   ζ  û   ҵ           ֡    ʵ            canExtractFrame       Ѿ       
    //              ϲ   ִ е     Ϊ ˴          Ի  Ǳ                 
    return QByteArray();
}

#endif


/**************************************************************************
*
*
*
*
 *************************************************************************/
//   ֤    
bool Widget::isValidProfile(Heat_Data_Set* Heat_Data, int numRows)
{
    if(numRows == 0)
    {
        return true;
    }

    for (int i = 0; i < numRows; i++)
    {
        //   ֤    
        if (Heat_Data->Profile_Heat_Type[i] != "P" && Heat_Data->Profile_Heat_Type[i] != "T" && Heat_Data->Profile_Heat_Type[i] != "V" && Heat_Data->Profile_Heat_Type[i] != "0")
        {
            if(Heat_Data == &H1_data)
            {
                QMessageBox::information(this,"State"," Heat profiel1 table type error");
            }
            else if(Heat_Data == &H2_data)
            {
                QMessageBox::information(this,"State"," Heat profiel2 table type error");
            }
            else if(Heat_Data == &H3_data)
            {
                QMessageBox::information(this,"State"," Heat profiel3 table type error");
            }
            return false;
        }

        //   ֤Ŀ  ֵ  Χ
        if (Heat_Data->Profile_Heat_Type[i] == "P" && (Heat_Data->Profile_Heat_Number[i] < MIN_P || Heat_Data->Profile_Heat_Number[i] > MAX_P))
        {
            if(Heat_Data == &H1_data)
            {
                QMessageBox::information(this,"State"," Heat profiel1 table power data error,range(0-90)");
            }
            else if(Heat_Data == &H2_data)
            {
                QMessageBox::information(this,"State"," Heat profiel2 table power data error,range(0-90)");
            }
            else if(Heat_Data == &H3_data)
            {
                QMessageBox::information(this,"State"," Heat profiel3 table power data error,range(0-90)");
            }
            return false;
        }

        //   ֤Ŀ  ֵ  Χ
        if(Heat_Data->Profile_Heat_Type[i] == "V" && (Heat_Data->Profile_Heat_Number[i] < MIN_V || Heat_Data->Profile_Heat_Number[i] > MAX_V))
        {
            if(Heat_Data == &H1_data)
            {
                QMessageBox::information(this,"State"," Heat profiel1 table volt data error,range(0-9)");
            }
            else if(Heat_Data == &H2_data)
            {
                QMessageBox::information(this,"State"," Heat profiel2 table volt data error,range(0-9)");
            }
            else if(Heat_Data == &H3_data)
            {
                QMessageBox::information(this,"State"," Heat profiel3 table volt data error,range(0-9)");
            }
            return false;
        }

        //   ֤Ŀ  ֵ  Χ
        if (Heat_Data->Profile_Heat_Type[i] == "T" && (Heat_Data->Profile_Heat_Number[i] < MIN_T || Heat_Data->Profile_Heat_Number[i] > MAX_T))
        {
            if(Heat_Data == &H1_data)
            {
                QMessageBox::information(this,"State"," Heat profiel1 table temp data error,range(100-450)");
            }
            else if(Heat_Data == &H2_data)
            {
                QMessageBox::information(this,"State"," Heat profiel2 table temp data error,range(100-450)");
            }
            else if(Heat_Data == &H3_data)
            {
                QMessageBox::information(this,"State"," Heat profiel3 table temp data error,range(100-450)");
            }
            return false;
        }

        //   ֤ʱ ䷶Χ
        if (Heat_Data->Profile_Heat_Start_Time[i] < 0 || Heat_Data->Profile_Heat_Stop_Time[i] > MAX_TIME || Heat_Data->Profile_Heat_Start_Time[i] > Heat_Data->Profile_Heat_Stop_Time[i])
        {

            if(Heat_Data == &H1_data)
            {
                QMessageBox::information(this,"State"," Heat profiel1 table time stop data error,range(0-450),start time > stop time");
            }
            else if(Heat_Data == &H2_data)
            {
                QMessageBox::information(this,"State"," Heat profiel2 table time stop data error,range(0-450),start time > stop time");
            }
            else if(Heat_Data == &H3_data)
            {
                QMessageBox::information(this,"State"," Heat profiel3 table time stop data error,range(0-450),start time > stop time");
            }

            return false;
        }

        //      ǵڶ  м  Ժ     ֤  ʼʱ    ǰһ н   ʱ   Ƿ   ͬ
        if (i > 0 && Heat_Data->Profile_Heat_Stop_Time[i-1] != Heat_Data->Profile_Heat_Start_Time[i])
        {
            if(Heat_Data == &H1_data)
            {
                QMessageBox::information(this,"State"," Heat profiel1 table time start time != last stop time");
            }
            else if(Heat_Data == &H2_data)
            {
                QMessageBox::information(this,"State"," Heat profiel2 table time start time != last stop time");
            }
            else if(Heat_Data == &H3_data)
            {
                QMessageBox::information(this,"State"," Heat profiel3 table time start time != last stop time");
            }
            return false;
        }
    }

    //   ֤  ʱ   Ƿ С  4500         Ǽ   ÿ  ʱ    end   ǽ       һ  ʱ    begin     ˵ һ к     һ  ֮   ļ      ǲ ֪    
    //          Ѿ   ֤  ÿ  ʱ    begin  end Ǻ    ģ                   ֻ  Ҫ      һ е begin      һ е end
    //         Ŀ˵  ÿһ ε         ʱ    4500      ʱ  ҲС  4500        ʵֻ  Ҫ    û  ʱ   γ   4500   ɣ        ˼򻯼  裩
    //     Ҫ ϸ   ֤  ʱ 䣬      Ҫ        Ϣ   ߼   ȷ      ֮   ļ         ʡ Ըò  ֣ 

    // ע ⣺        û   ϸ   ֤    ʱ  С  4500            Ϊ      Ŀ       Ѿ   ֤  ÿ  ʱ 䣬
    // ֻҪȷ  û е   ʱ 䳬  4500             Ϳ     Ϊ      ĿҪ        һ   򻯵Ĵ     

    return true;
}

/**************************************************************************
*
*
*
*
 *************************************************************************/
//   ֤    
bool Widget::isValidTcrT(Heat_Data_Set* Heat_Data, int numRows)
{

}



/**************************************************************************
*
*
*
*
**************************************************************************/
void Widget::on_BAT_Clean_Table_Set_Profile1_clicked()
{
    ui->BAT_Table_Set_Profile1->clear();

    Table_Init_Tcr_count = 0;
    QStringList HeadText2;
    HeadText2 << "Index" << "Begin(0.1s)" << "End(0.1s)" << "Type" << "Target Value";
    a_Init_TableWidget(ui->BAT_Table_Set_Profile1,TABLE_WIDTH_TCR,TABLE_HEATING_HORIZONTAL_NUMBER,Table_Heating_profiles1_count,TABLE_HEATING_VERTICAL_NUMBER,HeadText2);
}


/**************************************************************************
*
*
*
*
 *************************************************************************/
void Widget::on_BAT_Clean_Table_Set_Profile2_clicked()
{
    ui->BAT_Table_Set_Profile2->clear();

    Table_Init_Tcr_count = 0;
    QStringList HeadText2;
    HeadText2 << "Index" << "Begin(0.1s)" << "End(0.1s)" << "Type" << "Target Value";
    a_Init_TableWidget(ui->BAT_Table_Set_Profile2,TABLE_WIDTH_TCR,TABLE_HEATING_HORIZONTAL_NUMBER,Table_Heating_profiles1_count,TABLE_HEATING_VERTICAL_NUMBER,HeadText2);
}

/**************************************************************************
*
*
*
*
**************************************************************************/
void Widget::on_BAT_Clean_Table_Set_Profile3_clicked()
{

}

/**************************************************************************
*
*
*
*
**************************************************************************/

void Widget::on_Bat_export_flash_data_clicked()
{
     uint8_t i = 0,Send_count = 0;
     uint8_t SendBuffer[256] = {0};

     SendBuffer[i++] = 0xBA;
     SendBuffer[i++] = 0x00;
     SendBuffer[i++] = 0x00;
     SendBuffer[i++] = 0x10;

     SendBuffer[i++] = 0x5C;

     Uart_Send_CMD(SendBuffer,i);
}



/**************************************************************************
*4 װ     ˹
*
*
*
**************************************************************************/


// 26.6.10 mike 新建两个滤波器，低通




const double Widget::SOS_COEFFS[1][6] = {
    { 0.0304687473064697, 0.0304687473064697, 0.0,
      1.0, -0.939062505387061, 0.0 },
};



#if 0

// 26.6.10 mike 新建两个滤波器，高通
const double Widget::SOS_COEFFS[1][6] = {
    { 9.6953125269346970e-01, -9.6953125269346970e-01, 0.0000000000000000e+00,
      1.0000000000000000e+00, -9.3906250538693940e-01, 0.0000000000000000e+00 },
};

#endif

// 4  װ     ˹  ͨ ˲   ϵ   (0.02-0.20 Hz, fs=50Hz， time：26.6.3，purpose)




#if 0
const double Widget::SOS_COEFFS[1][6] = {
    { 2.4521609249465882e-02, 0.0000000000000000e+00, -2.4521609249465882e-02,
      1.0000000000000000e+00, -1.9460284953514277e+00, 9.5095678150106833e-01 },
};
#endif



#if 0

const double Widget::SOS_COEFFS[4][6] =
{
    {
        1.58872310e-08,  3.17744619e-08,  1.58872310e-08,
        1.0,            -1.96372568e+00,  9.64153485e-01
    },
    {
        1.0,             2.0,             1.0,
        1.0,            -1.98384485e+00,  9.84445873e-01
    },
    {
        1.0,            -2.0,             1.0,
        1.0,            -1.99471178e+00,  9.94720918e-01
    },
    {
        1.0,            -2.0,             1.0,
        1.0,            -1.99835997e+00,  9.98366551e-01
    }
};

#endif

// 26.6.3 mike 修改其他滤波器看是否23s卡死

#if 0
const double Widget::SOS_COEFFS[1][6] = {
    { 1, 1, 1,
      1, 1, 1 },
};
#endif




//        ׽  ˲  (Direct Form II Transposed)
double Widget::sosFilterSingle(const double sec[6], double in, SosState &state)
{
    double b0 = sec[0], b1 = sec[1], b2 = sec[2];
    double a1 = sec[4], a2 = sec[5]; // a0=1

    double out = b0 * in + state.z1;
    state.z1 = b1 * in - a1 * out + state.z2;
    state.z2 = b2 * in - a2 * out;
    return out;

}

//        ж  ׽ 
double Widget::sosFilterCascade(FilterChannel &ch, double in)
{
    double x = in;
    for (int s = 0; s < NUM_STAGES; s++) {

        // 26.6.3 mike 直接输出查看是否崩溃
         x = sosFilterSingle(SOS_COEFFS[s], x, ch.states[s]);
        //  x = x;

    }
    return x;
}

void Widget::initFilters()
{
    m_filtAx.reset();
    m_filtAy.reset();
    m_filtAz.reset();
    m_filtGx.reset();
    m_filtGy.reset();
    m_filtGz.reset();


}

void Widget::initHannWindow()
{
    for (int n = 0; n < WINDOW_SIZE; n++) {
        m_hannWindow[n] = 0.5 - 0.5 * cos(2.0 * M_PI * (double)n / (double)WINDOW_SIZE);
    }
}


void Widget::startSimulation()
{
    initFilters();
    initHannWindow();
    m_simTime = 0;
    m_simTimer->start(20); // 20ms ģ      
    qInfo() << "Simulation mode started.";
}

void Widget::onSimulationTimer()
{

}

void Widget::onSerialDataReady()
{

}

// ͨ     ݴ       
void Widget::processData(float rawAx, float rawAy, float rawAz,float rawGx, float rawGy, float rawGz)
{
    if(1 == Log_For_Act_display_count)
    {

        #if 0
        for (int g = 0; g < ui->BAT_Customplot_Set_Hannwindow->graphCount(); ++g)
            ui->BAT_Customplot_Set_Hannwindow->graph(g)->data()->clear();
        for (int g = 0; g < ui->BAT_Customplot_Set_Hannwindow_Gyro->graphCount(); ++g)
            ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(g)->data()->clear();
        #endif


        initFilterSteadyState(m_filtAx, rawAx);
        initFilterSteadyState(m_filtAy, rawAy);
        initFilterSteadyState(m_filtAz, rawAz);

        initFilterSteadyState(m_filtGx, rawGx);
        initFilterSteadyState(m_filtGy, rawGy);
        initFilterSteadyState(m_filtGz, rawGz);

        qDebug() << rawAx <<  rawAy <<  rawAz <<  rawGx <<  rawGy <<  rawGz;
    }

    // 2. ʵʱ ˲ 
    double filtAx = sosFilterCascade(m_filtAx, rawAx);
    double filtAy = sosFilterCascade(m_filtAy, rawAy);
    double filtAz = sosFilterCascade(m_filtAz, rawAz);

    double filtGx = sosFilterCascade(m_filtGx, rawGx);
    double filtGy = sosFilterCascade(m_filtGy, rawGy);
    double filtGz = sosFilterCascade(m_filtGz, rawGz);


    LOG_Accel_x_h[Log_For_Act_display_count-1] = filtAx;
    LOG_Accel_y_h[Log_For_Act_display_count-1] = filtAy;
    LOG_Accel_z_h[Log_For_Act_display_count-1] = filtAz;

    LOG_Gyro_x_h[Log_For_Act_display_count-1] = filtGx;
    LOG_Gyro_y_h[Log_For_Act_display_count-1] = filtGy;
    LOG_Gyro_z_h[Log_For_Act_display_count-1] = filtGz;



    // 实时绘图：addData + 20s 滑动窗口（原 %500 约 10s 才刷新一次）
    constexpr double WINDOW_SEC = 20.0;
    const double sampleHz = static_cast<double>(logSamplingRateHz());
    const double t = (static_cast<double>(Log_For_Act_display_count) - 1.0) / sampleHz;

    ui->BAT_Customplot_Set_Hannwindow->graph(0)->addData(t, filtAx);
    ui->BAT_Customplot_Set_Hannwindow->graph(1)->addData(t, filtAy);
    ui->BAT_Customplot_Set_Hannwindow->graph(2)->addData(t, filtAz);
    ui->BAT_Customplot_Set_Hannwindow->graph(0)->data()->removeBefore(t - WINDOW_SEC);
    ui->BAT_Customplot_Set_Hannwindow->graph(1)->data()->removeBefore(t - WINDOW_SEC);
    ui->BAT_Customplot_Set_Hannwindow->graph(2)->data()->removeBefore(t - WINDOW_SEC);

    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(0)->addData(t, filtGx);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(1)->addData(t, filtGy);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(2)->addData(t, filtGz);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(0)->data()->removeBefore(t - WINDOW_SEC);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(1)->data()->removeBefore(t - WINDOW_SEC);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(2)->data()->removeBefore(t - WINDOW_SEC);

    ui->BAT_Customplot_Set_Hannwindow->xAxis->setRange(t - WINDOW_SEC, t);
    ui->BAT_Customplot_Set_Hannwindow->yAxis->setRange(-1500, 1500);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setRange(t - WINDOW_SEC, t);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setRange(-1000, 1000);

    static uint8_t replot_div = 0;
    replot_div++;
    if (replot_div >= ACCEL_FILTER_REPLOT_INTERVAL)
    {
        replot_div = 0;
        ui->BAT_Customplot_Set_Hannwindow->replot(QCustomPlot::rpQueuedReplot);
        ui->BAT_Customplot_Set_Hannwindow_Gyro->replot(QCustomPlot::rpQueuedReplot);
    }

}





// mike 26.6.3  const int PRE_FILTER_POINTS = 0  查看影响

void Widget::initFilterSteadyState(FilterChannel &ch, double dcOffset)
{
    // ?     λ  ʼ        Ԥ ˲ N   㣬      ʼ  λƫ
    // Ԥ ˲     =3*fs/fc_low = 3*50/0.02=7500 㣬ȡ1000   㹻
    const int PRE_FILTER_POINTS = 200;

    ch.reset();
    for (int i = 0; i < PRE_FILTER_POINTS; i++) {
        sosFilterCascade(ch, dcOffset);
    }
}




#if 0
#endif




























#if 0

void Widget::updateRawData(float ax, float ay, float az)
{
    Q_UNUSED(ay); Q_UNUSED(az);

    m_timeData.append(m_currentTime);
    m_rawAxData.append(ax);
    m_currentTime += 0.02;

    //        ݵ        MAX_DATA_POINTS   
    if (m_timeData.size() > MAX_DATA_POINTS) {
        m_timeData.removeFirst();
        m_rawAxData.removeFirst();
    }

    //     ͼ  
    ui->BAT_Customplot_Set_Hannwindow->graph(0)->setData(m_timeData, m_rawAxData);
    ui->BAT_Customplot_Set_Hannwindow->xAxis->setRange(m_timeData.first(), m_timeData.last());


    //     ͼ  
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(0)->setData(m_timeData, m_rawAxData);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setRange(m_timeData.first(), m_timeData.last());

    //  Զ      Y  ᷶Χ
    double minVal = *std::min_element(m_rawAxData.constBegin(), m_rawAxData.constEnd());
    double maxVal = *std::max_element(m_rawAxData.constBegin(), m_rawAxData.constEnd());
    ui->BAT_Customplot_Set_Hannwindow->yAxis->setRange(minVal - 0.1, maxVal + 0.1);

    ui->BAT_Customplot_Set_Hannwindow->replot(QCustomPlot::rpQueuedReplot);


    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setRange(minVal - 0.1, maxVal + 0.1);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->replot(QCustomPlot::rpQueuedReplot);
}








void Widget::updateFilteredData(float ax, float ay, float az)
{
    Q_UNUSED(ay); Q_UNUSED(az);

    //     ʱ   ᣬֻ     ˲      Ax
    if (m_filtAxData.size() > MAX_DATA_POINTS) {
        m_filtAxData.removeFirst();
    }
    m_filtAxData.append(ax);

    if (m_filtAxData.size() == m_timeData.size()) {
        ui->BAT_Customplot_Set_Hannwindow->graph(0)->setData(m_timeData, m_filtAxData);
        ui->BAT_Customplot_Set_Hannwindow->xAxis->setRange(m_timeData.first(), m_timeData.last());

        //     ͼ  
        ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(0)->setData(m_timeData, m_rawAxData);
        ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setRange(m_timeData.first(), m_timeData.last());

        double minVal = *std::min_element(m_filtAxData.constBegin(), m_filtAxData.constEnd());
        double maxVal = *std::max_element(m_filtAxData.constBegin(), m_filtAxData.constEnd());
        ui->BAT_Customplot_Set_Hannwindow->yAxis->setRange(minVal - 0.1, maxVal + 0.1);
        ui->BAT_Customplot_Set_Hannwindow->replot(QCustomPlot::rpQueuedReplot);

        ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setRange(minVal - 0.1, maxVal + 0.1);
        ui->BAT_Customplot_Set_Hannwindow_Gyro->replot(QCustomPlot::rpQueuedReplot);
    }
}

void Widget::updateWindowedData(const QVector<double> &xAxis, const QVector<double> &data)
{
    ui->BAT_Customplot_Set_Hannwindow->graph(0)->setData(xAxis, data);
    ui->BAT_Customplot_Set_Hannwindow->xAxis->setRange(xAxis.first(), xAxis.last());

    //     ͼ  
    ui->BAT_Customplot_Set_Hannwindow_Gyro->graph(0)->setData(xAxis, data);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->xAxis->setRange(xAxis.first(), xAxis.last());

    double minVal = *std::min_element(data.constBegin(), data.constEnd());
    double maxVal = *std::max_element(data.constBegin(), data.constEnd());
    //ui->BAT_Customplot_Set_Hannwindow->yAxis->setRange(minVal - 0.1, maxVal + 0.1);
    ui->BAT_Customplot_Set_Hannwindow->yAxis->setRange(-1500, 1500);
    ui->BAT_Customplot_Set_Hannwindow->replot(QCustomPlot::rpQueuedReplot);


    ui->BAT_Customplot_Set_Hannwindow_Gyro->yAxis->setRange(-1500, 1500);
    ui->BAT_Customplot_Set_Hannwindow_Gyro->replot(QCustomPlot::rpQueuedReplot);

}

#endif


/*emit filteredDataReady(filtAx, filtAy, filtAz);

// 3.    뻷 λ        Ӵ
m_rbAx.push(filtAx);
if (m_rbAx.count == WINDOW_SIZE) {
    double tempData[WINDOW_SIZE];
    double windowedData[WINDOW_SIZE];
    m_rbAx.getAll(tempData);

    // Ӧ ú
    for (int i=0; i<WINDOW_SIZE; i++) {
        windowedData[i] = tempData[i] * m_hannWindow[i];
    }

    //     Ƶ     (   ڻ ͼ)
    QVector<double> freqAxis(WINDOW_SIZE/2 + 1);
    QVector<double> ampAxis(WINDOW_SIZE/2 + 1);

    //         ֻչʾ Ӵ     ʱ     ݣ     ҪFFT
    // Ϊ    ʾ     ǰѼӴ         ֱ ӵ       ֵ    ʾ
    QVector<double> timeAxis(WINDOW_SIZE);
    QVector<double> windowedVec(WINDOW_SIZE);
    for(int i=0; i<WINDOW_SIZE; i++) {
        timeAxis[i] = i * 0.02;
        windowedVec[i] = windowedData[i];
    }

    //windowedDataReady(timeAxis, windowedVec);
    updateWindowedData(timeAxis, windowedVec);
}*/


