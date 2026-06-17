#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QIcon>
#include <QAction>
#include <QPixmap>
#include <QMouseEvent>
#include <QSplitter>
#include "qcustomplot.h"
#include "Uart_Connect.h"
#include <algorithm>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QChar>
#include <QTimer>
#include <QTime>
#include <QAxObject>
#include <QDir>

/****************************************
 * 文件相关库函数
 * *************************************/
#include <QFile>
#include <QTextStream>
#include <QFileDialog>

#include <algorithm>

#include <cstring>
#include <cmath>
/*     math           */


/* 滤波器参数 */
#define NUM_STAGES 1          // 4 阶巴特沃斯（4 个二阶节）
#define WINDOW_SIZE 256        // 汉宁窗大小
#define FS 50.0f               // 采样率 50Hz








typedef struct
{
    uint16_t Heater_enabled_state;
    uint16_t Max_R;
    uint16_t Min_R;

    uint16_t Temp_Detection;

    uint16_t Temp_limit;

    uint16_t Resi_Protection;
    uint16_t Temp_Protection;

    uint16_t Kp;
    uint16_t Ki;
    uint16_t Kd;

    uint16_t High_TC_Set;
    uint16_t Low_TC_Set;


    uint16_t TCR_Temp[50];
    double TCR_Resi[50];
    double TCR_TCR[50];


    uint8_t TCRT_count;
    uint8_t Profile_count;
    uint8_t Middle_count;
    uint8_t Profile_Heat_Start_count;
    uint8_t Profile_Heat_Stop_count;
    uint8_t Profile_Heat_Type_count;
    uint8_t Profile_Heat_Number_count;
    uint8_t TCR_Temp_count;
    uint8_t TCR_Resi_count;
    uint8_t TCR_TCR_count;

    uint16_t Profile_Heat_Start_Time[20];
    uint16_t Profile_Heat_Stop_Time[20];
    QString Profile_Heat_Type[20];
    double Profile_Heat_Number[20];
}Heat_Data_Set;




typedef struct
{
    uint8_t Vin;
    uint8_t Ain;
    uint8_t Vbd;
    uint8_t Abd;
    uint8_t TC1;
    uint8_t TC2;
    uint8_t TC3;
    uint8_t TC4;
    uint8_t TC5;
    uint8_t TC6;

    uint8_t Rh1;
    uint8_t Vh1;
    uint8_t Ah1;
    uint8_t Acp1;
    uint8_t Th1;
    uint8_t Dty1;
    uint8_t Pf1;

    uint8_t Rh2;
    uint8_t Vh2;
    uint8_t Ah2;
    uint8_t Acp2;
    uint8_t Th2;
    uint8_t Dty2;
    uint8_t Pf2;

    uint8_t Rh3;
    uint8_t Vh3;
    uint8_t Ah3;
    uint8_t Acp3;
    uint8_t Th3;
    uint8_t Dty3;
    uint8_t Pf3;

    uint16_t Sampling_period;
    uint16_t Priority_selection;
    uint16_t Pwm_Fre;


    int16_t P_comp1;
    int16_t T_comp1;
    int16_t V_comp1;

    int16_t P_comp2;
    int16_t T_comp2;
    int16_t V_comp2;

    int16_t P_comp3;
    int16_t T_comp3;
    int16_t V_comp3;

    uint16_t R_Time_Log_State;
    uint16_t Sampling_mode;

    uint8_t Heat_loop_number;
    uint16_t Heat_interval;

    uint8_t heat_profile_time_flag;
    uint8_t heat_tcrt_time_flag;


    uint32_t heat_time_count;
}Board_Data_Base_Set;

enum DEVICE_STATE
{
    DEVICE_OFF = 0,
    DEVICE_ON,
    DEVICE_ERROR,

    DEVICE_MAX,
};


enum FLASH_TRAN_STATE
{
    SUCCESS = 0,
    ERR_RECV_TIMEOUT = 0xAB,
    ERR_PACKET_HEAD = 0xFE,
    ERR_PACKET_TAIL = 0xFD,
    ERR_PACKET_CRC = 0xFC,
    ERR_PACKET_NUM = 0xFB,
};

enum FLASH_EXPORT_STATE
{
    NOT_TRANSMITTED = 0,
    TRANSFERRING,
    TRANSMISSION_COMPLETED,
};


enum QCUSTOMPLOT_TYPE
{
    TCR_T = 0,
    PROFILE,
    REAL_LOG,
};
#define DEVICECONNECTING            1
#define DEVICEDISCONNECTING         0


QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    QTimer *CTimer;          //定时器
    QTime  CTimerCounter;    //计时器

    bool        serialExpectStatus = DEVICE_OFF;  // 用户选择的状态
    bool        serialRealStatus = DEVICE_OFF;    // 串口实际的状态
    QSerialPort *C_Serialport_Bat;

    void connectSignalSolt();
    void Uart_Send_CMD(unsigned char *addr, int num);
    void a_log_flag_set(void);
    QByteArray extractFrame(QByteArray &buffer);
    bool canExtractFrame(const QByteArray &buffer);
    bool isValidProfile(Heat_Data_Set* Heat_Data, int numRows);
    bool isValidTcrT(Heat_Data_Set* Heat_Data, int numRows);


    void startSimulation();

    void processData(float rawAx, float rawAy, float rawAz,float rawGx, float rawGy, float rawGz);

signals:

    // 信号：原始数据、滤波后的数据、加窗后的数据
    void rawDataReady(float ax, float ay, float az);
    void filteredDataReady(float ax, float ay, float az);
    void windowedDataReady(const QVector<double> &freqAxis, const QVector<double> &windowedAx);

protected:

    void closeEvent(QCloseEvent *event) override{


        //这里执行你的操作
        if(QMessageBox::Yes == QMessageBox::question(this, tr("退出程序"),
                                       QString(tr("确认退出程序?")),
                                       QMessageBox::Yes | QMessageBox::No))
        {
            event->accept();          //点击是即退出程序
        }
        else
        {
            event->ignore();          //点击否即不退出程序
        }

    }


    // mike 26.6.3

//public slots:
   // void updateRawData(float ax, float ay, float az);
    //void updateFilteredData(float ax, float ay, float az);
   // void updateWindowedData(const QVector<double> &xAxis, const QVector<double> &data);



private slots:
    void on_BAT_Clean_Table_Init_Tcr_clicked();

    void on_BAT_Save_Table_Init_Tcr_clicked();

    void on_BAT_Load_Table_Init_Tcr_clicked();

    void on_BAT_Save_Table_Set_Profile1_clicked();

    void on_BAT_Load_Table_Set_Profile1_clicked();

    void on_BAT_Save_Table_Set_Profile2_clicked();

    void on_BAT_Load_Table_Set_Profile2_clicked();

    void on_BAT_Save_Table_Set_Profile3_clicked();

    void on_BAT_Load_Table_Set_Profile3_clicked();

    void on_timer_timeout();

    void C_serialPortReadYRead_slot();

    void on_BAT_Key_Serial_Control_clicked();

    void on_BAT_Export_Flash_Data_clicked();

    void a_clean_Flash(void);

    void on_BAT_Clear_Screen_clicked();

    void on_BAT_Pause_clicked();

    void a_BAT_SerialData_display_Set(QString Display_Data,uint8_t Refresh_State);

    void QCustomplot_Display_Log_Proc(QCustomPlot *CustomPlot,uint32_t ArrayNumber);

    void Board_Save_Log_File_In_Pc(const QString &fileName, uint32_t ArrayNumber);

    void QCustomplot_Display_Data_Proc(QTableWidget *TableWidget,QCustomPlot *CustomPlot,uint8_t ArrayNumber);

    void Rec_Data_Proc(uint8_t *Rec);

    void on_BAT_Save_TCR_List_Flash_clicked();


    void on_BAT_Read_Configration_clicked();

    void on_BAT_Push_Configration_clicked();

    void a_Uart_Sync_time_to_device(void);

    uint8_t a_H1_Data_Check_And_Set(void);
    uint8_t a_H2_Data_Check_And_Set(void);
    uint8_t a_H3_Data_Check_And_Set(void);

    uint8_t a_Board_Base_Check_And_Set(void);

    void a_H1_Data_Check_And_Set_ui(void);
    void a_H2_Data_Check_And_Set_ui(void);
    void a_H3_Data_Check_And_Set_ui(void);

    void a_Board_Base_Check_And_Set_ui(void);


    void a_Uart_Write_H1_Setting(void);
    void a_Uart_Write_H2_Setting(void);
    void a_Uart_Write_H3_Setting(void);

    void a_Uart_Read_H1_Setting(void);
    void a_Uart_Read_H2_Setting(void);
    void a_Uart_Read_H3_Setting(void);

    void a_Uart_Write_Heat_Tcr_table(void);
    void a_Uart_Read_Heat_Tcr_table(void);


    void a_Uart_Write_H1_Profile(void);
    void a_Uart_Write_H2_Profile(void);
    void a_Uart_Write_H3_Profile(void);

    void a_Uart_Read_H1_Profile(void);
    void a_Uart_Read_H2_Profile(void);
    void a_Uart_Read_H3_Profile(void);

    void a_Uart_Write_Board_baisic_info(void);
    void a_Uart_Read_Board_baisic_info(void);

    void a_Read_board_basic_info_Get(void);
    void a_Write_Real_Time_Log_Set(void);


    uint8_t a_TableWidget_Data_Tran_Array(QTableWidget *TableWidget,Heat_Data_Set* Heat_Data,uint8_t ArrayNumber);

    void a_Array_Data_Tran_TableWidget(QTableWidget *TableWidget,Heat_Data_Set* Heat_Data,uint8_t ArrayNumber);


    void on_BAT_Display_Table_Set_Profile1_clicked();

    void on_BAT_Display_Table_Set_Profile2_clicked();

    void on_BAT_Display_Table_Set_Profile3_clicked();

    void on_BAT_Display_Table_Init_Tcr_clicked();

    void on_BAT_Write_TCR_List_Flash_clicked();

    void on_BAT_Read_TCR_List_Flash_clicked();

    void on_BAT_Clean_Flash_Data_clicked();

    void on_BAT_Start_Heat_clicked();

    void on_BAT_Stop_Heat_clicked();

    void on_BAT_Myself_Cali_Start_clicked();

    void on_BAT_Myself_Cali_Stop_clicked();

    void on_Log_display_recover_clicked();

    void on_Log_display_change_clicked();

    void a_log_flag_set_ui(void);


    void on_BAT_Clean_Table_Set_Profile1_clicked();

    void on_BAT_Clean_Table_Set_Profile2_clicked();

    void on_BAT_Clean_Table_Set_Profile3_clicked();

    void on_Bat_export_flash_data_clicked();

    void onSerialDataReady();
    void onSimulationTimer(); // 模拟数据定时器


private:
    Ui::Widget *ui;
    void a_Init_TableWidget(QTableWidget *TableWidget, uint16_t Table_Width,uint16_t Table_Horizontal_Number,uint16_t Table_Count,uint16_t Table_Vertical_Number,QStringList HeadText);
    void a_Init_CustomPlot(QCustomPlot *CustomPlot,QString xAxis_Label,QString yAxis_Label,uint8_t Graph_Count,uint16_t xAxis_Range,uint16_t yAxis_Range,uint8_t ArrayNumber,QStringList Qcustomplot_display_Name);
    void a_Save_TableWidget(QTableWidget *TableWidget);
    void a_Save_TableWidget_Tcr(void);
    void a_Load_TableWidget(QTableWidget *TableWidget,uint8_t ArrayNumber);


    // 单个二阶节状态
       struct SosState {
           double z1;
           double z2;
           SosState() : z1(0), z2(0) {}
       };

       // 滤波器通道（包含 4 个二阶节的状态）
       struct FilterChannel {
           SosState states[NUM_STAGES];
           void reset() { for (int i=0; i<NUM_STAGES; i++) states[i] = SosState(); }
       };

       // 环形缓冲区
       struct RingBuffer {
           double data[WINDOW_SIZE];
           int head;
           int count;
           RingBuffer() : head(0), count(0) { memset(data, 0, sizeof(data)); }
           void push(double val) {
               data[head] = val;
               head = (head + 1) % WINDOW_SIZE;
               if (count < WINDOW_SIZE) count++;
           }
           void getAll(double *out) {
               int start = (head - count + WINDOW_SIZE) % WINDOW_SIZE;
               for (int i = 0; i < count; i++)
                   out[i] = data[(start + i) % WINDOW_SIZE];
           }
       };

       // 纯 C++ 滤波器函数
       double sosFilterSingle(const double sec[6], double in, SosState &state);
       double sosFilterCascade(FilterChannel &ch,double in, const double coeffs[NUM_STAGES][6]);
       void initFilters();
       void initHannWindow();
       void initFilterSteadyState(FilterChannel &ch, double dcOffset);



       // 成员变量
       QTimer *m_simTimer;
       QByteArray m_serialBuffer;

       FilterChannel m_filtAx, m_filtAy, m_filtAz,m_filtGx, m_filtGy, m_filtGz;

       RingBuffer m_rbAx;
       double m_hannWindow[WINDOW_SIZE];
       double m_simTime;

       // 滤波器系数 (SOS 格式: [b0, b1, b2, a0, a1, a2], a0=1)

       static const double SOS_COEFFS_ACC[NUM_STAGES][6];
       static const double SOS_COEFFS_GYRO[NUM_STAGES][6];

       QVector<double> m_timeData;
       QVector<double> m_rawAxData;
       QVector<double> m_filtAxData;
       double m_currentTime;
       const int MAX_DATA_POINTS = 500; // 界面显示的最大点数
};
#endif // WIDGET_H
