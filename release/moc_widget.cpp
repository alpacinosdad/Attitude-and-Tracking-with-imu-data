/****************************************************************************
** Meta object code from reading C++ file 'widget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../widget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QVector>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Widget_t {
    QByteArrayData data[103];
    char stringdata0[2461];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Widget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Widget_t qt_meta_stringdata_Widget = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Widget"
QT_MOC_LITERAL(1, 7, 12), // "rawDataReady"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 2), // "ax"
QT_MOC_LITERAL(4, 24, 2), // "ay"
QT_MOC_LITERAL(5, 27, 2), // "az"
QT_MOC_LITERAL(6, 30, 17), // "filteredDataReady"
QT_MOC_LITERAL(7, 48, 17), // "windowedDataReady"
QT_MOC_LITERAL(8, 66, 15), // "QVector<double>"
QT_MOC_LITERAL(9, 82, 8), // "freqAxis"
QT_MOC_LITERAL(10, 91, 10), // "windowedAx"
QT_MOC_LITERAL(11, 102, 13), // "updateRawData"
QT_MOC_LITERAL(12, 116, 18), // "updateFilteredData"
QT_MOC_LITERAL(13, 135, 18), // "updateWindowedData"
QT_MOC_LITERAL(14, 154, 5), // "xAxis"
QT_MOC_LITERAL(15, 160, 4), // "data"
QT_MOC_LITERAL(16, 165, 35), // "on_BAT_Clean_Table_Init_Tcr_c..."
QT_MOC_LITERAL(17, 201, 34), // "on_BAT_Save_Table_Init_Tcr_cl..."
QT_MOC_LITERAL(18, 236, 34), // "on_BAT_Load_Table_Init_Tcr_cl..."
QT_MOC_LITERAL(19, 271, 38), // "on_BAT_Save_Table_Set_Profile..."
QT_MOC_LITERAL(20, 310, 38), // "on_BAT_Load_Table_Set_Profile..."
QT_MOC_LITERAL(21, 349, 38), // "on_BAT_Save_Table_Set_Profile..."
QT_MOC_LITERAL(22, 388, 38), // "on_BAT_Load_Table_Set_Profile..."
QT_MOC_LITERAL(23, 427, 38), // "on_BAT_Save_Table_Set_Profile..."
QT_MOC_LITERAL(24, 466, 38), // "on_BAT_Load_Table_Set_Profile..."
QT_MOC_LITERAL(25, 505, 16), // "on_timer_timeout"
QT_MOC_LITERAL(26, 522, 26), // "C_serialPortReadYRead_slot"
QT_MOC_LITERAL(27, 549, 33), // "on_BAT_Key_Serial_Control_cli..."
QT_MOC_LITERAL(28, 583, 32), // "on_BAT_Export_Flash_Data_clicked"
QT_MOC_LITERAL(29, 616, 13), // "a_clean_Flash"
QT_MOC_LITERAL(30, 630, 27), // "on_BAT_Clear_Screen_clicked"
QT_MOC_LITERAL(31, 658, 20), // "on_BAT_Pause_clicked"
QT_MOC_LITERAL(32, 679, 28), // "a_BAT_SerialData_display_Set"
QT_MOC_LITERAL(33, 708, 12), // "Display_Data"
QT_MOC_LITERAL(34, 721, 7), // "uint8_t"
QT_MOC_LITERAL(35, 729, 13), // "Refresh_State"
QT_MOC_LITERAL(36, 743, 28), // "QCustomplot_Display_Log_Proc"
QT_MOC_LITERAL(37, 772, 12), // "QCustomPlot*"
QT_MOC_LITERAL(38, 785, 10), // "CustomPlot"
QT_MOC_LITERAL(39, 796, 8), // "uint32_t"
QT_MOC_LITERAL(40, 805, 11), // "ArrayNumber"
QT_MOC_LITERAL(41, 817, 25), // "Board_Save_Log_File_In_Pc"
QT_MOC_LITERAL(42, 843, 8), // "fileName"
QT_MOC_LITERAL(43, 852, 29), // "QCustomplot_Display_Data_Proc"
QT_MOC_LITERAL(44, 882, 13), // "QTableWidget*"
QT_MOC_LITERAL(45, 896, 11), // "TableWidget"
QT_MOC_LITERAL(46, 908, 13), // "Rec_Data_Proc"
QT_MOC_LITERAL(47, 922, 8), // "uint8_t*"
QT_MOC_LITERAL(48, 931, 3), // "Rec"
QT_MOC_LITERAL(49, 935, 34), // "on_BAT_Save_TCR_List_Flash_cl..."
QT_MOC_LITERAL(50, 970, 32), // "on_BAT_Read_Configration_clicked"
QT_MOC_LITERAL(51, 1003, 32), // "on_BAT_Push_Configration_clicked"
QT_MOC_LITERAL(52, 1036, 26), // "a_Uart_Sync_time_to_device"
QT_MOC_LITERAL(53, 1063, 23), // "a_H1_Data_Check_And_Set"
QT_MOC_LITERAL(54, 1087, 23), // "a_H2_Data_Check_And_Set"
QT_MOC_LITERAL(55, 1111, 23), // "a_H3_Data_Check_And_Set"
QT_MOC_LITERAL(56, 1135, 26), // "a_Board_Base_Check_And_Set"
QT_MOC_LITERAL(57, 1162, 26), // "a_H1_Data_Check_And_Set_ui"
QT_MOC_LITERAL(58, 1189, 26), // "a_H2_Data_Check_And_Set_ui"
QT_MOC_LITERAL(59, 1216, 26), // "a_H3_Data_Check_And_Set_ui"
QT_MOC_LITERAL(60, 1243, 29), // "a_Board_Base_Check_And_Set_ui"
QT_MOC_LITERAL(61, 1273, 23), // "a_Uart_Write_H1_Setting"
QT_MOC_LITERAL(62, 1297, 23), // "a_Uart_Write_H2_Setting"
QT_MOC_LITERAL(63, 1321, 23), // "a_Uart_Write_H3_Setting"
QT_MOC_LITERAL(64, 1345, 22), // "a_Uart_Read_H1_Setting"
QT_MOC_LITERAL(65, 1368, 22), // "a_Uart_Read_H2_Setting"
QT_MOC_LITERAL(66, 1391, 22), // "a_Uart_Read_H3_Setting"
QT_MOC_LITERAL(67, 1414, 27), // "a_Uart_Write_Heat_Tcr_table"
QT_MOC_LITERAL(68, 1442, 26), // "a_Uart_Read_Heat_Tcr_table"
QT_MOC_LITERAL(69, 1469, 23), // "a_Uart_Write_H1_Profile"
QT_MOC_LITERAL(70, 1493, 23), // "a_Uart_Write_H2_Profile"
QT_MOC_LITERAL(71, 1517, 23), // "a_Uart_Write_H3_Profile"
QT_MOC_LITERAL(72, 1541, 22), // "a_Uart_Read_H1_Profile"
QT_MOC_LITERAL(73, 1564, 22), // "a_Uart_Read_H2_Profile"
QT_MOC_LITERAL(74, 1587, 22), // "a_Uart_Read_H3_Profile"
QT_MOC_LITERAL(75, 1610, 30), // "a_Uart_Write_Board_baisic_info"
QT_MOC_LITERAL(76, 1641, 29), // "a_Uart_Read_Board_baisic_info"
QT_MOC_LITERAL(77, 1671, 27), // "a_Read_board_basic_info_Get"
QT_MOC_LITERAL(78, 1699, 25), // "a_Write_Real_Time_Log_Set"
QT_MOC_LITERAL(79, 1725, 29), // "a_TableWidget_Data_Tran_Array"
QT_MOC_LITERAL(80, 1755, 14), // "Heat_Data_Set*"
QT_MOC_LITERAL(81, 1770, 9), // "Heat_Data"
QT_MOC_LITERAL(82, 1780, 29), // "a_Array_Data_Tran_TableWidget"
QT_MOC_LITERAL(83, 1810, 41), // "on_BAT_Display_Table_Set_Prof..."
QT_MOC_LITERAL(84, 1852, 41), // "on_BAT_Display_Table_Set_Prof..."
QT_MOC_LITERAL(85, 1894, 41), // "on_BAT_Display_Table_Set_Prof..."
QT_MOC_LITERAL(86, 1936, 37), // "on_BAT_Display_Table_Init_Tcr..."
QT_MOC_LITERAL(87, 1974, 35), // "on_BAT_Write_TCR_List_Flash_c..."
QT_MOC_LITERAL(88, 2010, 34), // "on_BAT_Read_TCR_List_Flash_cl..."
QT_MOC_LITERAL(89, 2045, 31), // "on_BAT_Clean_Flash_Data_clicked"
QT_MOC_LITERAL(90, 2077, 25), // "on_BAT_Start_Heat_clicked"
QT_MOC_LITERAL(91, 2103, 24), // "on_BAT_Stop_Heat_clicked"
QT_MOC_LITERAL(92, 2128, 32), // "on_BAT_Myself_Cali_Start_clicked"
QT_MOC_LITERAL(93, 2161, 31), // "on_BAT_Myself_Cali_Stop_clicked"
QT_MOC_LITERAL(94, 2193, 30), // "on_Log_display_recover_clicked"
QT_MOC_LITERAL(95, 2224, 29), // "on_Log_display_change_clicked"
QT_MOC_LITERAL(96, 2254, 17), // "a_log_flag_set_ui"
QT_MOC_LITERAL(97, 2272, 39), // "on_BAT_Clean_Table_Set_Profil..."
QT_MOC_LITERAL(98, 2312, 39), // "on_BAT_Clean_Table_Set_Profil..."
QT_MOC_LITERAL(99, 2352, 39), // "on_BAT_Clean_Table_Set_Profil..."
QT_MOC_LITERAL(100, 2392, 32), // "on_Bat_export_flash_data_clicked"
QT_MOC_LITERAL(101, 2425, 17), // "onSerialDataReady"
QT_MOC_LITERAL(102, 2443, 17) // "onSimulationTimer"

    },
    "Widget\0rawDataReady\0\0ax\0ay\0az\0"
    "filteredDataReady\0windowedDataReady\0"
    "QVector<double>\0freqAxis\0windowedAx\0"
    "updateRawData\0updateFilteredData\0"
    "updateWindowedData\0xAxis\0data\0"
    "on_BAT_Clean_Table_Init_Tcr_clicked\0"
    "on_BAT_Save_Table_Init_Tcr_clicked\0"
    "on_BAT_Load_Table_Init_Tcr_clicked\0"
    "on_BAT_Save_Table_Set_Profile1_clicked\0"
    "on_BAT_Load_Table_Set_Profile1_clicked\0"
    "on_BAT_Save_Table_Set_Profile2_clicked\0"
    "on_BAT_Load_Table_Set_Profile2_clicked\0"
    "on_BAT_Save_Table_Set_Profile3_clicked\0"
    "on_BAT_Load_Table_Set_Profile3_clicked\0"
    "on_timer_timeout\0C_serialPortReadYRead_slot\0"
    "on_BAT_Key_Serial_Control_clicked\0"
    "on_BAT_Export_Flash_Data_clicked\0"
    "a_clean_Flash\0on_BAT_Clear_Screen_clicked\0"
    "on_BAT_Pause_clicked\0a_BAT_SerialData_display_Set\0"
    "Display_Data\0uint8_t\0Refresh_State\0"
    "QCustomplot_Display_Log_Proc\0QCustomPlot*\0"
    "CustomPlot\0uint32_t\0ArrayNumber\0"
    "Board_Save_Log_File_In_Pc\0fileName\0"
    "QCustomplot_Display_Data_Proc\0"
    "QTableWidget*\0TableWidget\0Rec_Data_Proc\0"
    "uint8_t*\0Rec\0on_BAT_Save_TCR_List_Flash_clicked\0"
    "on_BAT_Read_Configration_clicked\0"
    "on_BAT_Push_Configration_clicked\0"
    "a_Uart_Sync_time_to_device\0"
    "a_H1_Data_Check_And_Set\0a_H2_Data_Check_And_Set\0"
    "a_H3_Data_Check_And_Set\0"
    "a_Board_Base_Check_And_Set\0"
    "a_H1_Data_Check_And_Set_ui\0"
    "a_H2_Data_Check_And_Set_ui\0"
    "a_H3_Data_Check_And_Set_ui\0"
    "a_Board_Base_Check_And_Set_ui\0"
    "a_Uart_Write_H1_Setting\0a_Uart_Write_H2_Setting\0"
    "a_Uart_Write_H3_Setting\0a_Uart_Read_H1_Setting\0"
    "a_Uart_Read_H2_Setting\0a_Uart_Read_H3_Setting\0"
    "a_Uart_Write_Heat_Tcr_table\0"
    "a_Uart_Read_Heat_Tcr_table\0"
    "a_Uart_Write_H1_Profile\0a_Uart_Write_H2_Profile\0"
    "a_Uart_Write_H3_Profile\0a_Uart_Read_H1_Profile\0"
    "a_Uart_Read_H2_Profile\0a_Uart_Read_H3_Profile\0"
    "a_Uart_Write_Board_baisic_info\0"
    "a_Uart_Read_Board_baisic_info\0"
    "a_Read_board_basic_info_Get\0"
    "a_Write_Real_Time_Log_Set\0"
    "a_TableWidget_Data_Tran_Array\0"
    "Heat_Data_Set*\0Heat_Data\0"
    "a_Array_Data_Tran_TableWidget\0"
    "on_BAT_Display_Table_Set_Profile1_clicked\0"
    "on_BAT_Display_Table_Set_Profile2_clicked\0"
    "on_BAT_Display_Table_Set_Profile3_clicked\0"
    "on_BAT_Display_Table_Init_Tcr_clicked\0"
    "on_BAT_Write_TCR_List_Flash_clicked\0"
    "on_BAT_Read_TCR_List_Flash_clicked\0"
    "on_BAT_Clean_Flash_Data_clicked\0"
    "on_BAT_Start_Heat_clicked\0"
    "on_BAT_Stop_Heat_clicked\0"
    "on_BAT_Myself_Cali_Start_clicked\0"
    "on_BAT_Myself_Cali_Stop_clicked\0"
    "on_Log_display_recover_clicked\0"
    "on_Log_display_change_clicked\0"
    "a_log_flag_set_ui\0"
    "on_BAT_Clean_Table_Set_Profile1_clicked\0"
    "on_BAT_Clean_Table_Set_Profile2_clicked\0"
    "on_BAT_Clean_Table_Set_Profile3_clicked\0"
    "on_Bat_export_flash_data_clicked\0"
    "onSerialDataReady\0onSimulationTimer"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Widget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      79,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,  409,    2, 0x06 /* Public */,
       6,    3,  416,    2, 0x06 /* Public */,
       7,    2,  423,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    3,  428,    2, 0x0a /* Public */,
      12,    3,  435,    2, 0x0a /* Public */,
      13,    2,  442,    2, 0x0a /* Public */,
      16,    0,  447,    2, 0x08 /* Private */,
      17,    0,  448,    2, 0x08 /* Private */,
      18,    0,  449,    2, 0x08 /* Private */,
      19,    0,  450,    2, 0x08 /* Private */,
      20,    0,  451,    2, 0x08 /* Private */,
      21,    0,  452,    2, 0x08 /* Private */,
      22,    0,  453,    2, 0x08 /* Private */,
      23,    0,  454,    2, 0x08 /* Private */,
      24,    0,  455,    2, 0x08 /* Private */,
      25,    0,  456,    2, 0x08 /* Private */,
      26,    0,  457,    2, 0x08 /* Private */,
      27,    0,  458,    2, 0x08 /* Private */,
      28,    0,  459,    2, 0x08 /* Private */,
      29,    0,  460,    2, 0x08 /* Private */,
      30,    0,  461,    2, 0x08 /* Private */,
      31,    0,  462,    2, 0x08 /* Private */,
      32,    2,  463,    2, 0x08 /* Private */,
      36,    2,  468,    2, 0x08 /* Private */,
      41,    2,  473,    2, 0x08 /* Private */,
      43,    3,  478,    2, 0x08 /* Private */,
      46,    1,  485,    2, 0x08 /* Private */,
      49,    0,  488,    2, 0x08 /* Private */,
      50,    0,  489,    2, 0x08 /* Private */,
      51,    0,  490,    2, 0x08 /* Private */,
      52,    0,  491,    2, 0x08 /* Private */,
      53,    0,  492,    2, 0x08 /* Private */,
      54,    0,  493,    2, 0x08 /* Private */,
      55,    0,  494,    2, 0x08 /* Private */,
      56,    0,  495,    2, 0x08 /* Private */,
      57,    0,  496,    2, 0x08 /* Private */,
      58,    0,  497,    2, 0x08 /* Private */,
      59,    0,  498,    2, 0x08 /* Private */,
      60,    0,  499,    2, 0x08 /* Private */,
      61,    0,  500,    2, 0x08 /* Private */,
      62,    0,  501,    2, 0x08 /* Private */,
      63,    0,  502,    2, 0x08 /* Private */,
      64,    0,  503,    2, 0x08 /* Private */,
      65,    0,  504,    2, 0x08 /* Private */,
      66,    0,  505,    2, 0x08 /* Private */,
      67,    0,  506,    2, 0x08 /* Private */,
      68,    0,  507,    2, 0x08 /* Private */,
      69,    0,  508,    2, 0x08 /* Private */,
      70,    0,  509,    2, 0x08 /* Private */,
      71,    0,  510,    2, 0x08 /* Private */,
      72,    0,  511,    2, 0x08 /* Private */,
      73,    0,  512,    2, 0x08 /* Private */,
      74,    0,  513,    2, 0x08 /* Private */,
      75,    0,  514,    2, 0x08 /* Private */,
      76,    0,  515,    2, 0x08 /* Private */,
      77,    0,  516,    2, 0x08 /* Private */,
      78,    0,  517,    2, 0x08 /* Private */,
      79,    3,  518,    2, 0x08 /* Private */,
      82,    3,  525,    2, 0x08 /* Private */,
      83,    0,  532,    2, 0x08 /* Private */,
      84,    0,  533,    2, 0x08 /* Private */,
      85,    0,  534,    2, 0x08 /* Private */,
      86,    0,  535,    2, 0x08 /* Private */,
      87,    0,  536,    2, 0x08 /* Private */,
      88,    0,  537,    2, 0x08 /* Private */,
      89,    0,  538,    2, 0x08 /* Private */,
      90,    0,  539,    2, 0x08 /* Private */,
      91,    0,  540,    2, 0x08 /* Private */,
      92,    0,  541,    2, 0x08 /* Private */,
      93,    0,  542,    2, 0x08 /* Private */,
      94,    0,  543,    2, 0x08 /* Private */,
      95,    0,  544,    2, 0x08 /* Private */,
      96,    0,  545,    2, 0x08 /* Private */,
      97,    0,  546,    2, 0x08 /* Private */,
      98,    0,  547,    2, 0x08 /* Private */,
      99,    0,  548,    2, 0x08 /* Private */,
     100,    0,  549,    2, 0x08 /* Private */,
     101,    0,  550,    2, 0x08 /* Private */,
     102,    0,  551,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,
    QMetaType::Void, 0x80000000 | 8, 0x80000000 | 8,    9,   10,

 // slots: parameters
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,
    QMetaType::Void, 0x80000000 | 8, 0x80000000 | 8,   14,   15,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 34,   33,   35,
    QMetaType::Void, 0x80000000 | 37, 0x80000000 | 39,   38,   40,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 39,   42,   40,
    QMetaType::Void, 0x80000000 | 44, 0x80000000 | 37, 0x80000000 | 34,   45,   38,   40,
    QMetaType::Void, 0x80000000 | 47,   48,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 34,
    0x80000000 | 34,
    0x80000000 | 34,
    0x80000000 | 34,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 34, 0x80000000 | 44, 0x80000000 | 80, 0x80000000 | 34,   45,   81,   40,
    QMetaType::Void, 0x80000000 | 44, 0x80000000 | 80, 0x80000000 | 34,   45,   81,   40,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Widget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->rawDataReady((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 1: _t->filteredDataReady((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 2: _t->windowedDataReady((*reinterpret_cast< const QVector<double>(*)>(_a[1])),(*reinterpret_cast< const QVector<double>(*)>(_a[2]))); break;
        case 3: _t->updateRawData((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 4: _t->updateFilteredData((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 5: _t->updateWindowedData((*reinterpret_cast< const QVector<double>(*)>(_a[1])),(*reinterpret_cast< const QVector<double>(*)>(_a[2]))); break;
        case 6: _t->on_BAT_Clean_Table_Init_Tcr_clicked(); break;
        case 7: _t->on_BAT_Save_Table_Init_Tcr_clicked(); break;
        case 8: _t->on_BAT_Load_Table_Init_Tcr_clicked(); break;
        case 9: _t->on_BAT_Save_Table_Set_Profile1_clicked(); break;
        case 10: _t->on_BAT_Load_Table_Set_Profile1_clicked(); break;
        case 11: _t->on_BAT_Save_Table_Set_Profile2_clicked(); break;
        case 12: _t->on_BAT_Load_Table_Set_Profile2_clicked(); break;
        case 13: _t->on_BAT_Save_Table_Set_Profile3_clicked(); break;
        case 14: _t->on_BAT_Load_Table_Set_Profile3_clicked(); break;
        case 15: _t->on_timer_timeout(); break;
        case 16: _t->C_serialPortReadYRead_slot(); break;
        case 17: _t->on_BAT_Key_Serial_Control_clicked(); break;
        case 18: _t->on_BAT_Export_Flash_Data_clicked(); break;
        case 19: _t->a_clean_Flash(); break;
        case 20: _t->on_BAT_Clear_Screen_clicked(); break;
        case 21: _t->on_BAT_Pause_clicked(); break;
        case 22: _t->a_BAT_SerialData_display_Set((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< uint8_t(*)>(_a[2]))); break;
        case 23: _t->QCustomplot_Display_Log_Proc((*reinterpret_cast< QCustomPlot*(*)>(_a[1])),(*reinterpret_cast< uint32_t(*)>(_a[2]))); break;
        case 24: _t->Board_Save_Log_File_In_Pc((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< uint32_t(*)>(_a[2]))); break;
        case 25: _t->QCustomplot_Display_Data_Proc((*reinterpret_cast< QTableWidget*(*)>(_a[1])),(*reinterpret_cast< QCustomPlot*(*)>(_a[2])),(*reinterpret_cast< uint8_t(*)>(_a[3]))); break;
        case 26: _t->Rec_Data_Proc((*reinterpret_cast< uint8_t*(*)>(_a[1]))); break;
        case 27: _t->on_BAT_Save_TCR_List_Flash_clicked(); break;
        case 28: _t->on_BAT_Read_Configration_clicked(); break;
        case 29: _t->on_BAT_Push_Configration_clicked(); break;
        case 30: _t->a_Uart_Sync_time_to_device(); break;
        case 31: { uint8_t _r = _t->a_H1_Data_Check_And_Set();
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 32: { uint8_t _r = _t->a_H2_Data_Check_And_Set();
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 33: { uint8_t _r = _t->a_H3_Data_Check_And_Set();
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 34: { uint8_t _r = _t->a_Board_Base_Check_And_Set();
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 35: _t->a_H1_Data_Check_And_Set_ui(); break;
        case 36: _t->a_H2_Data_Check_And_Set_ui(); break;
        case 37: _t->a_H3_Data_Check_And_Set_ui(); break;
        case 38: _t->a_Board_Base_Check_And_Set_ui(); break;
        case 39: _t->a_Uart_Write_H1_Setting(); break;
        case 40: _t->a_Uart_Write_H2_Setting(); break;
        case 41: _t->a_Uart_Write_H3_Setting(); break;
        case 42: _t->a_Uart_Read_H1_Setting(); break;
        case 43: _t->a_Uart_Read_H2_Setting(); break;
        case 44: _t->a_Uart_Read_H3_Setting(); break;
        case 45: _t->a_Uart_Write_Heat_Tcr_table(); break;
        case 46: _t->a_Uart_Read_Heat_Tcr_table(); break;
        case 47: _t->a_Uart_Write_H1_Profile(); break;
        case 48: _t->a_Uart_Write_H2_Profile(); break;
        case 49: _t->a_Uart_Write_H3_Profile(); break;
        case 50: _t->a_Uart_Read_H1_Profile(); break;
        case 51: _t->a_Uart_Read_H2_Profile(); break;
        case 52: _t->a_Uart_Read_H3_Profile(); break;
        case 53: _t->a_Uart_Write_Board_baisic_info(); break;
        case 54: _t->a_Uart_Read_Board_baisic_info(); break;
        case 55: _t->a_Read_board_basic_info_Get(); break;
        case 56: _t->a_Write_Real_Time_Log_Set(); break;
        case 57: { uint8_t _r = _t->a_TableWidget_Data_Tran_Array((*reinterpret_cast< QTableWidget*(*)>(_a[1])),(*reinterpret_cast< Heat_Data_Set*(*)>(_a[2])),(*reinterpret_cast< uint8_t(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 58: _t->a_Array_Data_Tran_TableWidget((*reinterpret_cast< QTableWidget*(*)>(_a[1])),(*reinterpret_cast< Heat_Data_Set*(*)>(_a[2])),(*reinterpret_cast< uint8_t(*)>(_a[3]))); break;
        case 59: _t->on_BAT_Display_Table_Set_Profile1_clicked(); break;
        case 60: _t->on_BAT_Display_Table_Set_Profile2_clicked(); break;
        case 61: _t->on_BAT_Display_Table_Set_Profile3_clicked(); break;
        case 62: _t->on_BAT_Display_Table_Init_Tcr_clicked(); break;
        case 63: _t->on_BAT_Write_TCR_List_Flash_clicked(); break;
        case 64: _t->on_BAT_Read_TCR_List_Flash_clicked(); break;
        case 65: _t->on_BAT_Clean_Flash_Data_clicked(); break;
        case 66: _t->on_BAT_Start_Heat_clicked(); break;
        case 67: _t->on_BAT_Stop_Heat_clicked(); break;
        case 68: _t->on_BAT_Myself_Cali_Start_clicked(); break;
        case 69: _t->on_BAT_Myself_Cali_Stop_clicked(); break;
        case 70: _t->on_Log_display_recover_clicked(); break;
        case 71: _t->on_Log_display_change_clicked(); break;
        case 72: _t->a_log_flag_set_ui(); break;
        case 73: _t->on_BAT_Clean_Table_Set_Profile1_clicked(); break;
        case 74: _t->on_BAT_Clean_Table_Set_Profile2_clicked(); break;
        case 75: _t->on_BAT_Clean_Table_Set_Profile3_clicked(); break;
        case 76: _t->on_Bat_export_flash_data_clicked(); break;
        case 77: _t->onSerialDataReady(); break;
        case 78: _t->onSimulationTimer(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<double> >(); break;
            }
            break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QVector<double> >(); break;
            }
            break;
        case 23:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QCustomPlot* >(); break;
            }
            break;
        case 25:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QCustomPlot* >(); break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QTableWidget* >(); break;
            }
            break;
        case 57:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QTableWidget* >(); break;
            }
            break;
        case 58:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QTableWidget* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Widget::*)(float , float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Widget::rawDataReady)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Widget::*)(float , float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Widget::filteredDataReady)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Widget::*)(const QVector<double> & , const QVector<double> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Widget::windowedDataReady)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Widget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_Widget.data,
    qt_meta_data_Widget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Widget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Widget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 79)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 79;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 79)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 79;
    }
    return _id;
}

// SIGNAL 0
void Widget::rawDataReady(float _t1, float _t2, float _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Widget::filteredDataReady(float _t1, float _t2, float _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Widget::windowedDataReady(const QVector<double> & _t1, const QVector<double> & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
