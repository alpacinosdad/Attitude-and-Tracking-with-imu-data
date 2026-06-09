/****************************************************************************
** Meta object code from reading C++ file 'widget.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.11.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../widget.h"
#include <QtGui/qtextcursor.h>
#include <QtGui/qscreen.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.11.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN6WidgetE_t {};
} // unnamed namespace

template <> constexpr inline auto Widget::qt_create_metaobjectdata<qt_meta_tag_ZN6WidgetE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "Widget",
        "rawDataReady",
        "",
        "ax",
        "ay",
        "az",
        "filteredDataReady",
        "windowedDataReady",
        "QList<double>",
        "freqAxis",
        "windowedAx",
        "on_BAT_Clean_Table_Init_Tcr_clicked",
        "on_BAT_Save_Table_Init_Tcr_clicked",
        "on_BAT_Load_Table_Init_Tcr_clicked",
        "on_BAT_Save_Table_Set_Profile1_clicked",
        "on_BAT_Load_Table_Set_Profile1_clicked",
        "on_BAT_Save_Table_Set_Profile2_clicked",
        "on_BAT_Load_Table_Set_Profile2_clicked",
        "on_BAT_Save_Table_Set_Profile3_clicked",
        "on_BAT_Load_Table_Set_Profile3_clicked",
        "on_timer_timeout",
        "C_serialPortReadYRead_slot",
        "on_BAT_Key_Serial_Control_clicked",
        "on_BAT_Export_Flash_Data_clicked",
        "a_clean_Flash",
        "on_BAT_Clear_Screen_clicked",
        "on_BAT_Pause_clicked",
        "a_BAT_SerialData_display_Set",
        "Display_Data",
        "uint8_t",
        "Refresh_State",
        "QCustomplot_Display_Log_Proc",
        "QCustomPlot*",
        "CustomPlot",
        "uint32_t",
        "ArrayNumber",
        "Board_Save_Log_File_In_Pc",
        "fileName",
        "QCustomplot_Display_Data_Proc",
        "QTableWidget*",
        "TableWidget",
        "Rec_Data_Proc",
        "uint8_t*",
        "Rec",
        "on_BAT_Save_TCR_List_Flash_clicked",
        "on_BAT_Read_Configration_clicked",
        "on_BAT_Push_Configration_clicked",
        "a_Uart_Sync_time_to_device",
        "a_H1_Data_Check_And_Set",
        "a_H2_Data_Check_And_Set",
        "a_H3_Data_Check_And_Set",
        "a_Board_Base_Check_And_Set",
        "a_H1_Data_Check_And_Set_ui",
        "a_H2_Data_Check_And_Set_ui",
        "a_H3_Data_Check_And_Set_ui",
        "a_Board_Base_Check_And_Set_ui",
        "a_Uart_Write_H1_Setting",
        "a_Uart_Write_H2_Setting",
        "a_Uart_Write_H3_Setting",
        "a_Uart_Read_H1_Setting",
        "a_Uart_Read_H2_Setting",
        "a_Uart_Read_H3_Setting",
        "a_Uart_Write_Heat_Tcr_table",
        "a_Uart_Read_Heat_Tcr_table",
        "a_Uart_Write_H1_Profile",
        "a_Uart_Write_H2_Profile",
        "a_Uart_Write_H3_Profile",
        "a_Uart_Read_H1_Profile",
        "a_Uart_Read_H2_Profile",
        "a_Uart_Read_H3_Profile",
        "a_Uart_Write_Board_baisic_info",
        "a_Uart_Read_Board_baisic_info",
        "a_Read_board_basic_info_Get",
        "a_Write_Real_Time_Log_Set",
        "a_TableWidget_Data_Tran_Array",
        "Heat_Data_Set*",
        "Heat_Data",
        "a_Array_Data_Tran_TableWidget",
        "on_BAT_Display_Table_Set_Profile1_clicked",
        "on_BAT_Display_Table_Set_Profile2_clicked",
        "on_BAT_Display_Table_Set_Profile3_clicked",
        "on_BAT_Display_Table_Init_Tcr_clicked",
        "on_BAT_Write_TCR_List_Flash_clicked",
        "on_BAT_Read_TCR_List_Flash_clicked",
        "on_BAT_Clean_Flash_Data_clicked",
        "on_BAT_Start_Heat_clicked",
        "on_BAT_Stop_Heat_clicked",
        "on_BAT_Myself_Cali_Start_clicked",
        "on_BAT_Myself_Cali_Stop_clicked",
        "on_Log_display_recover_clicked",
        "on_Log_display_change_clicked",
        "a_log_flag_set_ui",
        "on_BAT_Clean_Table_Set_Profile1_clicked",
        "on_BAT_Clean_Table_Set_Profile2_clicked",
        "on_BAT_Clean_Table_Set_Profile3_clicked",
        "on_Bat_export_flash_data_clicked",
        "onSerialDataReady",
        "onSimulationTimer"
    };

    QtMocHelpers::UintData qt_methods {
        // Signal 'rawDataReady'
        QtMocHelpers::SignalData<void(float, float, float)>(1, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Float, 3 }, { QMetaType::Float, 4 }, { QMetaType::Float, 5 },
        }}),
        // Signal 'filteredDataReady'
        QtMocHelpers::SignalData<void(float, float, float)>(6, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Float, 3 }, { QMetaType::Float, 4 }, { QMetaType::Float, 5 },
        }}),
        // Signal 'windowedDataReady'
        QtMocHelpers::SignalData<void(const QVector<double> &, const QVector<double> &)>(7, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 8, 9 }, { 0x80000000 | 8, 10 },
        }}),
        // Slot 'on_BAT_Clean_Table_Init_Tcr_clicked'
        QtMocHelpers::SlotData<void()>(11, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Save_Table_Init_Tcr_clicked'
        QtMocHelpers::SlotData<void()>(12, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Load_Table_Init_Tcr_clicked'
        QtMocHelpers::SlotData<void()>(13, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Save_Table_Set_Profile1_clicked'
        QtMocHelpers::SlotData<void()>(14, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Load_Table_Set_Profile1_clicked'
        QtMocHelpers::SlotData<void()>(15, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Save_Table_Set_Profile2_clicked'
        QtMocHelpers::SlotData<void()>(16, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Load_Table_Set_Profile2_clicked'
        QtMocHelpers::SlotData<void()>(17, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Save_Table_Set_Profile3_clicked'
        QtMocHelpers::SlotData<void()>(18, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Load_Table_Set_Profile3_clicked'
        QtMocHelpers::SlotData<void()>(19, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_timer_timeout'
        QtMocHelpers::SlotData<void()>(20, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'C_serialPortReadYRead_slot'
        QtMocHelpers::SlotData<void()>(21, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Key_Serial_Control_clicked'
        QtMocHelpers::SlotData<void()>(22, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Export_Flash_Data_clicked'
        QtMocHelpers::SlotData<void()>(23, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_clean_Flash'
        QtMocHelpers::SlotData<void()>(24, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Clear_Screen_clicked'
        QtMocHelpers::SlotData<void()>(25, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Pause_clicked'
        QtMocHelpers::SlotData<void()>(26, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_BAT_SerialData_display_Set'
        QtMocHelpers::SlotData<void(QString, uint8_t)>(27, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 28 }, { 0x80000000 | 29, 30 },
        }}),
        // Slot 'QCustomplot_Display_Log_Proc'
        QtMocHelpers::SlotData<void(QCustomPlot *, uint32_t)>(31, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 32, 33 }, { 0x80000000 | 34, 35 },
        }}),
        // Slot 'Board_Save_Log_File_In_Pc'
        QtMocHelpers::SlotData<void(const QString &, uint32_t)>(36, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QString, 37 }, { 0x80000000 | 34, 35 },
        }}),
        // Slot 'QCustomplot_Display_Data_Proc'
        QtMocHelpers::SlotData<void(QTableWidget *, QCustomPlot *, uint8_t)>(38, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 39, 40 }, { 0x80000000 | 32, 33 }, { 0x80000000 | 29, 35 },
        }}),
        // Slot 'Rec_Data_Proc'
        QtMocHelpers::SlotData<void(uint8_t *)>(41, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 42, 43 },
        }}),
        // Slot 'on_BAT_Save_TCR_List_Flash_clicked'
        QtMocHelpers::SlotData<void()>(44, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Read_Configration_clicked'
        QtMocHelpers::SlotData<void()>(45, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Push_Configration_clicked'
        QtMocHelpers::SlotData<void()>(46, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Sync_time_to_device'
        QtMocHelpers::SlotData<void()>(47, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_H1_Data_Check_And_Set'
        QtMocHelpers::SlotData<uint8_t()>(48, 2, QMC::AccessPrivate, 0x80000000 | 29),
        // Slot 'a_H2_Data_Check_And_Set'
        QtMocHelpers::SlotData<uint8_t()>(49, 2, QMC::AccessPrivate, 0x80000000 | 29),
        // Slot 'a_H3_Data_Check_And_Set'
        QtMocHelpers::SlotData<uint8_t()>(50, 2, QMC::AccessPrivate, 0x80000000 | 29),
        // Slot 'a_Board_Base_Check_And_Set'
        QtMocHelpers::SlotData<uint8_t()>(51, 2, QMC::AccessPrivate, 0x80000000 | 29),
        // Slot 'a_H1_Data_Check_And_Set_ui'
        QtMocHelpers::SlotData<void()>(52, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_H2_Data_Check_And_Set_ui'
        QtMocHelpers::SlotData<void()>(53, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_H3_Data_Check_And_Set_ui'
        QtMocHelpers::SlotData<void()>(54, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Board_Base_Check_And_Set_ui'
        QtMocHelpers::SlotData<void()>(55, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_H1_Setting'
        QtMocHelpers::SlotData<void()>(56, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_H2_Setting'
        QtMocHelpers::SlotData<void()>(57, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_H3_Setting'
        QtMocHelpers::SlotData<void()>(58, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_H1_Setting'
        QtMocHelpers::SlotData<void()>(59, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_H2_Setting'
        QtMocHelpers::SlotData<void()>(60, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_H3_Setting'
        QtMocHelpers::SlotData<void()>(61, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_Heat_Tcr_table'
        QtMocHelpers::SlotData<void()>(62, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_Heat_Tcr_table'
        QtMocHelpers::SlotData<void()>(63, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_H1_Profile'
        QtMocHelpers::SlotData<void()>(64, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_H2_Profile'
        QtMocHelpers::SlotData<void()>(65, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_H3_Profile'
        QtMocHelpers::SlotData<void()>(66, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_H1_Profile'
        QtMocHelpers::SlotData<void()>(67, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_H2_Profile'
        QtMocHelpers::SlotData<void()>(68, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_H3_Profile'
        QtMocHelpers::SlotData<void()>(69, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Write_Board_baisic_info'
        QtMocHelpers::SlotData<void()>(70, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Uart_Read_Board_baisic_info'
        QtMocHelpers::SlotData<void()>(71, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Read_board_basic_info_Get'
        QtMocHelpers::SlotData<void()>(72, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_Write_Real_Time_Log_Set'
        QtMocHelpers::SlotData<void()>(73, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_TableWidget_Data_Tran_Array'
        QtMocHelpers::SlotData<uint8_t(QTableWidget *, Heat_Data_Set *, uint8_t)>(74, 2, QMC::AccessPrivate, 0x80000000 | 29, {{
            { 0x80000000 | 39, 40 }, { 0x80000000 | 75, 76 }, { 0x80000000 | 29, 35 },
        }}),
        // Slot 'a_Array_Data_Tran_TableWidget'
        QtMocHelpers::SlotData<void(QTableWidget *, Heat_Data_Set *, uint8_t)>(77, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 39, 40 }, { 0x80000000 | 75, 76 }, { 0x80000000 | 29, 35 },
        }}),
        // Slot 'on_BAT_Display_Table_Set_Profile1_clicked'
        QtMocHelpers::SlotData<void()>(78, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Display_Table_Set_Profile2_clicked'
        QtMocHelpers::SlotData<void()>(79, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Display_Table_Set_Profile3_clicked'
        QtMocHelpers::SlotData<void()>(80, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Display_Table_Init_Tcr_clicked'
        QtMocHelpers::SlotData<void()>(81, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Write_TCR_List_Flash_clicked'
        QtMocHelpers::SlotData<void()>(82, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Read_TCR_List_Flash_clicked'
        QtMocHelpers::SlotData<void()>(83, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Clean_Flash_Data_clicked'
        QtMocHelpers::SlotData<void()>(84, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Start_Heat_clicked'
        QtMocHelpers::SlotData<void()>(85, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Stop_Heat_clicked'
        QtMocHelpers::SlotData<void()>(86, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Myself_Cali_Start_clicked'
        QtMocHelpers::SlotData<void()>(87, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Myself_Cali_Stop_clicked'
        QtMocHelpers::SlotData<void()>(88, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Log_display_recover_clicked'
        QtMocHelpers::SlotData<void()>(89, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Log_display_change_clicked'
        QtMocHelpers::SlotData<void()>(90, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'a_log_flag_set_ui'
        QtMocHelpers::SlotData<void()>(91, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Clean_Table_Set_Profile1_clicked'
        QtMocHelpers::SlotData<void()>(92, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Clean_Table_Set_Profile2_clicked'
        QtMocHelpers::SlotData<void()>(93, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_BAT_Clean_Table_Set_Profile3_clicked'
        QtMocHelpers::SlotData<void()>(94, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'on_Bat_export_flash_data_clicked'
        QtMocHelpers::SlotData<void()>(95, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'onSerialDataReady'
        QtMocHelpers::SlotData<void()>(96, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'onSimulationTimer'
        QtMocHelpers::SlotData<void()>(97, 2, QMC::AccessPrivate, QMetaType::Void),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<Widget, qt_meta_tag_ZN6WidgetE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject Widget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN6WidgetE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN6WidgetE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN6WidgetE_t>.metaTypes,
    nullptr
} };

void Widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<Widget *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->rawDataReady((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[3]))); break;
        case 1: _t->filteredDataReady((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[3]))); break;
        case 2: _t->windowedDataReady((*reinterpret_cast<std::add_pointer_t<QList<double>>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<QList<double>>>(_a[2]))); break;
        case 3: _t->on_BAT_Clean_Table_Init_Tcr_clicked(); break;
        case 4: _t->on_BAT_Save_Table_Init_Tcr_clicked(); break;
        case 5: _t->on_BAT_Load_Table_Init_Tcr_clicked(); break;
        case 6: _t->on_BAT_Save_Table_Set_Profile1_clicked(); break;
        case 7: _t->on_BAT_Load_Table_Set_Profile1_clicked(); break;
        case 8: _t->on_BAT_Save_Table_Set_Profile2_clicked(); break;
        case 9: _t->on_BAT_Load_Table_Set_Profile2_clicked(); break;
        case 10: _t->on_BAT_Save_Table_Set_Profile3_clicked(); break;
        case 11: _t->on_BAT_Load_Table_Set_Profile3_clicked(); break;
        case 12: _t->on_timer_timeout(); break;
        case 13: _t->C_serialPortReadYRead_slot(); break;
        case 14: _t->on_BAT_Key_Serial_Control_clicked(); break;
        case 15: _t->on_BAT_Export_Flash_Data_clicked(); break;
        case 16: _t->a_clean_Flash(); break;
        case 17: _t->on_BAT_Clear_Screen_clicked(); break;
        case 18: _t->on_BAT_Pause_clicked(); break;
        case 19: _t->a_BAT_SerialData_display_Set((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[2]))); break;
        case 20: _t->QCustomplot_Display_Log_Proc((*reinterpret_cast<std::add_pointer_t<QCustomPlot*>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<uint32_t>>(_a[2]))); break;
        case 21: _t->Board_Save_Log_File_In_Pc((*reinterpret_cast<std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<uint32_t>>(_a[2]))); break;
        case 22: _t->QCustomplot_Display_Data_Proc((*reinterpret_cast<std::add_pointer_t<QTableWidget*>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<QCustomPlot*>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[3]))); break;
        case 23: _t->Rec_Data_Proc((*reinterpret_cast<std::add_pointer_t<uint8_t*>>(_a[1]))); break;
        case 24: _t->on_BAT_Save_TCR_List_Flash_clicked(); break;
        case 25: _t->on_BAT_Read_Configration_clicked(); break;
        case 26: _t->on_BAT_Push_Configration_clicked(); break;
        case 27: _t->a_Uart_Sync_time_to_device(); break;
        case 28: { uint8_t _r = _t->a_H1_Data_Check_And_Set();
            if (_a[0]) *reinterpret_cast<uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 29: { uint8_t _r = _t->a_H2_Data_Check_And_Set();
            if (_a[0]) *reinterpret_cast<uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 30: { uint8_t _r = _t->a_H3_Data_Check_And_Set();
            if (_a[0]) *reinterpret_cast<uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 31: { uint8_t _r = _t->a_Board_Base_Check_And_Set();
            if (_a[0]) *reinterpret_cast<uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 32: _t->a_H1_Data_Check_And_Set_ui(); break;
        case 33: _t->a_H2_Data_Check_And_Set_ui(); break;
        case 34: _t->a_H3_Data_Check_And_Set_ui(); break;
        case 35: _t->a_Board_Base_Check_And_Set_ui(); break;
        case 36: _t->a_Uart_Write_H1_Setting(); break;
        case 37: _t->a_Uart_Write_H2_Setting(); break;
        case 38: _t->a_Uart_Write_H3_Setting(); break;
        case 39: _t->a_Uart_Read_H1_Setting(); break;
        case 40: _t->a_Uart_Read_H2_Setting(); break;
        case 41: _t->a_Uart_Read_H3_Setting(); break;
        case 42: _t->a_Uart_Write_Heat_Tcr_table(); break;
        case 43: _t->a_Uart_Read_Heat_Tcr_table(); break;
        case 44: _t->a_Uart_Write_H1_Profile(); break;
        case 45: _t->a_Uart_Write_H2_Profile(); break;
        case 46: _t->a_Uart_Write_H3_Profile(); break;
        case 47: _t->a_Uart_Read_H1_Profile(); break;
        case 48: _t->a_Uart_Read_H2_Profile(); break;
        case 49: _t->a_Uart_Read_H3_Profile(); break;
        case 50: _t->a_Uart_Write_Board_baisic_info(); break;
        case 51: _t->a_Uart_Read_Board_baisic_info(); break;
        case 52: _t->a_Read_board_basic_info_Get(); break;
        case 53: _t->a_Write_Real_Time_Log_Set(); break;
        case 54: { uint8_t _r = _t->a_TableWidget_Data_Tran_Array((*reinterpret_cast<std::add_pointer_t<QTableWidget*>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<Heat_Data_Set*>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[3])));
            if (_a[0]) *reinterpret_cast<uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 55: _t->a_Array_Data_Tran_TableWidget((*reinterpret_cast<std::add_pointer_t<QTableWidget*>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<Heat_Data_Set*>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<uint8_t>>(_a[3]))); break;
        case 56: _t->on_BAT_Display_Table_Set_Profile1_clicked(); break;
        case 57: _t->on_BAT_Display_Table_Set_Profile2_clicked(); break;
        case 58: _t->on_BAT_Display_Table_Set_Profile3_clicked(); break;
        case 59: _t->on_BAT_Display_Table_Init_Tcr_clicked(); break;
        case 60: _t->on_BAT_Write_TCR_List_Flash_clicked(); break;
        case 61: _t->on_BAT_Read_TCR_List_Flash_clicked(); break;
        case 62: _t->on_BAT_Clean_Flash_Data_clicked(); break;
        case 63: _t->on_BAT_Start_Heat_clicked(); break;
        case 64: _t->on_BAT_Stop_Heat_clicked(); break;
        case 65: _t->on_BAT_Myself_Cali_Start_clicked(); break;
        case 66: _t->on_BAT_Myself_Cali_Stop_clicked(); break;
        case 67: _t->on_Log_display_recover_clicked(); break;
        case 68: _t->on_Log_display_change_clicked(); break;
        case 69: _t->a_log_flag_set_ui(); break;
        case 70: _t->on_BAT_Clean_Table_Set_Profile1_clicked(); break;
        case 71: _t->on_BAT_Clean_Table_Set_Profile2_clicked(); break;
        case 72: _t->on_BAT_Clean_Table_Set_Profile3_clicked(); break;
        case 73: _t->on_Bat_export_flash_data_clicked(); break;
        case 74: _t->onSerialDataReady(); break;
        case 75: _t->onSimulationTimer(); break;
        default: ;
        }
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QList<double> >(); break;
            }
            break;
        case 20:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QCustomPlot* >(); break;
            }
            break;
        case 22:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QCustomPlot* >(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QTableWidget* >(); break;
            }
            break;
        case 54:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QTableWidget* >(); break;
            }
            break;
        case 55:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QTableWidget* >(); break;
            }
            break;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        if (QtMocHelpers::indexOfMethod<void (Widget::*)(float , float , float )>(_a, &Widget::rawDataReady, 0))
            return;
        if (QtMocHelpers::indexOfMethod<void (Widget::*)(float , float , float )>(_a, &Widget::filteredDataReady, 1))
            return;
        if (QtMocHelpers::indexOfMethod<void (Widget::*)(const QVector<double> & , const QVector<double> & )>(_a, &Widget::windowedDataReady, 2))
            return;
    }
}

const QMetaObject *Widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Widget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN6WidgetE_t>.strings))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 76)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 76;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 76)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 76;
    }
    return _id;
}

// SIGNAL 0
void Widget::rawDataReady(float _t1, float _t2, float _t3)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 0, nullptr, _t1, _t2, _t3);
}

// SIGNAL 1
void Widget::filteredDataReady(float _t1, float _t2, float _t3)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 1, nullptr, _t1, _t2, _t3);
}

// SIGNAL 2
void Widget::windowedDataReady(const QVector<double> & _t1, const QVector<double> & _t2)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 2, nullptr, _t1, _t2);
}
QT_WARNING_POP
