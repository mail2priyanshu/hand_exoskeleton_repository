/****************************************************************************
** Meta object code from reading C++ file 'data_recv.h'
**
** Created: Wed Feb 19 18:03:10 2014
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "data_recv.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'data_recv.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Data_Recv[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x0a,
      30,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Data_Recv[] = {
    "Data_Recv\0\0Data_ReadHandler()\0"
    "initialization()\0"
};

const QMetaObject Data_Recv::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Data_Recv,
      qt_meta_data_Data_Recv, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Data_Recv::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Data_Recv::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Data_Recv::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Data_Recv))
        return static_cast<void*>(const_cast< Data_Recv*>(this));
    return QObject::qt_metacast(_clname);
}

int Data_Recv::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: Data_ReadHandler(); break;
        case 1: initialization(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
