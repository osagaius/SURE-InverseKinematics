/****************************************************************************
** Meta object code from reading C++ file 'myplugin.h'
**
** Created: Tue Jul 29 16:25:11 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/myplugin.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'myplugin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MyPlugin[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      10,    9,    9,    9, 0x08,
      21,    9,    9,    9, 0x08,
      35,    9,    9,    9, 0x08,
      53,    9,    9,    9, 0x08,
      68,    9,    9,    9, 0x08,
      82,    9,    9,    9, 0x08,
     129,  104,    9,    9, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MyPlugin[] = {
    "MyPlugin\0\0addValue()\0reduceValue()\0"
    "continuousValue()\0seeRobotInfo()\0"
    "moveLeftArm()\0setKrangStartConfig()\0"
    "xdot,qdot_nullspace,qdot\0"
    "refJSVelocity(Eigen::VectorXd,Eigen::VectorXd,Eigen::VectorXd&)\0"
};

void MyPlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MyPlugin *_t = static_cast<MyPlugin *>(_o);
        switch (_id) {
        case 0: _t->addValue(); break;
        case 1: _t->reduceValue(); break;
        case 2: _t->continuousValue(); break;
        case 3: _t->seeRobotInfo(); break;
        case 4: _t->moveLeftArm(); break;
        case 5: _t->setKrangStartConfig(); break;
        case 6: _t->refJSVelocity((*reinterpret_cast< const Eigen::VectorXd(*)>(_a[1])),(*reinterpret_cast< const Eigen::VectorXd(*)>(_a[2])),(*reinterpret_cast< Eigen::VectorXd(*)>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MyPlugin::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MyPlugin::staticMetaObject = {
    { &GripTab::staticMetaObject, qt_meta_stringdata_MyPlugin,
      qt_meta_data_MyPlugin, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MyPlugin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MyPlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MyPlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MyPlugin))
        return static_cast<void*>(const_cast< MyPlugin*>(this));
    if (!strcmp(_clname, "com.gatech.Grip2.GripTab/1.0"))
        return static_cast< GripTab*>(const_cast< MyPlugin*>(this));
    return GripTab::qt_metacast(_clname);
}

int MyPlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GripTab::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
