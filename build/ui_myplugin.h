/********************************************************************************
** Form generated from reading UI file 'myplugin.ui'
**
** Created: Wed Jul 30 13:59:49 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MYPLUGIN_H
#define UI_MYPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MyPluginTab
{
public:
    QWidget *dockWidgetContents;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButtonMoveLeftArm;
    QPushButton *pushButtonAdd;
    QPushButton *pushButtonReduce;
    QPushButton *pushButtonSetStartConfiguration;
    QPushButton *seeRobotInfo;

    void setupUi(QDockWidget *MyPluginTab)
    {
        if (MyPluginTab->objectName().isEmpty())
            MyPluginTab->setObjectName(QString::fromUtf8("MyPluginTab"));
        MyPluginTab->resize(731, 361);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        horizontalLayout = new QHBoxLayout(dockWidgetContents);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        pushButtonMoveLeftArm = new QPushButton(dockWidgetContents);
        pushButtonMoveLeftArm->setObjectName(QString::fromUtf8("pushButtonMoveLeftArm"));

        horizontalLayout->addWidget(pushButtonMoveLeftArm);

        pushButtonAdd = new QPushButton(dockWidgetContents);
        pushButtonAdd->setObjectName(QString::fromUtf8("pushButtonAdd"));

        horizontalLayout->addWidget(pushButtonAdd);

        pushButtonReduce = new QPushButton(dockWidgetContents);
        pushButtonReduce->setObjectName(QString::fromUtf8("pushButtonReduce"));

        horizontalLayout->addWidget(pushButtonReduce);

        pushButtonSetStartConfiguration = new QPushButton(dockWidgetContents);
        pushButtonSetStartConfiguration->setObjectName(QString::fromUtf8("pushButtonSetStartConfiguration"));

        horizontalLayout->addWidget(pushButtonSetStartConfiguration);

        seeRobotInfo = new QPushButton(dockWidgetContents);
        seeRobotInfo->setObjectName(QString::fromUtf8("seeRobotInfo"));

        horizontalLayout->addWidget(seeRobotInfo);

        MyPluginTab->setWidget(dockWidgetContents);

        retranslateUi(MyPluginTab);

        QMetaObject::connectSlotsByName(MyPluginTab);
    } // setupUi

    void retranslateUi(QDockWidget *MyPluginTab)
    {
        MyPluginTab->setWindowTitle(QApplication::translate("MyPluginTab", "Edit and Clear Tab", 0, QApplication::UnicodeUTF8));
        pushButtonMoveLeftArm->setText(QApplication::translate("MyPluginTab", "Move Left Arm", 0, QApplication::UnicodeUTF8));
        pushButtonAdd->setText(QApplication::translate("MyPluginTab", "Add Value", 0, QApplication::UnicodeUTF8));
        pushButtonReduce->setText(QApplication::translate("MyPluginTab", "Reduce value", 0, QApplication::UnicodeUTF8));
        pushButtonSetStartConfiguration->setText(QApplication::translate("MyPluginTab", "Change Configuration", 0, QApplication::UnicodeUTF8));
        seeRobotInfo->setText(QApplication::translate("MyPluginTab", "See Robot Info", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MyPluginTab: public Ui_MyPluginTab {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MYPLUGIN_H
