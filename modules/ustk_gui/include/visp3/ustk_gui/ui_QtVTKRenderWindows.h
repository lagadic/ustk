/********************************************************************************
** Form generated from reading UI file 'QtVTKRenderWindows.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QTVTKRENDERWINDOWS_H
#define UI_QTVTKRENDERWINDOWS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_QtVTKRenderWindows
{
public:
    QAction *actionOpenFile;
    QAction *actionExit;
    QAction *actionPrint;
    QAction *actionHelp;
    QAction *actionSave;
    QWidget *centralwidget;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_2;
    QVTKWidget *view2;
    QVTKWidget *view4;
    QVTKWidget *view3;
    QVTKWidget *view1;
    QPushButton *resetButton;
    QPushButton *AddDistance1Button;

    void setupUi(QMainWindow *QtVTKRenderWindows)
    {
        if (QtVTKRenderWindows->objectName().isEmpty())
            QtVTKRenderWindows->setObjectName(QString::fromUtf8("QtVTKRenderWindows"));
        QtVTKRenderWindows->resize(844, 601);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/Icons/help.png"), QSize(), QIcon::Normal, QIcon::Off);
        QtVTKRenderWindows->setWindowIcon(icon);
        QtVTKRenderWindows->setIconSize(QSize(22, 22));
        actionOpenFile = new QAction(QtVTKRenderWindows);
        actionOpenFile->setObjectName(QString::fromUtf8("actionOpenFile"));
        actionOpenFile->setEnabled(true);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/Icons/fileopen.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpenFile->setIcon(icon1);
        actionExit = new QAction(QtVTKRenderWindows);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8("."), QSize(), QIcon::Normal, QIcon::Off);
        actionExit->setIcon(icon2);
        actionPrint = new QAction(QtVTKRenderWindows);
        actionPrint->setObjectName(QString::fromUtf8("actionPrint"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/Icons/print.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionPrint->setIcon(icon3);
        actionHelp = new QAction(QtVTKRenderWindows);
        actionHelp->setObjectName(QString::fromUtf8("actionHelp"));
        actionHelp->setIcon(icon);
        actionSave = new QAction(QtVTKRenderWindows);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/Icons/filesave.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon4);
        centralwidget = new QWidget(QtVTKRenderWindows);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayoutWidget = new QWidget(centralwidget);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 30, 591, 531));
        gridLayout_2 = new QGridLayout(gridLayoutWidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        view2 = new QVTKWidget(gridLayoutWidget);
        view2->setObjectName(QString::fromUtf8("view2"));

        gridLayout_2->addWidget(view2, 1, 0, 1, 1);

        view4 = new QVTKWidget(gridLayoutWidget);
        view4->setObjectName(QString::fromUtf8("view4"));

        gridLayout_2->addWidget(view4, 0, 1, 1, 1);

        view3 = new QVTKWidget(gridLayoutWidget);
        view3->setObjectName(QString::fromUtf8("view3"));

        gridLayout_2->addWidget(view3, 1, 1, 1, 1);

        view1 = new QVTKWidget(gridLayoutWidget);
        view1->setObjectName(QString::fromUtf8("view1"));

        gridLayout_2->addWidget(view1, 0, 0, 1, 1);

        resetButton = new QPushButton(centralwidget);
        resetButton->setObjectName(QString::fromUtf8("resetButton"));
        resetButton->setGeometry(QRect(630, 30, 201, 31));
        AddDistance1Button = new QPushButton(centralwidget);
        AddDistance1Button->setObjectName(QString::fromUtf8("AddDistance1Button"));
        AddDistance1Button->setGeometry(QRect(630, 80, 201, 31));
        QtVTKRenderWindows->setCentralWidget(centralwidget);

        retranslateUi(QtVTKRenderWindows);

        QMetaObject::connectSlotsByName(QtVTKRenderWindows);
    } // setupUi

    void retranslateUi(QMainWindow *QtVTKRenderWindows)
    {
        QtVTKRenderWindows->setWindowTitle(QApplication::translate("QtVTKRenderWindows", "QtVTKRenderWindows"));
        actionOpenFile->setText(QApplication::translate("QtVTKRenderWindows", "Open File..."));
        actionExit->setText(QApplication::translate("QtVTKRenderWindows", "Exit"));
        actionPrint->setText(QApplication::translate("QtVTKRenderWindows", "Print"));
        actionHelp->setText(QApplication::translate("QtVTKRenderWindows", "Help"));
        actionSave->setText(QApplication::translate("QtVTKRenderWindows", "Save"));
        resetButton->setText(QApplication::translate("QtVTKRenderWindows", "Reset"));
        AddDistance1Button->setText(QApplication::translate("QtVTKRenderWindows", "Add Distance On View 1"));
    } // retranslateUi

};

namespace Ui {
    class QtVTKRenderWindows: public Ui_QtVTKRenderWindows {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QTVTKRENDERWINDOWS_H
