/********************************************************************************
** Form generated from reading UI file 'visualizer.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VISUALIZER_H
#define UI_VISUALIZER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHeaderView>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QCheckBox *ic_cbox;
    QCheckBox *pd_cbox;
    QCheckBox *od_cbox;
    QCheckBox *darknet_cbox;
    QCheckBox *or_cbox;
    QCheckBox *fd_cbox;
    QCheckBox *fr_cbox;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(211, 247);
        layoutWidget = new QWidget(Form);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 189, 227));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        ic_cbox = new QCheckBox(layoutWidget);
        ic_cbox->setObjectName(QString::fromUtf8("ic_cbox"));

        verticalLayout->addWidget(ic_cbox);

        pd_cbox = new QCheckBox(layoutWidget);
        pd_cbox->setObjectName(QString::fromUtf8("pd_cbox"));

        verticalLayout->addWidget(pd_cbox);

        od_cbox = new QCheckBox(layoutWidget);
        od_cbox->setObjectName(QString::fromUtf8("od_cbox"));

        verticalLayout->addWidget(od_cbox);

        darknet_cbox = new QCheckBox(layoutWidget);
        darknet_cbox->setObjectName(QString::fromUtf8("darknet_cbox"));

        verticalLayout->addWidget(darknet_cbox);

        or_cbox = new QCheckBox(layoutWidget);
        or_cbox->setObjectName(QString::fromUtf8("or_cbox"));

        verticalLayout->addWidget(or_cbox);

        fd_cbox = new QCheckBox(layoutWidget);
        fd_cbox->setObjectName(QString::fromUtf8("fd_cbox"));

        verticalLayout->addWidget(fd_cbox);

        fr_cbox = new QCheckBox(layoutWidget);
        fr_cbox->setObjectName(QString::fromUtf8("fr_cbox"));

        verticalLayout->addWidget(fr_cbox);


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", 0, QApplication::UnicodeUTF8));
        ic_cbox->setText(QApplication::translate("Form", "image_capture", 0, QApplication::UnicodeUTF8));
        pd_cbox->setText(QApplication::translate("Form", "plane_detection", 0, QApplication::UnicodeUTF8));
        od_cbox->setText(QApplication::translate("Form", "object_detection", 0, QApplication::UnicodeUTF8));
        darknet_cbox->setText(QApplication::translate("Form", "darknet", 0, QApplication::UnicodeUTF8));
        or_cbox->setText(QApplication::translate("Form", "object_recognition", 0, QApplication::UnicodeUTF8));
        fd_cbox->setText(QApplication::translate("Form", "face_detection", 0, QApplication::UnicodeUTF8));
        fr_cbox->setText(QApplication::translate("Form", "face_recognition", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VISUALIZER_H
