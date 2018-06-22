#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    nccWidget=new nccMatchWidget;
}

MainWindow::~MainWindow()
{
    nccWidget->close();
    delete nccWidget;
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    nccWidget->show();
}
