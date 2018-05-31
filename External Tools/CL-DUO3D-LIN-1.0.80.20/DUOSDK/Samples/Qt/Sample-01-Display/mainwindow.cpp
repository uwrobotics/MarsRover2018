#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), 
      _duo(NULL)
{
    resize(320*2, 240);
    setWindowTitle("DUO Test");
    setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, size(),
                                    qApp->desktop()->availableGeometry()));
    auto main = new QWidget();
        auto hs = new QHBoxLayout();
            hs->addWidget(_img[0] = new ImageOutput());
            hs->addWidget(_img[1] = new ImageOutput());
        hs->setMargin(0);
        hs->setSpacing(0);
    main->setLayout(hs);
    setCentralWidget(main);

    DUOResolutionInfo ri;
    if(EnumerateDUOResolutions(&ri, 1, 320, 240, DUO_BIN_HORIZONTAL2 + DUO_BIN_VERTICAL2, 30))
    {
        Print("[%dx%d], [%f-%f], %f, [%d]", ri.width, ri.height, ri.minFps, ri.maxFps, ri.fps, ri.binning);

        if(OpenDUO(&_duo))
        {
            char buf[256];
            GetDUOSerialNumber(_duo, buf);       Print("Serial Number: %s", buf);
            GetDUOFirmwareVersion(_duo, buf);    Print("Firmware Version: v%s", buf);
            GetDUOFirmwareBuild(_duo, buf);      Print("Firmware Build Time: %s", buf);
            Print("Library Version: v%s", GetDUOLibVersion());
            Print("-------------------------------------------------");

            SetDUOResolutionInfo(_duo, ri);
            uint32_t w, h;
            GetDUOFrameDimension(_duo, &w, &h);  Print("Frame Dimension: [%d, %d]", w, h);

            StartDUO(_duo, [](const PDUOFrame pFrameData, void *pUserData)
            {
                ((MainWindow *)pUserData)->onNewFrame(pFrameData);
            }, this);

            SetDUOLedPWM(_duo, 30);
            SetDUOGain(_duo, 0);
            SetDUOExposure(_duo, 50);
            SetDUOVFlip(_duo, true);
        }
    }
}

MainWindow::~MainWindow()
{
    if(_duo) CloseDUO(_duo);
}

void MainWindow::closeEvent(QCloseEvent *)
{
    if(_duo) StopDUO(_duo);
}

void MainWindow::onNewFrame(const PDUOFrame pDFrame)
{
    Mat left(Size(pDFrame->width, pDFrame->height), CV_8U, pDFrame->leftData);
    Mat right(Size(pDFrame->width, pDFrame->height), CV_8U, pDFrame->rightData);
    cvtColor(left, _leftRGB, COLOR_GRAY2BGR);
    cvtColor(right, _rightRGB, COLOR_GRAY2BGR);
    Q_EMIT _img[0]->setImage(_leftRGB);
    Q_EMIT _img[1]->setImage(_rightRGB);
}
