#include <QDebug>
#include <QTimer>
#include <QSoundEffect>

#include "selfdrive/ui/sound.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/offroad/networking.h"

const QString SOUNDS_DIR = "/data/openpilot/selfdrive/assets/sounds/";

Sound::Sound(QWidget *parent) : QStackedWidget(parent) {

  /*
    TODO
      * volume slider
      * show ip addr
      * sound selector
  */

  // main screen
  main = new QWidget;
  {
    QVBoxLayout *layout = new QVBoxLayout(main);
    layout->setContentsMargins(100, 100, 100, 100);

    QLabel *title = new QLabel("Sound tester");
    title->setStyleSheet("font-size: 80px; font-weight: bold;");
    layout->addWidget(title);

    /*
    QLabel *desc = new QLabel("Connect to wifi.");
    desc->setWordWrap(true);
    desc->setStyleSheet("font-size: 65px;");
    layout->addWidget(desc);
    */

    QWidget *w = new QWidget();
    //w->setStyleSheet("background-color: grey;");
    vlayout = new QVBoxLayout(w);
    layout->addWidget(new ScrollView(w), 1);

    QSlider *slider = new QSlider(Qt::Horizontal, this);
    layout->addWidget(slider);

    QHBoxLayout *hlayout = new QHBoxLayout;
    hlayout->setSpacing(30);
    layout->addLayout(hlayout);

    QPushButton *connect = new QPushButton("Connect to WiFi");
    connect->setObjectName("navBtn");
    QObject::connect(connect, &QPushButton::clicked, [=]() {
      setCurrentWidget(wifi);
    });
    hlayout->addWidget(connect);

    QPushButton *install = new QPushButton("Install");
    install->setObjectName("navBtn");
    install->setStyleSheet("background-color: #465BEA;");
    hlayout->addWidget(install);
  }

  // wifi connection screen
  wifi = new QWidget;
  {
    QVBoxLayout *layout = new QVBoxLayout(wifi);
    layout->setContentsMargins(100, 100, 100, 100);

    Networking *networking = new Networking(this, false);
    networking->setStyleSheet("Networking { background-color: #292929; border-radius: 13px; }");
    layout->addWidget(networking, 1);

    QPushButton *back = new QPushButton("Back");
    back->setObjectName("navBtn");
    back->setStyleSheet("padding-left: 60px; padding-right: 60px;");
    QObject::connect(back, &QPushButton::clicked, [=]() {
      setCurrentWidget(main);
    });
    layout->addWidget(back, 0, Qt::AlignLeft);
  }

  addWidget(main);
  addWidget(wifi);

  QTimer *t = new QTimer(this);
  QObject::connect(t, &QTimer::timeout, this, &Sound::updateSounds);
  t->start(1000);

  setStyleSheet(R"(
    * {
      color: white;
      font-family: Inter;
    }
    Sound {
      background-color: black;
    }
    QPushButton#navBtn {
      height: 160;
      font-size: 55px;
      font-weight: 400;
      border-radius: 10px;
      background-color: #333333;
    }
  )");
}

void Sound::updateSounds() {
  clearLayout(vlayout);
  auto files = QDir(SOUNDS_DIR).entryList(QDir::Files);
  for (auto &f : files) {
    ButtonControl *b = new ButtonControl(f, "PLAY");
    QObject::connect(b, &ButtonControl::clicked, [=]() {
      e.setSource(QUrl::fromLocalFile(SOUNDS_DIR + f));
      e.setVolume(0.9);
      e.play();
    });
    vlayout->addWidget(b);
  }
  qDebug() << files;
}

int main(int argc, char *argv[]) {
  initApp();
  QApplication a(argc, argv);
  Sound sound;
  setMainWindow(&sound);
  return a.exec();
}
