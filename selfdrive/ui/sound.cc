#include <QDebug>
#include <QTimer>
#include <QSoundEffect>

#include "selfdrive/ui/sound.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

const QString SOUNDS_DIR = "/data/openpilot/selfdrive/assets/sounds/";

Sound::Sound(QWidget *parent) : QStackedWidget(parent) {

  // main screen
  main = new QWidget;
  {
    QVBoxLayout *layout = new QVBoxLayout(main);
    layout->setContentsMargins(100, 100, 100, 100);

    title = new QLabel("Sound tester");
    title->setStyleSheet("font-size: 80px; font-weight: bold;");
    layout->addWidget(title);

    QWidget *w = new QWidget();
    vlayout = new QVBoxLayout(w);
    layout->addWidget(new ScrollView(w), 1);

    QSlider *slider = new QSlider(Qt::Horizontal, this);
    slider->setValue(100);
    QObject::connect(slider, &QSlider::valueChanged, [=](int value) {
      e.setVolume((float)value / 100);
    });
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

    QPushButton *clear = new QPushButton("Clear Files");
    QObject::connect(clear, &QPushButton::clicked, [=]() {
      std::system(std::string("rm -rf " + SOUNDS_DIR.toStdString() + "*").c_str());
    });
    clear->setObjectName("navBtn");
    clear->setStyleSheet("background-color: #465BEA;");
    hlayout->addWidget(clear);
  }

  // wifi connection screen
  wifi = new QWidget;
  {
    QVBoxLayout *layout = new QVBoxLayout(wifi);
    layout->setContentsMargins(100, 100, 100, 100);

    n = new Networking(this, false);
    n->setStyleSheet("Networking { background-color: #292929; border-radius: 13px; }");
    layout->addWidget(n, 1);

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
    QSlider { height: 100px; }
    QSlider::sub-page:horizontal {
      background-color: #465BEA;
      border: 0px;
      border-radius: 5px;
    }
    QSlider::add-page:horizontal {
      background-color: #333333;
      border: 0px;
      border-radius: 5px;
    }
    QSlider::groove:horizontal {
      border: 0px;
      height: 10px;
      border-radius: 5px;
      background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4);
    }
    QSlider::handle:horizontal {
      background-color: #4D4D4D;
      border: 2px solid #4D4D4D;
      width: 38px;
      height: 38px;
      line-height: 20px;
      margin-top: -10px;
      margin-bottom: -10px;
      border-radius: 15px;
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
    b->setStyleSheet("border: none;");
    QObject::connect(b, &ButtonControl::clicked, [=]() {
      e.setSource(QUrl::fromLocalFile(SOUNDS_DIR + f));
      e.play();
    });
    vlayout->addWidget(b);
  }

  // update ip
  if (n->wifi->ipv4_address.size()) {
    title->setText(QString("Sound tester (%1)").arg(n->wifi->ipv4_address));
  } else {
    title->setText("Sound tester");
  }
}

int main(int argc, char *argv[]) {
  initApp();
  QApplication a(argc, argv);
  Sound sound;
  setMainWindow(&sound);
  return a.exec();
}
