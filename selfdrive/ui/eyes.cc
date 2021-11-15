#include <QApplication>
#include <QtWidgets>

#include <sys/resource.h>

#include "cereal/messaging/messaging.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

class Eyes : public QWidget {
public:
  Eyes(QWidget *parent = 0) : sm({"carState"}), QWidget(parent) {
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setMargin(0);
    layout->setSpacing(0);

    QLabel *l = new QLabel();
    QPixmap p("eyes.png");
    l->setPixmap(p.scaled(QSize(2160, 1080)));
    layout->addWidget(l);

    QTimer *timer = new QTimer(this);
    QObject::connect(timer, &QTimer::timeout, [=]() {
      sm.update(0);
      // scale volume with speed
      if (sm.updated("carState")) {
        if (sm["carState"].getCarState().getStandstill()) {
          if (l->pixmap()->toImage() != sleeping.toImage()) l->setPixmap(sleeping);
        } else {
          if (l->pixmap()->toImage() != moving.toImage()) l->setPixmap(moving);
        }
        update();
      }
    });
    timer->start();
  };

private:
  SubMaster sm;
  QPixmap sleeping = QPixmap("eyes.png").scaled(2160, 1080);
  QPixmap moving = QPixmap("moving.png").scaled(2160, 1080);
};

int main(int argc, char *argv[]) {
  initApp();
  QApplication a(argc, argv);
  Eyes eyes;
  setMainWindow(&eyes);
  return a.exec();
}
