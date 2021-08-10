#pragma once

#include <QtWidgets>
#include <QSoundEffect>
#include "selfdrive/ui/qt/offroad/networking.h"

class Sound : public QStackedWidget {
  Q_OBJECT

public:
  explicit Sound(QWidget *parent = 0);

private slots:
  void updateSounds();

private:
  Networking *n;
  QSoundEffect e;
  QLabel *title;
  QVBoxLayout *vlayout;
  QWidget *main, *wifi;
};
