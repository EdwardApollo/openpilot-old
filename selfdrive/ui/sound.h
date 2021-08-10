#pragma once

#include <QtWidgets>
#include <QSoundEffect>

class Sound : public QStackedWidget {
  Q_OBJECT

public:
  explicit Sound(QWidget *parent = 0);

private slots:
  void updateSounds();

private:
  QSoundEffect e;
  QVBoxLayout *vlayout;
  QWidget *main, *wifi;
};
