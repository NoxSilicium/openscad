#pragma once

#include "UXTest.h"

class TestMainWindow : public UXTest
{
  Q_OBJECT;
private slots:
  void checkOpenTabPropagateToWindow();
  void checkPreviewFollowsActiveTab();
  void checkSaveToShouldUpdateWindowTitle();
};
