#include "TestMainWindow.h"

#include <QFile>
#include <QString>
#include <QStringList>
#include <QTemporaryDir>
#include <QTest>

#include "gui/QGLView.h"
#include "platform/PlatformUtils.h"

void TestMainWindow::checkOpenTabPropagateToWindow()
{
  restoreWindowInitialState();

  QString filename = fixturePath("basic-ux/empty.scad");

  // When we open a new file,
  window->tabManager->open(filename);

  // The window title must also have the name of open file
  QCOMPARE(window->windowTitle(), QFileInfo(filename).fileName());

  filename = fixturePath("basic-ux/empty2.scad");

  // When we open a new file,
  window->tabManager->open(filename);

  // The window title must also have the name of open file
  QCOMPARE(window->windowTitle(), QFileInfo(filename).fileName());
}

void TestMainWindow::checkPreviewFollowsActiveTab()
{
  restoreWindowInitialState();

  QTemporaryDir models;
  QVERIFY(models.isValid());
  const auto smallModel = models.filePath("small.scad");
  const auto largeModel = models.filePath("large.scad");
  QFile small(smallModel), large(largeModel);
  QVERIFY(small.open(QIODevice::WriteOnly));
  QVERIFY(large.open(QIODevice::WriteOnly));
  QCOMPARE(small.write("cube(10);\n"), qint64(10));
  QCOMPARE(large.write("cube(20);\n"), qint64(10));
  small.close();
  large.close();

  const auto displayedWidth = [this]() -> double {
    const auto *renderer = window->qglview->getRenderer();
    return renderer ? renderer->getBoundingBox().sizes().x() : -1.0;
  };

  window->tabManager->open(smallModel);
  QTRY_COMPARE(displayedWidth(), 10.0);
  window->tabManager->open(largeModel);
  QTRY_COMPARE(displayedWidth(), 20.0);
  // Reopening an existing file selects its tab.
  window->tabManager->open(smallModel);
  QTRY_COMPARE(displayedWidth(), 10.0);

  window->designActionAutoReload->setChecked(false);
  window->tabManager->open(largeModel);
  QCOMPARE(displayedWidth(), -1.0);
  window->tabManager->open(smallModel);
  QCOMPARE(displayedWidth(), 10.0);

  window->tabManager->closeCurrentTab();
  window->tabManager->open(largeModel);
  window->tabManager->closeCurrentTab();
  window->designActionAutoReload->setChecked(true);
}

void TestMainWindow::checkSaveToShouldUpdateWindowTitle()
{
  restoreWindowInitialState();

  QString filename = fixturePath("basic-ux/empty.scad");

  // When we open a new file,
  window->tabManager->open(filename);

  window->tabManager->saveAs(window->activeEditor, "test-tmp.scad");

  // The window title must also have the name of open file
  QCOMPARE(window->windowTitle(), "test-tmp.scad");
}
