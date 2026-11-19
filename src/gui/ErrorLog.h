#pragma once

#include <QModelIndex>
#include <QStandardItemModel>
#include <QTimer>
#include <QWidget>
#include <list>

#include "gui/qtgettext.h"
#include "ui_ErrorLog.h"
#include "utils/printutils.h"

enum errorLog_column { group = 0, file, lineNo, message };

class ErrorLog : public QWidget, public Ui::errorLogWidget
{
  Q_OBJECT

public:
  ErrorLog(QWidget *parent = nullptr);
  ErrorLog(const ErrorLog& source) = delete;
  ErrorLog(ErrorLog&& source) = delete;
  ErrorLog& operator=(const ErrorLog& source) = delete;
  ErrorLog& operator=(ErrorLog&& source) = delete;
  ~ErrorLog() override = default;
  void toErrorLog(const Message& logMsg);
  void clearModel();

private:
  void onIndexSelected(const QModelIndex& index);
  void appendRow(const Message& logMsg);
  void scheduleRowResize();
  void resizeNextRows();
  void clearRows();
  QStandardItemModel *errorLogModel;
  std::list<Message> lastMessages;
  QTimer rowResizeTimer;
  int nextRowToResize{0};

signals:
  void openFile(const QString, int);

private slots:
  void on_logTable_doubleClicked(const QModelIndex& index);
  void on_errorLogComboBox_currentTextChanged(const QString& arg1);
  void on_actionRowSelected_triggered(bool);
};
