#include "gui/ErrorLog.h"

#include <QAbstractItemView>
#include <QColor>
#include <QElapsedTimer>
#include <QList>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QString>
#include <QWidget>
#include <filesystem>

#include "utils/printutils.h"

ErrorLog::ErrorLog(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  QList<QString> labels = QList<QString>()
                          << QString("Group") << QString("File") << QString("Line") << QString("Info");

  const int numColumns = labels.count();
  this->errorLogModel = new QStandardItemModel(0, numColumns, logTable);

  errorLogModel->setHorizontalHeaderLabels(labels);
  logTable->verticalHeader()->hide();
  logTable->setModel(errorLogModel);
  logTable->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
  logTable->setColumnWidth(errorLog_column::group, 80);
  logTable->setColumnWidth(errorLog_column::file, 200);
  logTable->setColumnWidth(errorLog_column::lineNo, 80);
  logTable->addAction(actionRowSelected);
  // last column will stretch itself

  rowResizeTimer.setSingleShot(true);
  connect(&rowResizeTimer, &QTimer::timeout, this, &ErrorLog::resizeNextRows);
  connect(logTable->horizontalHeader(), &QHeaderView::sectionResized, this, &ErrorLog::scheduleRowResize);
}

void ErrorLog::toErrorLog(const Message& logMsg)
{
  lastMessages.push_back(logMsg);
  if (errorLogComboBox->currentIndex() != 0 &&
      errorLogComboBox->currentText() != QString::fromStdString(getGroupName(logMsg.group))) return;

  appendRow(logMsg);
  logTable->resizeRowToContents(errorLogModel->rowCount() - 1);
}

void ErrorLog::appendRow(const Message& logMsg)
{
  auto *groupName = new QStandardItem(QString::fromStdString(getGroupName(logMsg.group)));
  groupName->setEditable(false);

  if (logMsg.group == message_group::Error)
    groupName->setForeground(QColor::fromRgb(255, 0, 0));  // make this item red.
  else if (logMsg.group == message_group::Warning)
    groupName->setForeground(QColor::fromRgb(252, 211, 3));  // make this item yellow

  QStandardItem *fileName;
  QStandardItem *lineNo;
  if (!logMsg.loc.isNone()) {
    const auto& filePath = logMsg.loc.filePath();
    if (is_regular_file(filePath)) {
      const auto path = QString::fromStdString(filePath.generic_string());
      fileName = new QStandardItem(QString::fromStdString(filePath.filename().generic_string()));
      fileName->setToolTip(path);
      fileName->setData(path, Qt::UserRole);
    } else {
      fileName = new QStandardItem(QString());
    }
    lineNo = new QStandardItem(QString::number(logMsg.loc.firstLine()));
  } else {
    fileName = new QStandardItem(QString());
    lineNo = new QStandardItem(QString());
  }
  fileName->setEditable(false);
  lineNo->setEditable(false);
  lineNo->setTextAlignment(Qt::AlignVCenter | Qt::AlignRight);
  auto *msg = new QStandardItem(QString::fromStdString(logMsg.msg));
  msg->setEditable(false);
  errorLogModel->appendRow({groupName, fileName, lineNo, msg});

  if (!logTable->selectionModel()->hasSelection()) {
    logTable->selectRow(0);
  }
}

void ErrorLog::scheduleRowResize()
{
  nextRowToResize = 0;
  if (errorLogModel->rowCount() > 0) rowResizeTimer.start(50);
}

void ErrorLog::resizeNextRows()
{
  // A column resize can invalidate every row's height. Reflow in short slices
  // so the view can still handle input while a large log is being resized.
  QElapsedTimer elapsed;
  elapsed.start();
  while (nextRowToResize < errorLogModel->rowCount()) {
    logTable->resizeRowToContents(nextRowToResize++);
    if (rowResizeTimer.isActive()) return;  // Column widths changed during reflow.
    if (elapsed.hasExpired(8) && nextRowToResize < errorLogModel->rowCount()) {
      rowResizeTimer.start(1);
      return;
    }
  }
}

void ErrorLog::clearModel()
{
  clearRows();
  lastMessages.clear();
}

void ErrorLog::clearRows()
{
  rowResizeTimer.stop();
  errorLogModel->removeRows(0, errorLogModel->rowCount());
}

void ErrorLog::on_errorLogComboBox_currentTextChanged(const QString& group)
{
  clearRows();
  const bool showAll = errorLogComboBox->currentIndex() == 0;
  for (const auto& lastMessage : lastMessages) {
    if (showAll || group == QString::fromStdString(getGroupName(lastMessage.group))) {
      appendRow(lastMessage);
    }
  }
  scheduleRowResize();
}

void ErrorLog::on_logTable_doubleClicked(const QModelIndex& index)
{
  onIndexSelected(index);
}

void ErrorLog::on_actionRowSelected_triggered(bool)
{
  const auto indexes = logTable->selectionModel()->selectedRows(0);
  if (indexes.size() == 1) {
    onIndexSelected(indexes.first());
  }
}

void ErrorLog::onIndexSelected(const QModelIndex& index)
{
  if (index.isValid()) {
    const int r = index.row();
    const int line = logTable->model()->index(r, errorLog_column::lineNo).data().toInt();
    const auto path = logTable->model()->index(r, errorLog_column::file).data(Qt::UserRole).toString();
    emit openFile(path, line - 1);
  }
}
