#pragma once

#include <QDialog>
#include <QString>

class QTextEdit;
class QPushButton;

class DebugLogWindow : public QDialog
{
    Q_OBJECT

public:
    explicit DebugLogWindow(QWidget* parent = nullptr);

public slots:
    void appendLine(const QString& line, bool isError = false);
    void clearLog();

private:
    QTextEdit* m_textEdit = nullptr;
    QPushButton* m_clearButton = nullptr;
    QPushButton* m_closeButton = nullptr;
};
