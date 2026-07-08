#include "DebugLogWindow.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextEdit>
#include <QVBoxLayout>

DebugLogWindow::DebugLogWindow(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("RK3588 板端调试输出");
    resize(900, 520);
    setAttribute(Qt::WA_DeleteOnClose, false);

    auto* root = new QVBoxLayout(this);
    m_textEdit = new QTextEdit(this);
    m_textEdit->setReadOnly(true);
    m_textEdit->setLineWrapMode(QTextEdit::NoWrap);

    m_clearButton = new QPushButton("清空", this);
    m_closeButton = new QPushButton("关闭", this);

    auto* buttons = new QHBoxLayout;
    buttons->addStretch();
    buttons->addWidget(m_clearButton);
    buttons->addWidget(m_closeButton);

    root->addWidget(m_textEdit);
    root->addLayout(buttons);

    connect(m_clearButton, &QPushButton::clicked, this, &DebugLogWindow::clearLog);
    connect(m_closeButton, &QPushButton::clicked, this, &DebugLogWindow::hide);
}

void DebugLogWindow::appendLine(const QString& line, bool isError)
{
    QTextCursor cursor = m_textEdit->textCursor();
    cursor.movePosition(QTextCursor::End);

    QTextCharFormat fmt;
    fmt.setForeground(isError ? Qt::darkRed : Qt::black);

    const QString time = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    cursor.insertText(QString("[%1] %2").arg(time, line), fmt);
    cursor.insertBlock();

    m_textEdit->setTextCursor(cursor);
    m_textEdit->ensureCursorVisible();
}

void DebugLogWindow::clearLog()
{
    m_textEdit->clear();
}
