#include "LoginPage.h"

#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

LoginPage::LoginPage(QWidget* parent)
    : QWidget(parent)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(40, 40, 40, 40);
    root->addStretch();

    auto* card = new QFrame(this);
    card->setFrameShape(QFrame::StyledPanel);
    card->setMinimumWidth(420);
    card->setMaximumWidth(520);

    auto* cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(36, 30, 36, 30);
    cardLayout->setSpacing(18);

    auto* title = new QLabel("智能在轨卫星跟踪与位姿测量系统", card);
    QFont titleFont = title->font();
    titleFont.setPointSize(18);
    titleFont.setBold(true);
    title->setFont(titleFont);
    title->setAlignment(Qt::AlignCenter);

    auto* subTitle = new QLabel("Ubuntu Qt 上位机客户端", card);
    subTitle->setAlignment(Qt::AlignCenter);

    m_usernameEdit = new QLineEdit(card);
    m_usernameEdit->setPlaceholderText("请输入用户名");
    m_usernameEdit->setText("admin");

    m_passwordEdit = new QLineEdit(card);
    m_passwordEdit->setPlaceholderText("请输入密码");
    m_passwordEdit->setEchoMode(QLineEdit::Password);
    m_passwordEdit->setText("123456");

    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    form->addRow("用户名：", m_usernameEdit);
    form->addRow("密码：", m_passwordEdit);

    m_hintLabel = new QLabel("开发阶段默认账号：admin，密码：123456", card);
    m_hintLabel->setAlignment(Qt::AlignCenter);

    m_loginButton = new QPushButton("登录", card);
    m_loginButton->setMinimumHeight(36);

    cardLayout->addWidget(title);
    cardLayout->addWidget(subTitle);
    cardLayout->addSpacing(8);
    cardLayout->addLayout(form);
    cardLayout->addWidget(m_hintLabel);
    cardLayout->addWidget(m_loginButton);

    auto* centerLine = new QHBoxLayout;
    centerLine->addStretch();
    centerLine->addWidget(card);
    centerLine->addStretch();

    root->addLayout(centerLine);
    root->addStretch();

    connect(m_loginButton, &QPushButton::clicked, this, &LoginPage::onLoginClicked);
    connect(m_passwordEdit, &QLineEdit::returnPressed, this, &LoginPage::onLoginClicked);
}

void LoginPage::onLoginClicked()
{
    const QString username = m_usernameEdit->text().trimmed();
    const QString password = m_passwordEdit->text();

    if (username == "admin" && password == "123456") {
        emit loginSucceeded();
        return;
    }

    QMessageBox::warning(this, "登录失败", "用户名或密码错误，请重新输入。\n开发阶段默认账号：admin，密码：123456");
}
