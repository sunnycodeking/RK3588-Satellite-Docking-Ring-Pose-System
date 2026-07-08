#include "ConnectPage.h"

#include "common/Protocol.h"

#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

ConnectPage::ConnectPage(QWidget* parent)
    : QWidget(parent)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(40, 40, 40, 40);
    root->addStretch();

    auto* card = new QFrame(this);
    card->setFrameShape(QFrame::StyledPanel);
    card->setMinimumWidth(460);
    card->setMaximumWidth(560);

    auto* cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(36, 30, 36, 30);
    cardLayout->setSpacing(16);

    auto* title = new QLabel("连接 RK3588 服务端", card);
    QFont titleFont = title->font();
    titleFont.setPointSize(16);
    titleFont.setBold(true);
    title->setFont(titleFont);
    title->setAlignment(Qt::AlignCenter);

    m_ipEdit = new QLineEdit(card);
    m_ipEdit->setPlaceholderText("例如：192.168.43.192");
    m_ipEdit->setText("192.168.43.192");

    m_portSpin = new QSpinBox(card);
    m_portSpin->setRange(1, 65535);
    m_portSpin->setValue(Protocol::kDefaultControlPort);

    m_videoPortSpin = new QSpinBox(card);
    m_videoPortSpin->setRange(1, 65535);
    m_videoPortSpin->setValue(Protocol::kDefaultVideoPort);

    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    form->addRow("服务器 IP：", m_ipEdit);
    form->addRow("TCP 控制端口：", m_portSpin);
    form->addRow("本机视频接收端口：", m_videoPortSpin);

    m_statusLabel = new QLabel("请输入 RK3588 板子的 IP 地址，然后点击连接。", card);
    m_statusLabel->setWordWrap(true);

    m_connectButton = new QPushButton("连接服务器", card);
    m_connectButton->setMinimumHeight(36);

    m_backButton = new QPushButton("返回登录", card);

    auto* buttonLine = new QHBoxLayout;
    buttonLine->addWidget(m_backButton);
    buttonLine->addWidget(m_connectButton);

    cardLayout->addWidget(title);
    cardLayout->addLayout(form);
    cardLayout->addWidget(m_statusLabel);
    cardLayout->addLayout(buttonLine);

    auto* centerLine = new QHBoxLayout;
    centerLine->addStretch();
    centerLine->addWidget(card);
    centerLine->addStretch();

    root->addLayout(centerLine);
    root->addStretch();

    connect(m_connectButton, &QPushButton::clicked, this, &ConnectPage::onConnectClicked);
    connect(m_backButton, &QPushButton::clicked, this, &ConnectPage::backToLoginRequested);
}

void ConnectPage::setBusy(bool busy)
{
    m_connectButton->setEnabled(!busy);
    m_backButton->setEnabled(!busy);
    m_ipEdit->setEnabled(!busy);
    m_portSpin->setEnabled(!busy);
    m_videoPortSpin->setEnabled(!busy);
    if (busy) {
        m_statusLabel->setText("正在连接，请稍候 ...");
    }
}

void ConnectPage::setConnected(bool connected)
{
    if (connected) {
        m_statusLabel->setText("服务器连接成功，正在进入主界面 ...");
    } else {
        m_statusLabel->setText("服务器未连接。请检查网络、IP、端口和 RK3588 服务端程序。 ");
    }
}

void ConnectPage::setStatusText(const QString& text)
{
    m_statusLabel->setText(text);
}

int ConnectPage::videoPort() const
{
    return m_videoPortSpin->value();
}

void ConnectPage::onConnectClicked()
{
    const QString ip = m_ipEdit->text().trimmed();
    QHostAddress address;
    if (ip.isEmpty() || !address.setAddress(ip)) {
        QMessageBox::warning(this, "IP 地址错误", "请输入合法的 IPv4 地址，例如：192.168.1.20");
        return;
    }

    emit connectionRequested(ip, static_cast<quint16>(m_portSpin->value()), m_videoPortSpin->value());
}
