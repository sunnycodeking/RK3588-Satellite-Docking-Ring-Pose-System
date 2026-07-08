#include "AppShell.h"

#include "common/Protocol.h"
#include "network/TcpClientManager.h"
#include "pages/ConnectPage.h"
#include "pages/DashboardPage.h"
#include "pages/LoginPage.h"
#include "video/VideoReceiver.h"

#include <QStackedWidget>
#include <QStatusBar>
#include <QTimer>

AppShell::AppShell(QWidget* parent)
    : QMainWindow(parent)
    , m_stack(new QStackedWidget(this))
    , m_tcpClient(new TcpClientManager(this))
    , m_videoReceiver(new VideoReceiver(this))
{
    setWindowTitle("智能在轨卫星故障巡检系统 - Qt 客户端");
    setCentralWidget(m_stack);
    statusBar()->showMessage("客户端已启动");

    m_loginPage = new LoginPage(this);
    m_connectPage = new ConnectPage(this);
    m_dashboardPage = new DashboardPage(m_tcpClient, m_videoReceiver, this);

    m_stack->addWidget(m_loginPage);
    m_stack->addWidget(m_connectPage);
    m_stack->addWidget(m_dashboardPage);

    connect(m_loginPage, &LoginPage::loginSucceeded,
            this, &AppShell::showConnectPage);
    connect(m_connectPage, &ConnectPage::backToLoginRequested,
            this, &AppShell::showLoginPage);
    connect(m_connectPage, &ConnectPage::connectionRequested,
            this, &AppShell::onConnectionRequested);

    connect(m_dashboardPage, &DashboardPage::disconnectRequested,
            this, &AppShell::onDisconnectRequested);
    connect(m_dashboardPage, &DashboardPage::logoutRequested,
            this, &AppShell::onLogoutRequested);

    connect(m_tcpClient, &TcpClientManager::connected,
            this, &AppShell::onTcpConnected);
    connect(m_tcpClient, &TcpClientManager::disconnected,
            this, &AppShell::onTcpDisconnected);
    connect(m_tcpClient, &TcpClientManager::errorMessage,
            this, &AppShell::onTcpError);

    connect(m_tcpClient, &TcpClientManager::logMessage,
            m_dashboardPage, &DashboardPage::appendLog);
    connect(m_tcpClient, &TcpClientManager::errorMessage,
            m_dashboardPage, &DashboardPage::appendError);
    connect(m_tcpClient, &TcpClientManager::messageReceived,
            m_dashboardPage, &DashboardPage::onServerMessage);
    connect(m_tcpClient, &TcpClientManager::connected,
            m_dashboardPage, &DashboardPage::onTcpConnected);
    connect(m_tcpClient, &TcpClientManager::disconnected,
            m_dashboardPage, &DashboardPage::onTcpDisconnected);

    connect(m_videoReceiver, &VideoReceiver::logMessage,
            m_dashboardPage, &DashboardPage::appendLog);
    connect(m_videoReceiver, &VideoReceiver::errorMessage,
            m_dashboardPage, &DashboardPage::appendError);
    connect(m_videoReceiver, &VideoReceiver::started,
            m_dashboardPage, &DashboardPage::onVideoStarted);
    connect(m_videoReceiver, &VideoReceiver::stopped,
            m_dashboardPage, &DashboardPage::onVideoStopped);

    showLoginPage();
}

AppShell::~AppShell()
{
    if (m_videoReceiver) {
        m_videoReceiver->stop();
    }
    if (m_tcpClient) {
        m_tcpClient->disconnectFromServer();
    }
}

void AppShell::showLoginPage()
{
    m_stack->setCurrentWidget(m_loginPage);
    statusBar()->showMessage("请登录");
}

void AppShell::showConnectPage()
{
    m_stack->setCurrentWidget(m_connectPage);
    statusBar()->showMessage("请连接 RK3588 服务端");
}

void AppShell::showDashboardPage()
{
    m_dashboardPage->setVideoPort(m_videoPort);
    m_dashboardPage->setConnectionInfo(m_tcpClient->host(), m_tcpClient->port());
    m_stack->setCurrentWidget(m_dashboardPage);
    statusBar()->showMessage("已进入主功能界面");
}

void AppShell::onConnectionRequested(const QString& ip, quint16 port, int videoPort)
{
    m_videoPort = videoPort;
    m_dashboardPage->setVideoPort(videoPort);
    m_connectPage->setBusy(true);
    m_tcpClient->connectToServer(ip, port);
}

void AppShell::onTcpConnected()
{
    m_connectPage->setBusy(false);
    m_connectPage->setConnected(true);

    m_tcpClient->sendJson(Protocol::hello(m_videoPort));

    QTimer::singleShot(250, this, &AppShell::showDashboardPage);
}

void AppShell::onTcpDisconnected()
{
    m_connectPage->setBusy(false);
    m_connectPage->setConnected(false);
    statusBar()->showMessage("服务器连接已断开");
}

void AppShell::onTcpError(const QString& message)
{
    m_connectPage->setBusy(false);
    m_connectPage->setStatusText(message);
    statusBar()->showMessage(message);
}

void AppShell::onDisconnectRequested()
{
    if (m_videoReceiver->isRunning()) {
        m_videoReceiver->stop();
    }
    m_tcpClient->disconnectFromServer();
    showConnectPage();
}

void AppShell::onLogoutRequested()
{
    if (m_videoReceiver->isRunning()) {
        m_videoReceiver->stop();
    }
    m_tcpClient->disconnectFromServer();
    showLoginPage();
}
