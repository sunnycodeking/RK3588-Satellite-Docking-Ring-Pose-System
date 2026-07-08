#pragma once

#include <QMainWindow>

class QStackedWidget;
class LoginPage;
class ConnectPage;
class DashboardPage;
class TcpClientManager;
class VideoReceiver;

class AppShell : public QMainWindow
{
    Q_OBJECT

public:
    explicit AppShell(QWidget* parent = nullptr);
    ~AppShell() override;

private slots:
    void showLoginPage();
    void showConnectPage();
    void showDashboardPage();

    void onConnectionRequested(const QString& ip, quint16 port, int videoPort);
    void onTcpConnected();
    void onTcpDisconnected();
    void onTcpError(const QString& message);
    void onDisconnectRequested();
    void onLogoutRequested();

private:
    QStackedWidget* m_stack = nullptr;
    LoginPage* m_loginPage = nullptr;
    ConnectPage* m_connectPage = nullptr;
    DashboardPage* m_dashboardPage = nullptr;

    TcpClientManager* m_tcpClient = nullptr;
    VideoReceiver* m_videoReceiver = nullptr;

    int m_videoPort = 5000;
};
