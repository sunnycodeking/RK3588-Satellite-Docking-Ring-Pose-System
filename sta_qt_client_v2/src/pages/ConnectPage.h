#pragma once

#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;

class ConnectPage : public QWidget
{
    Q_OBJECT

public:
    explicit ConnectPage(QWidget* parent = nullptr);

    void setBusy(bool busy);
    void setConnected(bool connected);
    void setStatusText(const QString& text);

    int videoPort() const;

signals:
    void connectionRequested(const QString& ip, quint16 port, int videoPort);
    void backToLoginRequested();

private slots:
    void onConnectClicked();

private:
    QLineEdit* m_ipEdit = nullptr;
    QSpinBox* m_portSpin = nullptr;
    QSpinBox* m_videoPortSpin = nullptr;
    QLabel* m_statusLabel = nullptr;
    QPushButton* m_connectButton = nullptr;
    QPushButton* m_backButton = nullptr;
};
