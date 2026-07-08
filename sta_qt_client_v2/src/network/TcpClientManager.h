#pragma once

#include <QJsonObject>
#include <QObject>
#include <QTcpSocket>

class TcpClientManager : public QObject
{
    Q_OBJECT

public:
    explicit TcpClientManager(QObject* parent = nullptr);
    ~TcpClientManager() override;

    bool connectToServer(const QString& host, quint16 port);
    void disconnectFromServer();
    bool isConnected() const;

    QString host() const { return m_host; }
    quint16 port() const { return m_port; }

public slots:
    bool sendJson(const QJsonObject& object);
    bool sendRawLine(const QString& line);

signals:
    void connected();
    void disconnected();
    void messageReceived(const QJsonObject& object);
    void rawMessageReceived(const QString& line);
    void logMessage(const QString& message);
    void errorMessage(const QString& message);

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onErrorOccurred(QAbstractSocket::SocketError socketError);

private:
    void processLine(const QByteArray& line);

private:
    QTcpSocket* m_socket = nullptr;
    QByteArray m_buffer;
    QString m_host;
    quint16 m_port = 0;
};
