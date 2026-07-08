#include "TcpClientManager.h"

#include <QJsonDocument>
#include <QtGlobal>

TcpClientManager::TcpClientManager(QObject* parent)
    : QObject(parent)
    , m_socket(new QTcpSocket(this))
{
    connect(m_socket, &QTcpSocket::connected, this, &TcpClientManager::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &TcpClientManager::onDisconnected);
    connect(m_socket, &QTcpSocket::readyRead, this, &TcpClientManager::onReadyRead);

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    connect(m_socket, &QAbstractSocket::errorOccurred, this, &TcpClientManager::onErrorOccurred);
#else
    connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this, &TcpClientManager::onErrorOccurred);
#endif
}

TcpClientManager::~TcpClientManager()
{
    disconnectFromServer();
}

bool TcpClientManager::connectToServer(const QString& host, quint16 port)
{
    if (host.trimmed().isEmpty() || port == 0) {
        emit errorMessage("服务器 IP 或端口为空");
        return false;
    }

    if (m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->abort();
    }

    m_host = host.trimmed();
    m_port = port;
    m_buffer.clear();

    emit logMessage(QString("正在连接服务器 %1:%2 ...").arg(m_host).arg(m_port));
    m_socket->connectToHost(m_host, m_port);
    return true;
}

void TcpClientManager::disconnectFromServer()
{
    if (!m_socket) {
        return;
    }

    if (m_socket->state() == QAbstractSocket::UnconnectedState) {
        return;
    }

    emit logMessage("正在断开服务器连接 ...");
    m_socket->disconnectFromHost();
    if (m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->waitForDisconnected(800);
    }
}

bool TcpClientManager::isConnected() const
{
    return m_socket && m_socket->state() == QAbstractSocket::ConnectedState;
}

bool TcpClientManager::sendJson(const QJsonObject& object)
{
    if (!isConnected()) {
        emit errorMessage("TCP 未连接，命令未发送");
        return false;
    }

    const QByteArray payload = QJsonDocument(object).toJson(QJsonDocument::Compact) + '\n';
    const qint64 written = m_socket->write(payload);
    if (written != payload.size()) {
        emit errorMessage("TCP 写入失败或写入不完整");
        return false;
    }

    emit logMessage(QString("发送: %1").arg(QString::fromUtf8(payload).trimmed()));
    return true;
}

bool TcpClientManager::sendRawLine(const QString& line)
{
    if (!isConnected()) {
        emit errorMessage("TCP 未连接，命令未发送");
        return false;
    }

    QByteArray payload = line.trimmed().toUtf8();
    if (!payload.endsWith('\n')) {
        payload.append('\n');
    }

    const qint64 written = m_socket->write(payload);
    if (written != payload.size()) {
        emit errorMessage("TCP 写入失败或写入不完整");
        return false;
    }

    emit logMessage(QString("发送原始命令: %1").arg(QString::fromUtf8(payload).trimmed()));
    return true;
}

void TcpClientManager::onConnected()
{
    emit logMessage(QString("服务器已连接: %1:%2").arg(m_host).arg(m_port));
    emit connected();
}

void TcpClientManager::onDisconnected()
{
    emit logMessage("服务器连接已断开");
    emit disconnected();
}

void TcpClientManager::onReadyRead()
{
    m_buffer.append(m_socket->readAll());

    int newlineIndex = -1;
    while ((newlineIndex = m_buffer.indexOf('\n')) >= 0) {
        const QByteArray line = m_buffer.left(newlineIndex).trimmed();
        m_buffer.remove(0, newlineIndex + 1);
        if (!line.isEmpty()) {
            processLine(line);
        }
    }
}

void TcpClientManager::onErrorOccurred(QAbstractSocket::SocketError socketError)
{
    Q_UNUSED(socketError)
    emit errorMessage(QString("TCP 错误: %1").arg(m_socket->errorString()));
}

void TcpClientManager::processLine(const QByteArray& line)
{
    const QString text = QString::fromUtf8(line);
    emit rawMessageReceived(text);
    emit logMessage(QString("接收: %1").arg(text));

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(line, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        emit errorMessage(QString("收到非 JSON 数据: %1").arg(text));
        return;
    }

    emit messageReceived(doc.object());
}
