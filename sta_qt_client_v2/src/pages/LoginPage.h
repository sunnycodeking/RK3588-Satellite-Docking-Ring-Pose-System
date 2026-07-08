#pragma once

#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;

class LoginPage : public QWidget
{
    Q_OBJECT

public:
    explicit LoginPage(QWidget* parent = nullptr);

signals:
    void loginSucceeded();

private slots:
    void onLoginClicked();

private:
    QLineEdit* m_usernameEdit = nullptr;
    QLineEdit* m_passwordEdit = nullptr;
    QLabel* m_hintLabel = nullptr;
    QPushButton* m_loginButton = nullptr;
};
