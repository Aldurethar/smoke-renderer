#pragma once

#include <QMainWindow>

#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace Ui
{
	class GLMainWindow;
}

class OpenGLRenderer;
class QShortcut;

class GLMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	GLMainWindow(QWidget * parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
	~GLMainWindow();

	void setRendererFactory(std::function<OpenGLRenderer * (QObject * parent)> rendererFactory);

public slots:
	void setOpenGLLoggingEnabled(bool enabled);
	void setOpenGLLoggingSynchronous(bool synchronous);

signals:
	void openGLLoggingEnabledChanged(bool enabled);
	void openGLLoggingSynchronousChanged(bool synchronous);

private slots:
	// on_<x>_<y> slots are connected automatically by name when the ui is set up
	void on_actionFullScreen_toggled(bool checked);
	void on_actionFullScreenOpenGL_toggled(bool checked);
	void on_actionAbout_triggered();

private:
	std::unique_ptr<Ui::GLMainWindow> ui;

	std::map<QWidget *, bool> savedVisibilities;

	void fillActionShortcuts(QWidget * base);
	std::vector<QShortcut *> actionShortcuts;
};
