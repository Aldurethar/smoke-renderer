#include "GLMainWindow.hpp"
#include "ui_GLMainWindow.h"

#ifdef _WIN32
#include <QtPlatformHeaders/QWindowsWindowFunctions>
#endif

#include <QMessageBox>
#include <QShortcut>

GLMainWindow::GLMainWindow(QWidget * parent, Qt::WindowFlags f)
	: QMainWindow{parent, f}
	, ui{new Ui::GLMainWindow}
{
	this->ui->setupUi(this);
	this->setWindowTitle(QApplication::applicationDisplayName());

	// forward signals
	this->connect(this->ui->openGLWidget, &OpenGLWidget::loggingEnabledChanged, this, &GLMainWindow::openGLLoggingEnabledChanged);
	this->connect(this->ui->openGLWidget, &OpenGLWidget::loggingSynchronousChanged, this, &GLMainWindow::openGLLoggingSynchronousChanged);

	this->ui->actionExit->setShortcuts(QKeySequence::Quit);
	this->ui->actionFullScreen->setShortcuts(QKeySequence::FullScreen);

	// we hide the menuBar in full screen OpenGL mode, but this disables shortcuts as well, so we clone them
	this->fillActionShortcuts(this->menuBar());
	// add an additional shortcut (Escape) to leave full screen OpenGL mode
	{
		auto action = this->ui->actionFullScreenOpenGL;
		this->actionShortcuts.emplace_back(new QShortcut{QKeySequence::fromString(tr("Esc")), this});
		auto actionShortcut = this->actionShortcuts.back();
		actionShortcut->setAutoRepeat(false);
		actionShortcut->setEnabled(false);
		this->connect(actionShortcut, &QShortcut::activated, action, [action] { if(action->isEnabled()) action->trigger(); });
	}
}

GLMainWindow::~GLMainWindow() = default;

void GLMainWindow::setRendererFactory(std::function<OpenGLRenderer * (QObject * parent)> rendererFactory)
{
	this->ui->openGLWidget->setRendererFactory(std::move(rendererFactory));
}

// forward slots
void GLMainWindow::setOpenGLLoggingEnabled(bool enabled) { this->ui->openGLWidget->setLoggingEnabled(enabled); }
void GLMainWindow::setOpenGLLoggingSynchronous(bool synchronous) { this->ui->openGLWidget->setLoggingSynchronous(synchronous); }

void GLMainWindow::on_actionFullScreen_toggled(bool checked)
{
#ifdef _WIN32
	// add a window border to ensure window compositing is not disabled, otherwise context menus etc. stop working
	QWindowsWindowFunctions::setHasBorderInFullScreen(this->window()->windowHandle(), true);
	this->showNormal();
#endif

	if(checked)
		this->showFullScreen();
	else
		this->showNormal();
}

void GLMainWindow::on_actionFullScreenOpenGL_toggled(bool checked)
{
	this->ui->actionFullScreen->setEnabled(!checked);

	if(checked)
	{
		this->savedVisibilities.clear();
		for(auto child : this->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly))
		{
			if(child == this->ui->openGLWidget)
				continue;

			this->savedVisibilities[child] = child->isVisible();
			child->setVisible(false);
		}
	}
	else
	{
		for(auto child : this->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly))
		{
			if(child == this->ui->openGLWidget)
				continue;

			auto it = this->savedVisibilities.find(child);
			if(it != this->savedVisibilities.end())
				child->setVisible(it->second);
		}
	}

	// enable/disable shortcuts depending on menuBar visibility
	for(auto shortcut : this->actionShortcuts)
		shortcut->setEnabled(checked);

#ifdef _WIN32
	// see on_actionFullScreen_toggled
	QWindowsWindowFunctions::setHasBorderInFullScreen(this->window()->windowHandle(), !checked);
	this->showNormal();
#endif

	if(checked || this->ui->actionFullScreen->isChecked())
		this->showFullScreen();
	else
		this->showNormal();
}

void GLMainWindow::on_actionAbout_triggered()
{
	QMessageBox::about(this, tr("About %1").arg(QApplication::applicationDisplayName()), tr("This application is based on the simulation framework for the TU Darmstadt lecture on physically based animation."));
}

void GLMainWindow::fillActionShortcuts(QWidget * base)
{
	for(auto action : base->actions())
	{
		if(auto menu = action->menu())
		{
			this->fillActionShortcuts(menu);
			continue;
		}

		if(action->isSeparator())
			continue;

		for(auto && shortcut : action->shortcuts())
		{
			this->actionShortcuts.emplace_back(new QShortcut{shortcut, this});
			auto actionShortcut = this->actionShortcuts.back();
			actionShortcut->setAutoRepeat(false);
			// disable shortcuts by default to avoid ambiguity when menuBar is visible
			actionShortcut->setEnabled(false);
			this->connect(actionShortcut, &QShortcut::activated, action, [action] { if(action->isEnabled() && action->isVisible()) action->trigger(); });
		}
	}
}
