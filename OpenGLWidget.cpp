#include "OpenGLWidget.hpp"
#include "OpenGLRenderer.hpp"

#include <QDebug>
#include <QEvent>
#include <QMouseEvent>
#include <QOpenGLDebugLogger>

#include <cassert>

OpenGLWidget::OpenGLWidget(QWidget * parent, Qt::WindowFlags f)
	: QOpenGLWidget{parent, f}
	, logger{nullptr}
	, loggingEnabled{false}
	, loggingSynchronous{false}
	, renderer{nullptr}
{}

void OpenGLWidget::setRendererFactory(std::function<OpenGLRenderer * (QObject * parent)> rendererFactory)
{
	this->rendererFactory = std::move(rendererFactory);
	if(!this->isValid())
		return;

	this->makeCurrent();
	delete this->renderer;
	this->renderer = nullptr;

	if(this->rendererFactory)
	{
		this->renderer = this->rendererFactory(this);
		if(this->renderer)
			this->renderer->resize(this->width(), this->height());
	}

	this->update();
	this->doneCurrent();
}

bool OpenGLWidget::event(QEvent * e)
{
	switch (e->type())
	{
	case QEvent::MouseButtonPress:
	case QEvent::MouseButtonRelease:
	case QEvent::MouseMove:
		if (renderer)
			renderer->mouseEvent(static_cast<QMouseEvent *>(e));
		return true;
	case QEvent::Wheel:
		if (renderer)
			renderer->wheelEvent(static_cast<QWheelEvent *>(e));
		return true;
	}
	return QOpenGLWidget::event(e);
}

void OpenGLWidget::setLoggingEnabled(bool enable)
{
	if(enable == this->loggingEnabled)
		return;

	this->loggingEnabled = enable;
	emit this->loggingEnabledChanged(enable);

	if(!logger)
		return;

	this->makeCurrent();
	if(loggingEnabled)
		this->logger->startLogging(this->loggingSynchronous ? QOpenGLDebugLogger::SynchronousLogging : QOpenGLDebugLogger::AsynchronousLogging);
	else
		this->logger->stopLogging();
	this->doneCurrent();
}

void OpenGLWidget::setLoggingSynchronous(bool synchronous)
{
	if(synchronous == this->loggingSynchronous)
		return;

	this->loggingSynchronous = synchronous;
	emit this->loggingSynchronousChanged(synchronous);

	if(!this->logger || !this->loggingEnabled)
		return;

	this->makeCurrent();
	this->logger->stopLogging();
	this->logger->startLogging(synchronous ? QOpenGLDebugLogger::SynchronousLogging : QOpenGLDebugLogger::AsynchronousLogging);
	this->doneCurrent();
}

void OpenGLWidget::initializeGL()
{
	if(!this->logger)
	{
		this->logger = new QOpenGLDebugLogger{this};
		connect(this->logger, &QOpenGLDebugLogger::messageLogged, [] (QOpenGLDebugMessage const & debugMessage) {
			qDebug() << debugMessage;
		});
		this->logger->initialize();
		this->logger->disableMessages(QOpenGLDebugMessage::AnySource, QOpenGLDebugMessage::AnyType, QOpenGLDebugMessage::NotificationSeverity);

		if(this->loggingEnabled)
			this->logger->startLogging(this->loggingSynchronous ? QOpenGLDebugLogger::SynchronousLogging : QOpenGLDebugLogger::AsynchronousLogging);
	}
	
	// using a thread_local static variable as gladLoadGLLoader does not allow passing of user data
	thread_local QOpenGLContext * gl_context = nullptr;
	gl_context = context();
	gladLoadGLLoader([] (char const * name) -> void * { return gl_context->getProcAddress(name); });

	assert(this->renderer == nullptr);

	if(!this->rendererFactory)
		return;

	this->renderer = this->rendererFactory(this);
	if(this->renderer)
		this->renderer->resize(this->width(), this->height());
}

void OpenGLWidget::paintGL()
{
	if(this->renderer)
		this->renderer->render();
	update();
}

void OpenGLWidget::resizeGL(int w, int h)
{
	if(this->renderer)
		this->renderer->resize(w, h);
}
