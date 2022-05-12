#pragma once

#include <QObject>

class QMouseEvent;
class QWheelEvent;

class OpenGLRenderer : public QObject
{
	Q_OBJECT

public:
	using QObject::QObject;

	virtual void resize(int w, int h) = 0;
	virtual void render() = 0;

	virtual void mouseEvent(QMouseEvent * e) = 0;
	virtual void wheelEvent(QWheelEvent* e) = 0;

signals:
	void update();
};
