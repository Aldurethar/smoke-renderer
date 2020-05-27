#include <QApplication>
#include <QCommandLineParser>
#include <QSurfaceFormat>

#include "GLMainWindow.hpp"
#include "ExampleRenderer.hpp"
#include "MyRenderer.hpp"

int main(int argc, char ** argv)
{
	using App = QApplication;
	App app(argc, argv);
	App::setApplicationName("SimulationFramework");
	App::setApplicationDisplayName(App::translate("main", "Simulation Framework"));
	App::setApplicationVersion("1.0");

	QCommandLineParser parser;
	parser.setApplicationDescription(App::translate("main", "Simulation framework for the TU Darmstadt lecture on physically based animation."));
	parser.addHelpOption();
	parser.addVersionOption();

	QCommandLineOption debugGLOption({ "g", "debug-gl" }, App::translate("main", "Enable OpenGL debug logging"));
	parser.addOption(debugGLOption);

	parser.process(app);

	auto surfaceFormat = QSurfaceFormat::defaultFormat();
	surfaceFormat.setVersion(4, 3);
	surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
	surfaceFormat.setOption(QSurfaceFormat::DebugContext);
	QSurfaceFormat::setDefaultFormat(surfaceFormat);

	GLMainWindow widget;
	if(parser.isSet(debugGLOption))
	{
		widget.setOpenGLLoggingSynchronous(true);
		widget.setOpenGLLoggingEnabled(true);
	}

	widget.setRendererFactory(
		[] (QObject * parent) {
			return new MyRenderer{parent};
		}
	);
	widget.show();

	return app.exec();
}
