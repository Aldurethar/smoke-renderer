#include <QApplication>
#include <QCommandLineParser>
#include <QSurfaceFormat>

#include "GLMainWindow.hpp"
#include "MyRenderer.hpp"

int main(int argc, char ** argv)
{
	// set up a Qt application
	using App = QApplication;
	App app(argc, argv);
	App::setApplicationName("SimulationFramework");
	App::setApplicationDisplayName(App::translate("main", "Simulation Framework"));
	App::setApplicationVersion("1.0");
	App::setAttribute(Qt::AA_UseDesktopOpenGL);

	// configure command line parser
	QCommandLineParser parser;
	parser.setApplicationDescription(App::translate("main", "Simulation framework for the TU Darmstadt lecture on physically based simulation and animation."));
	parser.addHelpOption();
	parser.addVersionOption();

	// provide a flag for OpenGL debugging (outputs can be viewed with debugger!)
	QCommandLineOption debugGLOption({ "g", "debug-gl" }, App::translate("main", "Enable OpenGL debug logging"));
	parser.addOption(debugGLOption);

	// parse command line
	parser.process(app);

	// set up OpenGL surface format
	auto surfaceFormat = QSurfaceFormat::defaultFormat();
	surfaceFormat.setVersion(4, 5);
	surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
	surfaceFormat.setOption(QSurfaceFormat::DebugContext);
	surfaceFormat.setColorSpace(QSurfaceFormat::sRGBColorSpace);
	surfaceFormat.setSamples(4);
	QSurfaceFormat::setDefaultFormat(surfaceFormat);

	// create main window. modify GLMainWindow.ui to add widgets etc.
	GLMainWindow widget;

	// enable OpenGL error logging (look at debugger output!) if flag is passed
	if(parser.isSet(debugGLOption))
	{
		widget.setOpenGLLoggingSynchronous(true);
		widget.setOpenGLLoggingEnabled(true);
	}

	// set up which renderer to use. factory to create renderer when OpenGL context exists
	widget.setRendererFactory(
		[] (QObject * parent) {
			// MyRenderer contains our OpenGL code
			return new MyRenderer{parent};
		}
	);

	// show the main window
	widget.show();

	// run the event loop (do not write your own!)
	return app.exec();
}
