#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifdef USE_GLAD
#include <glad/glad.h>
#else
#include <GL/glew.h>
#endif

#include <memory>
#include <utility>

#pragma push_macro("OPENGL_OBJECT_PLURAL")
#ifdef OPENGL_OBJECT_PLURAL
#undef OPENGL_OBJECT_PLURAL
#endif
#define OPENGL_OBJECT_PLURAL(name_, plural_) \
	struct name_ \
	{ \
		name_() { glGen##plural_(1, &id_); } \
		name_(name_ const &) = delete; \
		name_(name_ && o) : id_(o.id_) { o.id_ = 0; } \
		~name_() { glDelete##plural_(1, &id_); } \
		name_ & operator=(name_ const &) = delete; \
		name_ & operator=(name_ && o) { std::swap(id_, o.id_); return *this; } \
		GLuint id() const { return id_; } \
	private: \
		GLuint id_; \
	}

#pragma push_macro("OPENGL_OBJECT")
#ifdef OPENGL_OBJECT
#undef OPENGL_OBJECT
#endif
#define OPENGL_OBJECT(name_) OPENGL_OBJECT_PLURAL(name_, name_ ## s)

namespace gl
{
	// GL 2.0
	OPENGL_OBJECT(Buffer);
	OPENGL_OBJECT(Framebuffer);
	OPENGL_OBJECT(Texture);
	OPENGL_OBJECT_PLURAL(Query, Queries);

	struct Shader
	{
		Shader(GLenum type) : id_(glCreateShader(type)) {}
		Shader(Shader const &) = delete;
		Shader(Shader && o) : id_(o.id_) { o.id_ = 0; }
		~Shader() { glDeleteShader(id_); }
		Shader & operator=(Shader const &) = delete;
		Shader & operator=(Shader && o) { std::swap(id_, o.id_); return *this; }
		GLuint id() const { return id_; }

		GLint compile(GLsizei count, GLchar const ** strings, GLint const * lengths);
		GLint compile(GLchar const * string) { return compile(1, &string, nullptr); }
		GLint compile(GLchar const * string, GLint length) { return compile(1, &string, &length); }

		std::unique_ptr<char[]> infoLog();

	private:
		GLuint id_;
	};

	struct Program
	{
		Program() : id_(glCreateProgram()) {}
		Program(Program const &) = delete;
		Program(Program && o) : id_(o.id_) { o.id_ = 0; }
		~Program() { glDeleteProgram(id_); }
		Program & operator=(Program const &) = delete;
		Program & operator=(Program && o) { std::swap(id_, o.id_); return *this; }
		GLuint id() const { return id_; }

		GLint link(std::size_t count, Shader const ** shaders); // switch to GSL span?
		template<typename... Shaders>
		GLint link(Shaders&&... shaders)
		{
			Shader const * tmp[] = { &shaders... };
			return link(sizeof...(shaders), tmp);
		}

		std::unique_ptr<char[]> infoLog();

	private:
		GLuint id_;
	};

	// GL 3.0
	OPENGL_OBJECT(VertexArray);
	OPENGL_OBJECT(Renderbuffer);

#ifdef GL_VERSION_3_2
	OPENGL_OBJECT(Sampler);
#endif

#ifdef GL_VERSION_4_0
	OPENGL_OBJECT(TransformFeedback);
#endif

#ifdef GL_VERSION_4_1
	OPENGL_OBJECT(ProgramPipeline);
#endif
}

#pragma pop_macro("OPENGL_OBJECT")
#pragma pop_macro("OPENGL_OBJECT_PLURAL")
