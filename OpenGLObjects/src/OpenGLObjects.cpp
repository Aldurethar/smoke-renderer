#include <OpenGLObjects.h>

namespace gl
{
	GLint Shader::compile(GLsizei count, GLchar const ** strings, GLint const * lengths)
	{
		glShaderSource(id_, count, strings, lengths);
		glCompileShader(id_);

		GLint ret;
		glGetShaderiv(id_, GL_COMPILE_STATUS, &ret);
		return ret;
	}

	std::unique_ptr<char[]> Shader::infoLog()
	{
		GLint length;
		glGetShaderiv(id_, GL_INFO_LOG_LENGTH, &length);
		auto ret = std::make_unique<char[]>(length);
		glGetShaderInfoLog(id_, length, nullptr, ret.get());
		return ret;
	}

	GLint Program::link(std::size_t count, Shader const ** shaders)
	{
		for(size_t i = 0; i < count; ++i)
			glAttachShader(id_, shaders[i]->id());
		glLinkProgram(id_);
		for(size_t i = count; i--;)
			glDetachShader(id_, shaders[i]->id());

		GLint ret;
		glGetProgramiv(id_, GL_LINK_STATUS, &ret);
		return ret;
	}

	std::unique_ptr<char[]> Program::infoLog()
	{
		GLint length;
		glGetProgramiv(id_, GL_INFO_LOG_LENGTH, &length);
		auto ret = std::make_unique<char[]>(length);
		glGetProgramInfoLog(id_, length, nullptr, ret.get());
		return ret;
	}
}
