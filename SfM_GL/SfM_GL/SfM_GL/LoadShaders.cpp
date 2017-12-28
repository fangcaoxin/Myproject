#include "LoadShaders.h"
#include "utils.h"

GLuint LoadShaders(ShaderInfo* shaders)
{
	printf("load shaders start\n");
	GLuint vShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fShader = glCreateShader(GL_FRAGMENT_SHADER);
	const char * vSrc;
	const char * fSrc;
	GLint *vLens = NULL;
	GLint *fLens = NULL;
	vSrc = t_read(shaders[0].filename);
	fSrc = t_read(shaders[1].filename);
	printf("read complete\n");
	glShaderSource(vShader, 1, &vSrc, 0);
	glShaderSource(fShader, 1, &fSrc, 0);
	printf("start compile\n");
	glCompileShader(fShader);
	glCompileShader(vShader);


	GLint vStatus;
	GLint fStatus;
	glGetShaderiv(vShader, GL_COMPILE_STATUS, &vStatus);
	glGetShaderiv(fShader, GL_COMPILE_STATUS, &fStatus);
	printf("vStatus:%d\n", vStatus);
	printf("fStatus:%d\n", fStatus);

	GLsizei vInfoLen;
	GLsizei fInfoLen;
	glGetShaderiv(vShader, GL_INFO_LOG_LENGTH, &vInfoLen);
	glGetShaderiv(fShader, GL_INFO_LOG_LENGTH, &fInfoLen);
	char *vInfoLog = (char*)malloc(vInfoLen);
	char *fInfoLog = (char*)malloc(fInfoLen);
	glGetShaderInfoLog(vShader, vInfoLen, &vInfoLen, vInfoLog);
	glGetShaderInfoLog(fShader, fInfoLen, &fInfoLen, fInfoLog);
	printf("Compile Vertex Shader:%s\n", vInfoLog);
	printf("Compile Fragment Shader:%s\n", fInfoLog);

	if (vStatus == GL_FALSE) {
		printf("vertex shader compile failed!\n");
		return 0;
	}
	if (fStatus == GL_FALSE) {
		printf("fragment shader compile failed!\n");
		return 0;
	}
	printf("compile complete\n");

	printf("start create program\n");
	GLuint program = glCreateProgram();
	if (program == 0)
	{
		printf("Create Program Error!");
		return 0;
	}
	glAttachShader(program, vShader);
	glAttachShader(program, fShader);
	printf("attach shader complete\n");
	glLinkProgram(program);
	printf("link program complete\n");
	GLint linked;
	glGetProgramiv(program, GL_LINK_STATUS, &linked);
	GLsizei infoLen;
	char infoLog[100];
	glGetProgramInfoLog(program, 100, &infoLen, infoLog);
	printf("Link program:%s\n", infoLog);
	printf("glGetProgramiv complete\n");
	if (linked == GL_TRUE)
	{
		printf("Link success!");
		return program;
	}
	else
	{
		printf("Link failed!");
		return 0;
	}

}