#include "vgl.h"


typedef struct {
	GLenum type;
	const char* filename;
} ShaderInfo;

GLuint LoadShaders(ShaderInfo* shaders);
