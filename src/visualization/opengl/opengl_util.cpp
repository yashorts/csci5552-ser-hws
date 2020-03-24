#include <visualization/opengl/opengl_utils.h>

SDL_GLContext context;
std::vector<int> shaderprogs;
GLuint vao;

using namespace std;

float rand01() {
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

SDL_Window* GL_SDL_init(const char* window_name) {
	SDL_Init(SDL_INIT_VIDEO);  //Initialize Graphics (for OpenGL)
    
    //Ask SDL to get a recent version of OpenGL (3.2 or greater)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	SDL_GL_SetSwapInterval(1);
	
	//Create a window (offsetx, offsety, width, height, flags)
	SDL_Window* window = SDL_CreateWindow(window_name, 100, 100, 800, 600, SDL_WINDOW_OPENGL);
	
	//SDL_SetRelativeMouseMode(SDL_TRUE);
	
	//Create a context to draw in
	context = SDL_GL_CreateContext(window);
	
	glEnable(GL_MULTISAMPLE);
	
	//GLEW loads new OpenGL functions
	glewExperimental = GL_TRUE; //Use the new way of testing which methods are supported
	glewInit();
	
	//Build a Vertex Array Object. This stores the VBO and attribute mappings in one object
	glGenVertexArrays(1, &vao); //Create a VAO
	glBindVertexArray(vao); //Bind the above created VAO to the current context
	
	return window;
}

void GL_SDL_del() {
	//Clean Up
	for (unsigned int i = 0; i < shaderprogs.size(); i++) {
		glDeleteProgram(shaderprogs[i]);
	}
    glDeleteVertexArrays(1, &vao);
	
	SDL_GL_DeleteContext(context);
	SDL_Quit();
}

// Create a GLSL program object from vertex and fragment shader files
GLuint InitShader(const char* vShaderFileName, const char* fShaderFileName, const char* gShaderFileName)
{
	GLint link_ok;
	GLuint vs,gs,fs;
	if ((vs = create_shader(vShaderFileName, GL_VERTEX_SHADER))   == 0) exit(-1);
	if ((gs = create_shader(gShaderFileName, GL_GEOMETRY_SHADER))   == 0) exit(-1);
	if ((fs = create_shader(fShaderFileName, GL_FRAGMENT_SHADER)) == 0) exit(-1);

	GLuint program = glCreateProgram();
	glAttachShader(program, vs);
	glAttachShader(program, gs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
	if (!link_ok) {
	fprintf(stderr, "glLinkProgram:");
	print_log(program);
	return 0;
	}
	glUseProgram(program);
	
	shaderprogs.push_back(program);
	
	return program;
}

// Create a GLSL program object from vertex and fragment shader files
GLuint InitShader(const char* vShaderFileName, const char* fShaderFileName)
{
	GLint link_ok;
	GLuint vs,fs;
	if ((vs = create_shader(vShaderFileName, GL_VERTEX_SHADER))   == 0) exit(-1);
	if ((fs = create_shader(fShaderFileName, GL_FRAGMENT_SHADER)) == 0) exit(-1);

	GLuint program = glCreateProgram();
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
	if (!link_ok) {
	fprintf(stderr, "glLinkProgram:");
	return 0;
	}
	glUseProgram(program);
	
	shaderprogs.push_back(program);
	
	return program;
}

char* file_read(const char* filename)
{
  FILE* input = fopen(filename, "rb");
  if(input == NULL) return NULL;
 
  if(fseek(input, 0, SEEK_END) == -1) return NULL;
  long size = ftell(input);
  if(size == -1) return NULL;
  if(fseek(input, 0, SEEK_SET) == -1) return NULL;
 
  /*if using c-compiler: dont cast malloc's return value*/
  char *content = (char*) malloc( (size_t) size +1  ); 
  if(content == NULL) return NULL;
 
  int rLen = fread(content, 1, (size_t)size, input);
  if(ferror(input) || rLen == 0) {
    free(content);
    return NULL;
  }
 
  fclose(input);
  content[size] = '\0';
  return content;
}

void print_log(GLuint object)
{
  GLint log_length = 0;
  if (glIsShader(object))
    glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
  else if (glIsProgram(object))
    glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
  else {
    fprintf(stderr, "printlog: Not a shader or a program\n");
    return;
  }
 
  char* log = (char*)malloc(log_length);
 
  if (glIsShader(object))
    glGetShaderInfoLog(object, log_length, NULL, log);
  else if (glIsProgram(object))
    glGetProgramInfoLog(object, log_length, NULL, log);
 
  fprintf(stderr, "%s", log);
  free(log);
}

GLuint create_shader(const char* filename, GLenum type)
{
	const GLchar* source = file_read(filename);
	if (source == NULL) {
		fprintf(stderr, "Error opening %s: ", filename); perror("");
		return 0;
	}
	GLuint res = glCreateShader(type);
	const GLchar* sources[2] = {"",	source };
	glShaderSource(res, 2, sources, NULL);
	free((void*)source);

	glCompileShader(res);
	GLint compile_ok = GL_FALSE;
	glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
	if (compile_ok == GL_FALSE) {
		fprintf(stderr, "%s:", filename);
		print_log(res);
		glDeleteShader(res);
		return 0;
	}
 
  return res;
}

int loadModel(string fName, float* &model1) {
	ifstream modelFile;
	modelFile.open(fName.c_str());
	int numLines = 0;
	modelFile >> numLines;
	model1 = new float[numLines];
	for (int i = 0; i < numLines; i++){
		modelFile >> model1[i];
	}
	modelFile.close();
	return numLines/8;
}

// GLuint loadTexture(const char* file, int &width, int &height) {
// 	int channels;
// 	stbi_set_flip_vertically_on_load(true);
// 	unsigned char *image = stbi_load(file,
// 									&width,
// 									&height,
// 									&channels,
// 									STBI_rgb_alpha);

//    //Now generate the OpenGL texture object
//    GLuint texture;
//    glGenTextures(1, &texture);
//    glBindTexture(GL_TEXTURE_2D, texture);
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//    glTexImage2D(GL_TEXTURE_2D,0, GL_RGBA, width, height, 0,
//        GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid*) image);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
//    glGenerateMipmap(GL_TEXTURE_2D);
 
//    //clean up memory and close stuff
//    stbi_image_free(image);
 
//    return texture;
//  }