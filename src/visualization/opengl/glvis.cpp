#include <visualization/opengl/glvis.h>

#include <chrono>

void GLVisualizer::handleEvents() {
	SDL_Event windowEvent;
	while (SDL_PollEvent(&windowEvent)){
		if (windowEvent.type == SDL_QUIT) {
			running = false;
			break;
		}
  }
}

GLVisualizer::GLVisualizer() {
  // Fork off the drawing loop
  drawing_thread = std::thread(&GLVisualizer::DrawLoop, this);
}

void GLVisualizer::DrawLoop() {
  wind = GL_SDL_init("GLVis");
  glEnable(GL_DEPTH_TEST);
  
  line_buffer = new float[MAX_LINE_SEGMENTS * SEG_SIZE];

  glGenBuffers(1, vbo);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]); //Set the vbo as the active array buffer (Only one buffer can be active at a time)
	glBufferData(GL_ARRAY_BUFFER, MAX_LINE_SEGMENTS*SEG_SIZE*sizeof(float), line_buffer, GL_STREAM_DRAW); //upload vertices to vbo

  std::string fShader = std::string(GL_SHADER_PATH) + std::string("fragShader.glsl");
  std::string vShader = std::string(GL_SHADER_PATH) + std::string("vertShader.glsl");

  shaderProgram = InitShader(vShader.c_str(), fShader.c_str());

	while (running) {
    auto t_iter_start = std::chrono::high_resolution_clock::now();
    handleEvents();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    Draw(wind);

    SDL_GL_SwapWindow(wind); //Double buffering
    // Try to run at 100 fps
    std::this_thread::sleep_until(std::chrono::milliseconds(10) + t_iter_start);
  }

  // Do cleanup
  delete[] line_buffer;
  glDeleteBuffers(1, vbo);
	GL_SDL_del();
}

void GLVisualizer::UpdateLines() {
  temp_line_lock.lock();
  if (clear_temp) {
    temp_lines.clear();
  }
  for (Line l : new_temp_lines) {
    temp_lines.push_back(l);
  }
  new_temp_lines.clear();
  temp_line_lock.unlock();
  perm_line_lock.lock();
  if (clear_perm) {
    perm_lines.clear();
  }
  for (Line l : new_perm_lines) {
    perm_lines.push_back(l);
  }
  new_perm_lines.clear();
  perm_line_lock.unlock();
}  

void GLVisualizer::AddTempLine(std::vector<Eigen::VectorXd> line, Color c, double pix_width) {
  Line l;
  l.segs = line;
  l.c = c;
  l.width = pix_width;
  new_temp_lines.push_back(l);
}

void GLVisualizer::ClearTempLines() {
  clear_temp = true;
  new_temp_lines.clear();
}

void GLVisualizer::AddPermLine(std::vector<Eigen::VectorXd> line, Color c, double pix_width) {
  Line l;
  l.segs = line;
  l.c = c;
  l.width = pix_width;
  new_perm_lines.push_back(l);
}

void GLVisualizer::ClearPermLines() {
  clear_perm = true;
  new_perm_lines.clear();
}

void GLVisualizer::SetCenterAndHeight(Eigen::Vector2d cen, double height) {
  cen_x = cen[0];
  cen_y = cen[1];
  view_height = height;
}

void GLVisualizer::DrawBackground() {
  glDepthMask(GL_FALSE);
  float max_y = 0.5f * view_height, min_y = -max_y;
  float max_x = max_y * static_cast<float>(w) / static_cast<float>(h), min_x = -max_x;
  
  // TODO: Switch shader programs to a textured one?

  glDepthMask(GL_TRUE);
}

void GLVisualizer::DrawLine(Line& l) {
  // Lines are the same width, regardless of zoom level
  float lw = l.width / static_cast<float>(h) * view_height;
  if (l.segs.size() < 2) {
    return;
  }
  // Set the line color
  Eigen::Vector3f c_vec = ColorToRGB(l.c);
  glUniform3f(glGetUniformLocation(shaderProgram, "color"), c_vec[0], c_vec[1], c_vec[2]);

  Eigen::Vector2f p1(l.segs[0][0], l.segs[0][1]), p2; //, p_end1 = p1, p_end2 = p2;
  for (size_t i = 1; i < l.segs.size(); ++i) {
    p2 = Eigen::Vector2f(l.segs[i][0], l.segs[i][1]);
    if ((p1 - p2).norm() == 0.0f) {
      // Don't Draw this segment
      continue;
    }
    // First, compute the four endpoints
    Eigen::Vector2f v = (p2 - p1).normalized(), v_perp = Eigen::Vector2f(-v[1], v[0]);
    Eigen::Vector2f tl = p1 + lw * v_perp, bl = p1 - lw * v_perp;
    Eigen::Vector2f tr = p2 + lw * v_perp, br = p2 - lw * v_perp;

    // TODO: Draw a sort of cap?

    // Draw CCW:
    // bl, br, tl, tr
    line_buffer[0] = bl.x();
    line_buffer[1] = bl.y();
    line_buffer[2] = tl.x();
    line_buffer[3] = tl.y();
    line_buffer[4] = br.x();
    line_buffer[5] = br.y();
    line_buffer[6] = tr.x();
    line_buffer[7] = tr.y();

    glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * SEG_SIZE * sizeof(float), line_buffer);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // p_end1 = tr; p_end2 = br;
    p1 = p2;
  }
}

void GLVisualizer::DrawLines() {
  perm_line_lock.lock();
  for (auto& line : perm_lines) {
    DrawLine(line);
  }
  perm_line_lock.unlock();
  temp_line_lock.lock();
  for (auto& line : temp_lines) {
    DrawLine(line);
  }
  temp_line_lock.unlock();
}

void GLVisualizer::Draw(SDL_Window* window) {
	SDL_GetWindowSize(window, &w, &h);
	glViewport(0,0,w,h);
  float max_y = 0.5f * view_height, min_y = -max_y;
  float max_x = max_y * static_cast<float>(w) / static_cast<float>(h), min_x = -max_x;
  proj = glm::ortho(min_x + cen_x, max_x + cen_x, min_y + cen_y, max_y + cen_y, 1.0f, 4.0f); // near, far
	
  view = glm::lookAt(
    glm::vec3(0.0f,0.0f,2.0f),  //Cam Position
		glm::vec3(0.0f, 0.0f, 0.0f), //Look at point
		glm::vec3(0.0f, 1.0f, 0.0f)); //Up
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setActiveProgram(shaderProgram);

  DrawBackground();
  DrawLines();

	glDisable(GL_BLEND);
}

void GLVisualizer::PauseFigure(double pause_time_s) {
  if (pause_time_s < 0) {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } else {
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int>(pause_time_s * 1e9)));
  }
}

GLVisualizer::~GLVisualizer() {
  running = false;
  if (drawing_thread.joinable()) {
    drawing_thread.join();
  }
}

void GLVisualizer::setActiveProgram(GLint program) {
	glUseProgram(program);
	
	GLint uniView = glGetUniformLocation(program, "view");
	glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
	
	GLint uniProj = glGetUniformLocation(program, "proj");
	glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));
	
	//Tell OpenGL how to set shader input 
	GLint posAttrib = glGetAttribLocation(program, "pos");
	glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(posAttrib);
	// GLint texAttrib = glGetAttribLocation(program, "tex_coord");
	// glVertexAttribPointer(texAttrib, 2, GL_FLOAT, GL_FALSE, PT_SIZE*sizeof(float), (void*)(3*sizeof(float)));
	// glEnableVertexAttribArray(texAttrib);
}

// // SDL event handling:
// void GLVisualizer::handleMouseScroll(SDL_Event& e) {
//   if (!adding_mode) {
//     view_scale = std::min(10.0f, std::max(0.1f, view_scale - view_scale / 20 * static_cast<float>(e.wheel.y)));
//   } else if (add_pos_mode) {
//     if (e.wheel.y > 0) {
//       virt_agent_type += 1;
//     } else if (e.wheel.y < 0) {
//       virt_agent_type -= 1;
//     }
//     // Wrap Agent Type
//     if (virt_agent_type < 0) {
//       virt_agent_type = 5;
//       virt_agent_react = !virt_agent_react;
//     } else if (virt_agent_type > 5) {
//       virt_agent_type = 0;
//       virt_agent_react = !virt_agent_react;
//     }
//   }
// }

// void GLVisualizer::handleMouseButtonDown(SDL_Event& e) {
//   if (e.button.button == SDL_BUTTON_LEFT) {
//     left_mouse_down = true;
//     held_pos_x = e.motion.x;
//     held_pos_y = e.motion.y;
//   }
// }

// void GLVisualizer::handleMouseButtonUp(SDL_Event& e) {
//   if (e.button.button == SDL_BUTTON_LEFT) {
//     left_mouse_down = false;
//     if (adding_mode) {
//       // Step new Agent setup Phase
//       if (add_pos_mode) {
//         if (virt_agent_type > 1) { // Not V or A agent
//           add_orient_mode = true;
//           add_pos_mode = false;
//         } else {
//           add_goal_mode = true;
//           add_pos_mode = false;
//           int x,y;
//           SDL_GetMouseState(&x, &y);
//           ConvertMouseXY(x, y, virt_agent_gx, virt_agent_gy);
//         }
//       } else if (add_orient_mode) {
//         add_goal_mode = true;
//         add_orient_mode = false;
//         int x,y;
//         SDL_GetMouseState(&x, &y);
//         ConvertMouseXY(x, y, virt_agent_gx, virt_agent_gy);
//       } else if (add_goal_mode) {
//         AddVirtualAgent();
//         adding_mode = false;
//         add_goal_mode = false;
//       }
//     } else {
//       cen_x += x_offset;
//       cen_y += y_offset;
//     }
//     x_offset = 0.0f;
//     y_offset = 0.0f;
//   }
// }

// void GLVisualizer::ConvertMouseXY(int x, int y, float& m_x, float& m_y) {
//   m_x = 2.0f * (static_cast<float>(x) / static_cast<float>(w) - 0.5f);
//   m_x *= view_scale * static_cast<float>(w) / static_cast<float>(h);
//   m_x += cen_x;
//   m_y = -2.0f * (static_cast<float>(y) / static_cast<float>(h) - 0.5f);
//   m_y *= view_scale;
//   m_y += cen_y;
// }

// void GLVisualizer::handleMouseMotion(SDL_Event& e) {
//   if (adding_mode) {
//     if (add_pos_mode) {
//       ConvertMouseXY(e.motion.x, e.motion.y, virt_agent_x, virt_agent_y);
//     } else if (add_orient_mode) {
//       float m_x, m_y;
//       ConvertMouseXY(e.motion.x, e.motion.y, m_x, m_y);
//       virt_agent_th = std::atan2(m_y - virt_agent_y, m_x - virt_agent_x);
//     } else if (add_goal_mode) {
//       float m_x, m_y;
//       ConvertMouseXY(e.motion.x, e.motion.y, m_x, m_y);
//       virt_agent_gx = m_x;
//       virt_agent_gy = m_y;
//     }
//   } else if (left_mouse_down) {
//     x_offset = -2.0f * (e.motion.x - held_pos_x) / static_cast<float>(w) * view_scale * static_cast<float>(w) / static_cast<float>(h);
//     y_offset = 2.0f * (e.motion.y - held_pos_y) / static_cast<float>(h) * view_scale;
//   }
// }

// void GLVisualizer::handleKeyup(SDL_Event& e) {
//   if (e.key.keysym.sym == SDLK_a && !adding_mode) {
//     int x,y;
//     SDL_GetMouseState(&x, &y);
//     ConvertMouseXY(x, y, virt_agent_x, virt_agent_y);
//     adding_mode = true;
//     add_pos_mode = true;
//     paused = true;
//   }
//   if (e.key.keysym.sym == SDLK_SPACE && !adding_mode) {
//     paused = !paused;
//   }
//   if (e.key.keysym.sym == SDLK_r && !adding_mode) {
//     for (int i = 0; i < agents.size(); ++i) {
//       Eigen::VectorXf tmp = agents[i].goal;
//       agents[i].goal = agent_start_poses[i];
//       agent_start_poses[i] = tmp;
//     }
//   }
//   if (e.key.keysym.sym == SDLK_c && !adding_mode) {
//     agents.clear();
//     agent_start_poses.clear();
//     reset_planning_agents = true;
//     new_planning_agents.clear();
//   }
//   if (e.key.keysym.sym == SDLK_l && !adding_mode) {
//     paused = true;
//     // Get File to Load:
//     nfdchar_t *outPath = NULL;
//     nfdresult_t result = NFD_OpenDialog( "scn", NULL, &outPath );
//     if ( result == NFD_OKAY ) {
//       // Parse file and add agents:
//       std::string file_name(outPath);

//       std::vector<std::vector<std::string>> file_parsed = LoadFileByToken(file_name, 0, ',');

//       for (size_t i = 0; i < file_parsed.size(); ++i) {
//         if (!file_parsed[i].empty()) {
//           agents.emplace_back(file_parsed[i], opt_params);
//           agent_start_poses.push_back(agents.back().prob->params.x_0.head<2>());
//           new_planning_agents.emplace_back(file_parsed[i], opt_params);
//         }
//       }

//       free(outPath);
//     } else if ( result == NFD_CANCEL ) {
//       std::cout << "user cancelled load." << std::endl;
//     } else {
//       std::cerr << "Error in file load: " << NFD_GetError() << std::endl;
//     }
//   }
//   if (e.key.keysym.sym == SDLK_s && !adding_mode) {
//     paused = true;

//     nfdchar_t *savePath = NULL;
//     nfdresult_t result = NFD_SaveDialog( "scn", NULL, &savePath );
//     if ( result == NFD_OKAY ) {
//       std::string scn_file(savePath);
      
//       std::ofstream f_out(scn_file);

//       for (Agent& a : agents) {
//         f_out << a.type_name << ",";
//         f_out << (a.controlled ? "y" : "n") << ",";
//         f_out << (a.reactive ? "y" : "n");
//         Eigen::VectorXf& pos = a.prob->params.x_0;
//         for (int i = 0; i < pos.size(); ++i) {
//           f_out << "," << pos[i];
//         }
//         Eigen::VectorXf& u = a.prob->params.u_curr;
//         for (int i = 0; i < u.size(); ++i) {
//           f_out << "," << u[i];
//         }
//         for (int i = 0; i < a.goal.size(); ++i) {
//           f_out << "," << a.goal[i];
//         }
//         f_out << std::endl;
//       }

//       free(savePath);
//     } else if ( result == NFD_CANCEL ) {
//       std::cout << "user cancelled save." << std::endl;
//     } else {
//       std::cerr << "Error in file save: " << NFD_GetError() << std::endl;
//     }
//   }
// }