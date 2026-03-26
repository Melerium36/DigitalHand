#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/quaternion_transform.hpp>
#include <glm/fwd.hpp>
#include <glm/trigonometric.hpp>
#include <iostream>
#include <linux/uinput.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "utility/ShaderLoader.h"
#include "opengl_api/build_shader.h"
#include "imu_reader/Imu_reader.h"
#include "MadgwickAlg/MadgwickAHRS.h"
#include "DrawObjects/Handpart.h"
#include "callibration/callibrator.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unistd.h>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

using std::int16_t;

#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <condition_variable>



int create_virtual_keyboard() {
      int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
      ioctl(fd, UI_SET_EVBIT, EV_KEY);
      ioctl(fd, UI_SET_KEYBIT, KEY_SPACE);
      ioctl(fd, UI_SET_KEYBIT, KEY_A);
      ioctl(fd, UI_SET_KEYBIT, KEY_ENTER);

      struct uinput_setup setup{};
      std::strcpy(setup.name, "HandGlove Virtual KB");
      setup.id.bustype = BUS_USB;
      ioctl(fd, UI_DEV_SETUP, &setup);
      ioctl(fd, UI_DEV_CREATE);
      usleep(200000);
      return fd;
}

void send_key(int fd, int keycode) {
      struct input_event ev{};
      ev.type = EV_KEY;
      ev.code = keycode;
      ev.value = 1;  // Key down
      write(fd, &ev, sizeof(ev));
   
      ev.type = EV_SYN;
      ev.code = SYN_REPORT;
      ev.value = 0;
      write(fd, &ev, sizeof(ev));
      usleep(20000);
   
      ev.type = EV_KEY;
      ev.code = keycode;
      ev.value = 0;  // Key up
      write(fd, &ev, sizeof(ev));

      ev.type = EV_SYN;
      ev.code = SYN_REPORT;
      ev.value = 0;
      write(fd, &ev, sizeof(ev));
}


// struct Full_Rotation_Data_Of_System {
//     glm::quat palm;
//     glm::quat thumb_1;
//     glm::quat thumb_2;
//     glm::quat index_1;
//     glm::quat index_2;
//     glm::quat middle_1;
//     glm::quat middle_2;
//     glm::quat ring_1;
//     glm::quat ring_2;
//     glm::quat small_1;
//     glm::quat small_2;
// };

struct Sensor_State {
    Madgwick sensor_fusion;
    Datapackage total_error = {};
    Datapackage old_package_data = {};
    glm::quat latest_rotation = glm::identity<glm::quat>();
    glm::mat3 sensor_to_model_mapping = glm::mat3(1.0f);
    // glm::mat3 sensor_to_model_mapping_gyro = glm::mat3(1.0f);


    Sensor_State() {
        sensor_fusion.begin(45.5f);
    }
};


struct Sensors {
    Sensor_State palm;
    Sensor_State thumb_1;
    Sensor_State thumb_2;
    Sensor_State index_1;
    Sensor_State index_2;
    Sensor_State middle_1;
    Sensor_State middle_2;
    Sensor_State ring_1;
    Sensor_State ring_2;
    Sensor_State small_1;
    Sensor_State small_2;
};


  void init_orientation_mapping(Sensors& senors) {
      glm::mat3 mapping = glm::mat3(
          glm::vec3(1, 0, 0),
          glm::vec3(0, 1, 0),
          glm::vec3(0, 0, 1)
      );

      senors.palm.sensor_to_model_mapping = mapping;
      senors.thumb_1.sensor_to_model_mapping = mapping;
      senors.thumb_2.sensor_to_model_mapping = mapping;
      senors.index_1.sensor_to_model_mapping = mapping;
      senors.index_2.sensor_to_model_mapping = mapping;
      senors.middle_1.sensor_to_model_mapping = mapping;
      senors.middle_2.sensor_to_model_mapping = mapping;
      senors.ring_1.sensor_to_model_mapping = mapping;
      senors.ring_2.sensor_to_model_mapping = mapping;
      senors.small_1.sensor_to_model_mapping = mapping;
      senors.small_2.sensor_to_model_mapping = mapping;
  }

void draw_handpart(glm::mat4 camera, const Handpart& handpart, unsigned int transformLoc, unsigned int vertexColorLocation){
        glm::mat4 object_movement = camera * handpart.get_draw_matrix();
        // *  handpart.get_local_position() * glm::mat4_cast(handpart.get_local_rotation());
        //* handpart.get_global_complete_matrix();


        glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(object_movement));


        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glUniform4f(vertexColorLocation, 0.0f, 0.0f, 0.0f, 1.0f);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
        glUniform4f(vertexColorLocation, 0.1, 0.5, 0.1f, 1.0f);
}

void processInput(GLFWwindow *window, glm::quat& camera, const float& movement_speed)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera = glm::rotate(camera, glm::radians(static_cast<float>(movement_speed)), glm::vec3(-1,0,0));
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera = glm::rotate(camera, glm::radians(static_cast<float>(movement_speed)), glm::vec3(1,0,0));
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera = glm::rotate(camera, glm::radians(static_cast<float>(movement_speed)), glm::vec3(0,-1,0));
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera = glm::rotate(camera, glm::radians(static_cast<float>(movement_speed)), glm::vec3(0,1,0));
    camera = glm::normalize(camera);
}


IMUSample apply_deadband(IMUSample& old_values, IMUSample& new_values,size_t threshold) {
    auto apply = [&](int16_t old_v, int16_t new_v) {
        return (std::abs(new_v - old_v) < threshold) ? old_v : new_v;
    };

    return {
        apply(old_values.z, new_values.z),
        apply(old_values.y, new_values.y),
        apply(old_values.x, new_values.x)
    };
}

glm::quat get_rotation_of_sensor_data(Datapackage& package_data, Sensor_State& state_of_sensor) {

        package_data.Gyro_data = package_data.Gyro_data - state_of_sensor.total_error.Gyro_data;
        package_data.Accel_data = apply_deadband(state_of_sensor.old_package_data.Accel_data, package_data.Accel_data, 10);

        state_of_sensor.old_package_data.Accel_data = package_data.Accel_data;

        // Endlich richtige Mappings gefunden!!!
        // float gx_m = (package_data.Gyro_data.x/32.8f);
        // float gy_m = (package_data.Gyro_data.z/32.8f);
        // float gz_m = -(package_data.Gyro_data.y/32.8f);
        //
        // float ax_m = package_data.Accel_data.z;
        // float ay_m = package_data.Accel_data.x;
        // float az_m = package_data.Accel_data.y;


        glm::vec3 mapped_gyro_data = state_of_sensor.sensor_to_model_mapping * glm::vec3(package_data.Gyro_data.x/32.8f,
                                                                                         package_data.Gyro_data.y/32.8f,
                                                                                         package_data.Gyro_data.z/32.8f);

        glm::vec3 mapped_accel_data = state_of_sensor.sensor_to_model_mapping * glm::vec3(package_data.Accel_data.x,
                                                                                          package_data.Accel_data.y,
                                                                                          package_data.Accel_data.z);

        float accel_norm =  glm::length(mapped_accel_data);
        float one_g = 16384.0f;
        float deviation = std::fabs(accel_norm - one_g) / one_g;
// 0.2f * (1.0f-deviation)
        state_of_sensor.sensor_fusion.updateIMU(mapped_gyro_data.x,
                                                mapped_gyro_data.y,
                                                mapped_gyro_data.z,
                                                mapped_accel_data.x,
                                                mapped_accel_data.y,
                                                mapped_accel_data.z,
                                                0.2 * (1.0f-deviation));

        glm::quat q(state_of_sensor.sensor_fusion.q0,
                    state_of_sensor.sensor_fusion.q1,
                    state_of_sensor.sensor_fusion.q2,
                    state_of_sensor.sensor_fusion.q3);

        q = glm::normalize(q);
        return q;
}

void imu_worker(Imu_reader& imu_reader, std::atomic<bool>& running, bool& ready_to_give_data, std::condition_variable& cv, Sensors& sensors, std::mutex& imu_mutex) {



    Callibrator callibrator = Callibrator(imu_reader, 100, 50, 2);
    callibrator.calc_error();
    
    sensors.small_2.total_error  = callibrator.get_error_value_by_identifier(0x00); // TODO: Enumarate für die Fehler einmal global definieren
    sensors.small_1.total_error  = callibrator.get_error_value_by_identifier(0x01);
    sensors.ring_2.total_error   = callibrator.get_error_value_by_identifier(0x10);
    sensors.ring_1.total_error   = callibrator.get_error_value_by_identifier(0x11);
    sensors.middle_2.total_error = callibrator.get_error_value_by_identifier(0x20);
    sensors.middle_1.total_error = callibrator.get_error_value_by_identifier(0x21);
    sensors.index_2.total_error  = callibrator.get_error_value_by_identifier(0x30);
    sensors.index_1.total_error  = callibrator.get_error_value_by_identifier(0x31);
    sensors.thumb_2.total_error  = callibrator.get_error_value_by_identifier(0x40);
    sensors.thumb_1.total_error  = callibrator.get_error_value_by_identifier(0x41);
    sensors.palm.total_error     = callibrator.get_error_value_by_identifier(0x50);

    int fd_virtual_keyboard = create_virtual_keyboard();
    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        ready_to_give_data = true;
    }
    cv.notify_one();

    int cooldown = 45 * 11;
    while (running.load()) {
        Datapackage package_data = imu_reader.read_next_record();
        // std::cout << package_data.Gyro_data.x << "\n";
        // std::cout << package_data.Gyro_data.y << "\n";
        // std::cout << package_data.Gyro_data.z << "\n";



        std::lock_guard<std::mutex> lock(imu_mutex);
        switch (package_data.Identifier) { // TODO: sensor Struct als Klasse mit thread Guard
            case 0x00:
                sensors.small_2.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.small_2);
                break;

            case 0x01:
                sensors.small_1.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.small_1);
                break;

            case 0x10:
                sensors.ring_2.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.ring_2);
                break;

            case 0x11:
                sensors.ring_1.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.ring_1);
                break;

            case 0x20:
                sensors.middle_2.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.middle_2);
                break;

            case 0x21:
                sensors.middle_1.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.middle_1);
                break;

            case 0x30:
                sensors.index_2.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.index_2);
                break;

            case 0x31:
                sensors.index_1.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.index_1);
                break;

            case 0x40:
                sensors.thumb_2.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.thumb_2);
                break;

            case 0x41:
                sensors.thumb_1.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.thumb_1);
                break;

            case 0x50: {
                sensors.palm.latest_rotation = get_rotation_of_sensor_data(package_data, sensors.palm);
                float norm = sqrt((float)(package_data.Accel_data.x * package_data.Accel_data.x +
                                                 package_data.Accel_data.y * package_data.Accel_data.y +
                                                 package_data.Accel_data.z * package_data.Accel_data.z));
                if (cooldown <= 2) {
                    cooldown = 2;
                    bool trigger = norm < 12500.0f;
                    if (trigger) {
                        std::cout << "Taste Druck \n";
                        send_key(fd_virtual_keyboard, KEY_ENTER);
                        cooldown = 45 * 11;
                    }
                }
                break;
            }

            default:
                throw std::runtime_error("Failed to identify Datasource");
                break;
        }
        cooldown--;
    }
      ioctl(fd_virtual_keyboard, UI_DEV_DESTROY);
      close(fd_virtual_keyboard);  
}



int main(){
    if (!glfwInit()) {
        std::cerr << "glfwInit() failed\n";
        return 1;
    }


    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "Digital Hand III", nullptr, nullptr);
    if (!window) {
        std::cerr << "glfwCreateWindow() failed\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "gladLoadGLLoader() failed\n";
        glfwTerminate();
        return 1;
    }

    std::cout << "OpenGL: " << glGetString(GL_VERSION) << "\n";
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << "\n";

    glViewport(0, 0, 800, 600);

float vertices[] = { // Eckpunkte eines Würfels
    -0.5f, -0.5f, -0.5f,  // 0
     0.5f, -0.5f, -0.5f,  // 1
     0.5f,  0.5f, -0.5f,  // 2
    -0.5f,  0.5f, -0.5f,  // 3
    -0.5f, -0.5f,  0.5f,  // 4
     0.5f, -0.5f,  0.5f,  // 5
     0.5f,  0.5f,  0.5f,  // 6
    -0.5f,  0.5f,  0.5f   // 7
};


    unsigned int indices[] = {
        // Rückseite
        0, 1, 2,
        2, 3, 0,

        // Vorderseite
        4, 5, 6,
        6, 7, 4,

        // Linke Seite
        0, 3, 7,
        7, 4, 0,

        // Rechte Seite
        1, 5, 6,
        6, 2, 1,

        // Unterseite
        0, 1, 5,
        5, 4, 0,

        // Oberseite
        3, 2, 6,
        6, 7, 3
    };

    unsigned int VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);


    Shader_loader shader_loader = Shader_loader("./src/res/").source_vertex("vertex_shader.glsl").source_fragment("fragment_shader.glsl");
    unsigned int vertex_shader = load_vertex_shader_into_state(shader_loader);
    unsigned int fragment_shader = load_fragment_shader_into_state(shader_loader);


    unsigned int shaderProgramm; // Als Objekt verlagern, vielleicht nach Shader_laoder
    shaderProgramm = glCreateProgram();
    glAttachShader(shaderProgramm, vertex_shader);
    glAttachShader(shaderProgramm, fragment_shader);
    glLinkProgram(shaderProgramm);
    glUseProgram(shaderProgramm);








    unsigned int VAO;
    glGenVertexArrays(1, &VAO); 
    glBindVertexArray(VAO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    


    unsigned int EBO;
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW); // schauen was es mit den Buffer auf sich hat reihenfolge



    glEnable(GL_DEPTH_TEST);

    // glEnable(GL_DEBUG_OUTPUT);
    // glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

    glfwSwapInterval(1); // Vsync

    std::mutex imu_mutex;
    std::atomic<bool> running{true};
    bool ready_to_give_data = false;
    std::condition_variable imu_cv;

    Imu_reader imu_reader = Imu_reader("/dev/ttyACM0", 115200);
    Sensors sensors = {};
    Sensors sensors_save_copy = {};
 
    init_orientation_mapping(sensors);

    std::thread imu_thread(
        imu_worker,
        std::ref(imu_reader),
        std::ref(running),
        std::ref(ready_to_give_data),
        std::ref(imu_cv),
        std::ref(sensors),
        std::ref(imu_mutex)
    );

    {
        std::unique_lock<std::mutex> lock(imu_mutex);
        imu_cv.wait(lock, [&]() { return ready_to_give_data; });
    }

 
 


    // auto start = std::chrono::steady_clock::now();
    // while (1) {
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
    //     std::cout << duration.count() << "\n";
    //     start = std::chrono::steady_clock::now();
    //     IMUSample gyro = imu_reader.read_next_record().Gyro_data;
    //     IMUSample accel = imu_reader.read_next_record().Accel_data;
    //
    //     // std::cout << "GYRO:";
    //     // std::cout << gyro.x << "\n";
    //     // std::cout << gyro.y << "\n";
    //     // std::cout << gyro.z << "\n";
    //     // std::cout << "\n";
    //     // std::cout << "ACCEL:";
    //     // std::cout << accel.x << "\n";
    //     // std::cout << accel.y << "\n";
    //     // std::cout << accel.z << "\n";
    //     // std::cout << "\n";
    // }


    Constraints finger_contraint = {{false, glm::radians(-10.0), glm::radians(95.0)}, // Pitch
                                    // {true, 0.0, 0.0},
                                    {true, 0.0, 0.0}, // Roll
                                    {false, glm::radians(-30.0), glm::radians(30.0)} // Yaw
                                   };

    Constraints second_finger_contraint = {{false, glm::radians(-5.0), glm::radians(100.0)},
                                          {true, 0.0, 0.0},
                                          {true, 0.0, 0.0}};

    Constraints palm_contraint =          {{false, -1.66, 1.66},
                                          {false, -1.0, 1.00},
                                          {}};

    glm::quat camera = glm::identity<glm::quat>();

    Handpart palm = Handpart(glm::vec3(0.4,0.5,0.1), glm::vec3(0.0,0.0,0.0), &sensors_save_copy.palm.latest_rotation, palm_contraint);

    Handpart first_index = Handpart(glm::vec3(0.09, 0.5 * 0.45, 0.1), glm::vec3(-0.15,0.37,0.0), &sensors_save_copy.index_1.latest_rotation, finger_contraint ,&palm);
    Handpart first_middle = Handpart(glm::vec3(0.09, 0.5 * 0.5, 0.1), glm::vec3(-0.05,0.38,0.0), &sensors_save_copy.middle_1.latest_rotation, finger_contraint ,&palm);
    Handpart first_ring = Handpart(glm::vec3(0.09, 0.5 * 0.45, 0.1), glm::vec3(0.05,0.36,0.0), &sensors_save_copy.ring_1.latest_rotation, finger_contraint , &palm);
    Handpart first_small = Handpart(glm::vec3(0.09, 0.5 * 0.3, 0.1), glm::vec3(0.15,0.33,0.0), &sensors_save_copy.small_1.latest_rotation, finger_contraint , &palm);
    Handpart first_thumb = Handpart(glm::vec3(0.09, 0.8 * 0.35, 0.1), glm::vec3(-0.25,-0.1,0.0), &sensors_save_copy.thumb_1.latest_rotation, {} , &palm); // Daumen kann rollen

    Handpart second_index = Handpart(glm::vec3(0.09, 0.5 * 0.45, 0.1), glm::vec3(0.0,0.225,0.0), &sensors_save_copy.index_2.latest_rotation, second_finger_contraint , &first_index);
    Handpart second_middle = Handpart(glm::vec3(0.09, 0.5 * 0.5, 0.1), glm::vec3(0.0,0.25,0.0), &sensors_save_copy.middle_2.latest_rotation, second_finger_contraint , &first_middle);
    Handpart second_ring = Handpart(glm::vec3(0.09, 0.5 * 0.45, 0.1), glm::vec3(0.0,0.225,0.0), &sensors_save_copy.ring_2.latest_rotation, second_finger_contraint , &first_ring);
    Handpart second_small = Handpart(glm::vec3(0.09, 0.5 * 0.3, 0.1), glm::vec3(0.0,0.15,0.0), &sensors_save_copy.small_2.latest_rotation, second_finger_contraint ,&first_small);
    Handpart second_thumb = Handpart(glm::vec3(0.09, 0.8 * 0.2, 0.1), glm::vec3(0.0,0.225,0.0), &sensors_save_copy.thumb_2.latest_rotation, second_finger_contraint ,&first_thumb);

    Handpart third_index = Handpart(glm::vec3(0.09, 0.2 * 0.45, 0.1), glm::vec3(0.0,0.16,0.0), nullptr, {} ,&second_index);
    Handpart third_middle = Handpart(glm::vec3(0.09, 0.2 * 0.45, 0.1), glm::vec3(0.0,0.175,0.0), nullptr, {} ,&second_middle);
    Handpart third_ring = Handpart(glm::vec3(0.09, 0.2 * 0.45, 0.1), glm::vec3(0.0,0.16,0.0), nullptr, {} ,&second_ring);
    Handpart third_small = Handpart(glm::vec3(0.09, 0.2 * 0.45, 0.1), glm::vec3(0.0,0.125,0.0), nullptr, {} ,&second_small);



    unsigned int transformLoc = glGetUniformLocation(shaderProgramm, "transform");
    int vertexColorLocation = glGetUniformLocation(shaderProgramm, "ourColor");

    while (!glfwWindowShouldClose(window)) {
        processInput(window, camera, 1);

        {
            std::unique_lock<std::mutex> lock(imu_mutex);
            sensors_save_copy = sensors;
        }


        // std::cout << sensors_save_copy.palm.latest_rotation << "\n";

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        glUniform4f(vertexColorLocation, 0.1, 0.8, 0.0f, 1.0f);

        // Objects
        draw_handpart(glm::mat4_cast(camera), palm, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), first_index, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), first_middle, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), first_ring, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), first_small, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), first_thumb, transformLoc, vertexColorLocation);

        draw_handpart(glm::mat4_cast(camera), second_index, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), second_middle, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), second_ring, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), second_small, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), second_thumb, transformLoc, vertexColorLocation);

        draw_handpart(glm::mat4_cast(camera), third_index, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), third_middle, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), third_ring, transformLoc, vertexColorLocation);
        draw_handpart(glm::mat4_cast(camera), third_small, transformLoc, vertexColorLocation);


        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    glDeleteProgram(shaderProgramm);
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    running = false;
    imu_reader.close_device();
    imu_thread.join();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
