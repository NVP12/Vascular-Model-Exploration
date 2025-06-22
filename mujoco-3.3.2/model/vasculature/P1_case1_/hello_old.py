import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
# import cv2
print(f"MuJoCo version: {mj.__version__}")
xml_path = 'hello.xml' #xml file (assumes this is in the same folder as this file)
simend = 20 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)
print_model = 0 #set to 1 to print the model info in the file model.txt in the current location

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# ——————————————————————————————————————————————————————
# 2) Add a small “step” size and share the offscreen_cam as a global:
CAM_STEP = 0.01
ANG_STEP = 1.0 # Degrees per press

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    global offscreen_cam

    # only act on PRESS or REPEAT
    if act not in (glfw.PRESS, glfw.REPEAT):
        return

    # Move the lookat point in the camera’s local frame:
    # Compute forward/right vectors from azimuth & elevation:
    az  = np.deg2rad(offscreen_cam.azimuth)
    el  = np.deg2rad(offscreen_cam.elevation)
    # Forward in the horizontal plane
    forward = np.array([
        np.cos(az), 
        np.sin(az), 
        0.0
    ])

    # Right is +90° yaw from forward
    right = np.array([-forward[1], forward[0], 0.0])
    up    = np.array([0.0, 0.0, 1.0])

    # Calculate camera's actual forward, right, and up vectors in global coordinates
    # These vectors define the camera's current orientation in 3D space.
    # Camera's "forward" vector (points *into* the scene from the camera's view)
    cam_forward_x = np.cos(az) * np.cos(el)
    cam_forward_y = np.sin(az) * np.cos(el)
    cam_forward_z = -np.sin(el) # MuJoCo's Z is up, so elevation pitches towards/away from Z
    cam_forward_vec = np.array([cam_forward_x, cam_forward_y, cam_forward_z])
    cam_forward_vec /= np.linalg.norm(cam_forward_vec) # Normalize for safety

    # Camera's "right" vector (local X-axis) - perpendicular to global Z
    cam_right_x = -np.sin(az)
    cam_right_y = np.cos(az)
    cam_right_z = 0.0
    cam_right_vec = np.array([cam_right_x, cam_right_y, cam_right_z])
    cam_right_vec /= np.linalg.norm(cam_right_vec) # Normalize

    # Camera's "up" vector (local Y-axis) - cross product of right and forward
    # This correctly accounts for pitch, so it's the camera's true 'up' direction
    cam_up_vec = np.cross(cam_right_vec, cam_forward_vec)
    cam_up_vec /= np.linalg.norm(cam_up_vec) # Normalize

    # -- yaw --
    if key == glfw.KEY_RIGHT:
        offscreen_cam.azimuth -= ANG_STEP
    elif key == glfw.KEY_LEFT:
        offscreen_cam.azimuth += ANG_STEP

    # -- pitch --
    elif key == glfw.KEY_UP:
        offscreen_cam.elevation -= ANG_STEP

    elif key == glfw.KEY_DOWN:
        offscreen_cam.elevation += ANG_STEP

    # -- move-forward/backward --
    elif key == glfw.KEY_W:
        offscreen_cam.lookat +=  CAM_STEP * cam_forward_vec
    elif key == glfw.KEY_S:
        offscreen_cam.lookat -=  CAM_STEP * cam_forward_vec
    
    # --move-right/left --
    elif key == glfw.KEY_A:
        offscreen_cam.lookat +=  CAM_STEP * cam_right_vec
    elif key == glfw.KEY_D:
        offscreen_cam.lookat -=  CAM_STEP * cam_right_vec

    # --move-up/down --
    elif key == glfw.KEY_Q:
        offscreen_cam.lookat +=  CAM_STEP * cam_up_vec
    elif key == glfw.KEY_E:
        offscreen_cam.lookat -=  CAM_STEP * cam_up_vec

    # (Optional) zoom in/out
    elif key == glfw.KEY_Z:
        offscreen_cam.distance = max(0.1, offscreen_cam.distance - CAM_STEP)
    elif key == glfw.KEY_X:
        offscreen_cam.distance += CAM_STEP

    # don’t forget your existing BACKSPACE reset, etc.
    elif key == glfw.KEY_BACKSPACE and act == glfw.PRESS:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

    print("Current Look At Point: ",offscreen_cam.lookat)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

#print the model
if (print_model==1):
    model_name = 'model.txt'
    model_path = os.path.join(dirname + "/" + model_name)
    mj.mj_printModel(model,model_path)


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera

opt = mj.MjvOption()                        # visualization options

# The line below is to enable the Headlight that follows the camera ---
# model.flags[mj.mjtRndFlag.mjRND_LIGHT_HEAD] = True

# Display Camera Frame in original world view
opt.frame = mj.mjtFrame.mjFRAME_CAMERA

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# The line below is to avoid back-culling so that the inner surface of the vascular model is visible while we are navigating.
scene.flags[mj.mjtRndFlag.mjRND_CULL_FACE.value] = False 

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration

# Main Window View
cam.type = mj.mjtCamera.mjCAMERA_FREE # Can still be free if you want to set it initially
cam.azimuth = 60                     # Example: Choose an angle
cam.elevation = -25                  # Example: Look down slightly
cam.distance = 4.0                   # Example: Zoom out to see the area
cam.lookat = np.array([0.0, -1.0, 1.0]) # Example: Center your view appropriately


# camera_name = 'drone_camera'
# camera_id   = model.camera(camera_name).id

# 1) When you create your offscreen_cam, make it FREE:
offscreen_cam = mj.MjvCamera()
offscreen_cam.type       = mj.mjtCamera.mjCAMERA_FREE
offscreen_cam.distance   = 0.01    # starting zoom
offscreen_cam.lookat     = np.array([0.0, -0.5, 1.1])
offscreen_cam.azimuth    = 90.0   # starting orientation
offscreen_cam.elevation  = 0.0
# offscreen_cam.znear = 1e-4
width = 0.7*640
height = 0.7*480


#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

while not glfw.window_should_close(window):
    time_prev = data.time

    # while (data.time - time_prev < 1.0/60.0):
    #     #mj.mj_step(model, data)
    #     qz = 0.25*np.sin(data.time);
    #     q0 = np.sqrt(1-qz*qz)
    #     quat = np.array([q0,0,0,qz])
    #     data.time += model.opt.timestep
    #     data.qpos[3:] = quat.copy()
    #     mj.mj_forward(model,data)
    mj.mj_forward(model, data)

    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # Code taken from https://github.com/dtorre38/mujoco_opencv
    # ******** inset view (code start) *********
    # Settings for inset view
    
    loc_x=int(viewport_width - width)
    loc_y=int(viewport_height - height)
    height = int(height)
    width = int(width)

    # 1. Create a rectangular viewport in the upper right corner for example.
    offscreen_viewport = mj.MjrRect(loc_x, loc_y, width, height)

    # inset viewport already computed as offscreen_viewport
    mj.mjv_updateScene(
        model, data, opt, None,
        offscreen_cam, mj.mjtCatBit.mjCAT_ALL.value,
        scene
    )
    mj.mjr_render(offscreen_viewport, scene, context)

    # ******** inset view (code end) *********


    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()