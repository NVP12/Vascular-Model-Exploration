import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
# import cv2 # Keep if you use the OpenCV part

# --- XML Path ---
# Ensure this points to your MODIFIED XML file
xml_path = 'P11_case9_.xml' # Or your vasculature/hello.xml if that's the one you modified

# --- Constants for Control ---
LINEAR_VEL_STEP = 0.3  # m/s per key press action
ANGULAR_VEL_STEP = 0.8 # rad/s per key press action

# --- Global variables for drone control and IDs ---
# These will be initialized in init_controller
drone_body_id = -1
# For a free joint, qpos starts at model.jnt_qposadr[joint_id] (7 values)
# qvel starts at model.jnt_dofadr[joint_id] (6 values)
drone_joint_qpos_addr = -1
drone_joint_dof_addr = -1 # This is for qvel

# Global desired velocities (local to drone body's frame)
g_desired_local_linear_velocity = np.zeros(3)  # [vx, vy, vz] in body frame
g_desired_local_angular_velocity = np.zeros(3) # [wx, wy, wz] in body frame (roll, pitch, yaw rates)

simend = 20 # simulation time
print_camera_config = 0 # set to 1 to print camera config
                        # this is useful for initializing view of the model)
print_model = 0 #set to 1 to print the model info in the file model.txt in the current location

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model,data):
    global drone_body_id, drone_joint_qpos_addr, drone_joint_dof_addr

    # Name of the body with the free joint in your XML
    piloted_body_name = "drone_piloted_body" 
    drone_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, piloted_body_name)
    if drone_body_id == -1:
        raise ValueError(f"Body '{piloted_body_name}' not found in XML. Did you name it correctly and add a free joint?")

    # Name of the free joint in your XML
    free_joint_name = "drone_free_joint" 
    joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, free_joint_name)
    if joint_id == -1:
        raise ValueError(f"Joint '{free_joint_name}' not found in XML.")
    
    if model.jnt_type[joint_id] != mj.mjtJoint.mjJNT_FREE:
        raise ValueError(f"Joint '{free_joint_name}' is not a free joint. Body cannot be controlled this way.")

    drone_joint_qpos_addr = model.jnt_qposadr[joint_id]
    drone_joint_dof_addr = model.jnt_dofadr[joint_id] # Start index in data.qvel for this joint's 6 DoFs

    print(f"Drone control initialized for body: '{piloted_body_name}' (ID: {drone_body_id})")
    print(f"  QPos Address: {drone_joint_qpos_addr}, QVel (DoF) Address: {drone_joint_dof_addr}")


def controller(model, data):
    # This controller callback is called by mj_step.
    # We will set data.qvel directly based on keyboard input.
    # If you were using actuators (e.g., data.ctrl), you'd set them here.
    pass

def keyboard(window, key, scancode, act, mods):
    global g_desired_local_linear_velocity, g_desired_local_angular_velocity

    # Reset simulation
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        # Reset desired velocities
        g_desired_local_linear_velocity[:] = 0.0
        g_desired_local_angular_velocity[:] = 0.0
        # Explicitly reset actual qvel of the drone too if addresses are valid
        if drone_joint_dof_addr != -1:
            data.qvel[drone_joint_dof_addr : drone_joint_dof_addr+6] = 0.0
        mj.mj_forward(model, data) # Update kinematics after reset
        return

    # Determine if it's a key press or release for movement keys
    is_press_or_repeat = (act == glfw.PRESS or act == glfw.REPEAT)
    is_release = (act == glfw.RELEASE)

    # --- Linear Movement (WASDQE) ---
    # W: Forward (body +X), S: Backward (body -X)
    if key == glfw.KEY_D:
        g_desired_local_linear_velocity[0] = LINEAR_VEL_STEP if is_press_or_repeat else 0.0
    elif key == glfw.KEY_A:
        g_desired_local_linear_velocity[0] = -LINEAR_VEL_STEP if is_press_or_repeat else 0.0
    
    # A: Left (body +Y), D: Right (body -Y)
    # Assuming body +Y is left. If body +Y is right, swap A/D signs.
    elif key == glfw.KEY_W:
        g_desired_local_linear_velocity[1] = LINEAR_VEL_STEP if is_press_or_repeat else 0.0
    elif key == glfw.KEY_S:
        g_desired_local_linear_velocity[1] = -LINEAR_VEL_STEP if is_press_or_repeat else 0.0

    # Q: Up (body +Z), E: Down (body -Z)
    elif key == glfw.KEY_Q:
        g_desired_local_linear_velocity[2] = LINEAR_VEL_STEP if is_press_or_repeat else 0.0
    elif key == glfw.KEY_E:
        g_desired_local_linear_velocity[2] = -LINEAR_VEL_STEP if is_press_or_repeat else 0.0

    # --- Angular Movement (Arrow Keys for Yaw, potentially others for Pitch/Roll) ---
    # LEFT/RIGHT Arrow: Yaw around body Z-axis
    elif key == glfw.KEY_LEFT:
        g_desired_local_angular_velocity[2] = ANGULAR_VEL_STEP if is_press_or_repeat else 0.0
    elif key == glfw.KEY_RIGHT:
        g_desired_local_angular_velocity[2] = -ANGULAR_VEL_STEP if is_press_or_repeat else 0.0
    
    # UP/DOWN Arrow: Pitch around body Y-axis (Optional)
    elif key == glfw.KEY_UP:
         g_desired_local_angular_velocity[0] = -ANGULAR_VEL_STEP if is_press_or_repeat else 0.0
    elif key == glfw.KEY_DOWN:
         g_desired_local_angular_velocity[0] = ANGULAR_VEL_STEP if is_press_or_repeat else 0.0

    # Add keys for Roll around body X-axis if needed (e.g., PageUp/PageDown)


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

# --- Path and Model Loading ---
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path) # Corrected path joining
xml_path = abspath

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera() # Main world view camera
opt_main_view = mj.MjvOption()
opt_main_view.frame = mj.mjtFrame.mjFRAME_BODY # To display frames

# --- GLFW and MuJoCo Viz Init ---
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
# Example: Set a fixed position for the main world camera 'cam'
cam.azimuth = 45
cam.elevation = -30
cam.distance = 10.0
cam.lookat = np.array([0.0, 0.0, 3]) # Adjust to view your scene well

# mj.mjv_defaultOption(opt)
# If you want to see frames of XML cameras automatically in the main view:
# opt.frame = mj.mjtFrame.mjFRAME_CAMERA # Or mjFRAME_ALL

scene = mj.MjvScene(model, maxgeom=10000)
scene.flags[mj.mjtRndFlag.mjRND_CULL_FACE.value] = False # to avoid back-face culling
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

init_controller(model,data) # Initialize drone body/joint IDs
mj.set_mjcb_control(controller) # Set the (currently empty) controller callback

# --- FPV Inset Camera (Abstract MjvCamera configured to use an XML camera) ---
# This can be created once if its fixedcamid doesn't change.
fpv_inset_camera_viewer = mj.MjvCamera()
fpv_inset_camera_name_xml = "drone_camera" # MUST MATCH NAME IN YOUR XML
xml_camera_id_for_fpv = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, fpv_inset_camera_name_xml)
opt_fpv_inset = mj.MjvOption()
mj.mjv_defaultOption(opt_fpv_inset) # Load default settings

#Inset Window Size
width = 0.7*640
height = 0.7*480

if xml_camera_id_for_fpv == -1:
    print(f"Warning: FPV camera '{fpv_inset_camera_name_xml}' not found in XML. Inset view will not work correctly.")
else:
    fpv_inset_camera_viewer.type = mj.mjtCamera.mjCAMERA_FIXED
    fpv_inset_camera_viewer.fixedcamid = xml_camera_id_for_fpv

# --- Main Loop ---
while not glfw.window_should_close(window):
    time_prev = data.time

    # Apply controls to the drone body by setting its qvel
    if drone_body_id != -1 and drone_joint_dof_addr != -1:
        # Get current orientation of the drone body
        drone_world_orientation_matrix = data.xmat[drone_body_id].reshape(3,3)
        
        # Transform desired local velocities to world frame
        world_linear_velocity = drone_world_orientation_matrix @ g_desired_local_linear_velocity
        world_angular_velocity = drone_world_orientation_matrix @ g_desired_local_angular_velocity
        
        # Set the qvel for the free joint
        data.qvel[drone_joint_dof_addr : drone_joint_dof_addr+3] = world_linear_velocity
        data.qvel[drone_joint_dof_addr+3 : drone_joint_dof_addr+6] = world_angular_velocity

    # --- Physics Step ---
    # This loop ensures simulation advances roughly in real-time if rendering is at 60fps
    # And allows for multiple physics steps per render frame if needed for stability.
    step_start_time = data.time
    while (data.time - step_start_time < 1.0/60.0): # Target 60Hz render rate
        mj.mj_step(model, data) # Critical: steps the simulation, integrates qvel
        # if data.time >= simend:
        #     break
    
    # if (data.time>=simend):
    #     break;

    # --- Rendering ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    main_viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    if (print_camera_config==1):
        # (your print camera config code for main 'cam')
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')


    # Update main scene (world view using 'cam') and render
    # If opt.frame includes mjFRAME_CAMERA, the frame of "drone_camera" (if it moves) will be shown
    mj.mjv_updateScene(model, data, opt_main_view, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(main_viewport, scene, context)

    # --- Inset FPV View ---
    # Define inset view dimensions (using your original variables)
    # User's original code for inset size had global 'width' and 'height'
    # Let's assume 'width' and 'height' are defined as they were in your script (0.4*640 etc.)
    # To avoid confusion, let's use specific names here:
    inset_width_px = int(width)  # From your original code
    inset_height_px = int(height) # From your original code

    loc_x = int(viewport_width - inset_width_px)
    loc_y = int(viewport_height - inset_height_px) # For top-right corner
    
    inset_viewport = mj.MjrRect(loc_x, loc_y, inset_width_px, inset_height_px)

    if xml_camera_id_for_fpv != -1: # Render FPV only if camera was found
        mj.mjv_updateScene(model, data, opt_fpv_inset, None, fpv_inset_camera_viewer,
                           mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(inset_viewport, scene, context)

    # # --- OpenCV Pixel Reading (Your existing code) ---
    # # Ensure inset_height_px and inset_width_px are used here
    # if cv2: # Check if cv2 was successfully imported
    #     try:
    #         rgb_pixels = np.zeros((inset_height_px, inset_width_px, 3), dtype=np.uint8)
    #         depth_pixels = np.zeros((inset_height_px, inset_width_px), dtype=np.float32) # Corrected variable name
    #         mj.mjr_readPixels(rgb_pixels, depth_pixels, inset_viewport, context) # Corrected variable name

    #         gray_pixels = np.dot(rgb_pixels[..., :3], [0.2989, 0.5870, 0.1140])
    #         bw_pixels = np.stack([gray_pixels] * 3, axis=-1).astype(np.uint8)
    #         bw_pixels_flipped = cv2.flip(bw_pixels, 0)
    #         bw_image_bgr = cv2.cvtColor(bw_pixels_flipped, cv2.COLOR_RGB2BGR)
    #         cv2.imshow("FPV Grayscale (OpenCV)", bw_image_bgr)
    #         cv2.waitKey(1)
    #     except Exception as e:
    #         print(f"OpenCV processing error: {e}")
    #         # You might want to disable OpenCV part if it keeps failing by setting a flag


    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
