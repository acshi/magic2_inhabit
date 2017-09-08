project('magic2_inhabit', 'c',
        subproject_dir : '.',
        default_options : ['c_std=gnu99',
                           'default_library=static',
                           'buildtype=debug'])

compiler = meson.get_compiler('c')

magic2 = subproject('magic2')

lcm_dep = dependency('lcm')
usb_dep = dependency('libusb-1.0')
thread_dep = dependency('threads')
math_dep = compiler.find_library('m', required: false)

april_lcmtype_dep = magic2.get_variable('april_lcmtype_dep')
april_common_dep = magic2.get_variable('april_common_dep')
apriltag_dep = magic2.get_variable('apriltag_dep')
ats_dep = magic2.get_variable('ats_dep')
camera_dep = magic2.get_variable('camera_dep')
ek_dep = magic2.get_variable('ek_dep')
graph_dep = magic2.get_variable('graph_dep')
imagesource_dep = magic2.get_variable('imagesource_dep')
scanmatch_dep = magic2.get_variable('scanmatch_dep')
sim_dep = magic2.get_variable('sim_dep')
velodyne_dep = magic2.get_variable('velodyne_dep')
vx_dep = magic2.get_variable('vx_dep')

src_inc = include_directories(['src', 'magic2/src', 'magic2/april2/src'])

bin_dir = meson.source_root() + '/bin'
lib_dir = meson.source_root() + '/lib'

subdir('src')
