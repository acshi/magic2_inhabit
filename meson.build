project('april_tag_follower', 'c',
        default_options : ['c_std=gnu99',
                           'default_library=static',
                           'buildtype=debug'])

compiler = meson.get_compiler('c')

lcm_dep = dependency('lcm')
math_dep = compiler.find_library('m', required: false)

april2 = subproject('april2')
common_dep = april2.get_variable('common_dep')

src_inc = include_directories(['src', 'magic2/april2/src'])
bin_dir = meson.source_root() + '/bin'
lib_dir = meson.source_root() + '/lib'

subdir('src')
