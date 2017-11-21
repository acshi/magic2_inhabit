acshi_common_lib = library('acshicommon',
                           'lcm_handle_async.c',
                           c_args: '-Wfloat-conversion',
                           include_directories : inhabit_src_inc,
                           dependencies : [lcm_dep],
                           install : true,
                           install_dir : lib_dir)

acshi_common_dep = declare_dependency(link_with : acshi_common_lib,
                                      include_directories : inhabit_src_inc)
