executable('inhabit',
           ['inhabit.c', 'lcm_handle_async.c'],
           c_args: '-Wfloat-conversion',
           dependencies : [lcm_dep, magic_lcmtype_dep, math_dep, april_common_dep, magic_common_dep],
           include_directories : src_inc,
           install : true,
           install_dir : bin_dir)
