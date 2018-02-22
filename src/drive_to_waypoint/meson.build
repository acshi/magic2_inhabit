executable('drive-to-waypoint',
           ['drive_to_waypoint.c', 'vfh_star.c', 'collision.c', 'gui.c'],
           c_args: ['-Wfloat-conversion', '-Werror', '-O0', '-ftree-vectorize'],
           dependencies : [inhabit_lcmtype_dep, acshi_common_dep, lcm_dep, magic_lcmtype_dep, math_dep, april_common_dep, magic_common_dep, vx_dep],
           include_directories : src_inc,
           install : true,
           install_dir : bin_dir)
