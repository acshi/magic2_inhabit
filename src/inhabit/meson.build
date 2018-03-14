executable('inhabit',
           'inhabit.c',
           c_args: '-Wfloat-conversion',
           dependencies : [applebot_lcmtype_dep, applebot_common_dep, lcm_dep, magic_lcmtype_dep, math_dep, april_common_dep, magic_common_dep, scanmatch_dep],
           include_directories : src_inc,
           install : true,
           install_dir : bin_dir)
