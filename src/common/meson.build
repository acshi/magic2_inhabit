subdir('general_search')

applebot_common_lib = library('applebotcommon',
                           ['lcm_handle_async.c', 'binary_heap.c', 'fbinary_heap.c', 'circ_buf.c', 'moving_filter.c', 'median_filter.c', 'chebyshev_filter.c', 'pid_ctrl.c', 'velo_to_points.c', 'velo_to_points.h'],
                           c_args: '-Wfloat-conversion',
                           include_directories : applebot_src_inc,
                           dependencies : [lcm_dep, april_common_dep, general_search_dep],
                           install : true,
                           install_dir : lib_dir)

applebot_common_dep = declare_dependency(link_with : applebot_common_lib,
                                      include_directories : applebot_src_inc)

executable('filter_test',
         'filter_test.c',
         c_args: '-Wfloat-conversion',
         dependencies : [applebot_common_dep],
         include_directories : src_inc,
         install : true,
         install_dir : bin_dir)
