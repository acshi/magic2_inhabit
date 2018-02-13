subdir('general_search')

acshi_common_lib = library('acshicommon',
                           ['lcm_handle_async.c', 'binary_heap.c', 'fbinary_heap.c', 'circ_buf.c', 'median_filter.c', 'chebyshev_filter.c', 'pid_ctrl.c'],
                           c_args: '-Wfloat-conversion',
                           include_directories : inhabit_src_inc,
                           dependencies : [lcm_dep, april_common_dep, general_search_dep],
                           install : true,
                           install_dir : lib_dir)

acshi_common_dep = declare_dependency(link_with : acshi_common_lib,
                                      include_directories : inhabit_src_inc)

executable('filter_test',
         'filter_test.c',
         c_args: '-Wfloat-conversion',
         dependencies : [acshi_common_dep],
         include_directories : src_inc,
         install : true,
         install_dir : bin_dir)
