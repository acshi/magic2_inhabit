general_search_lib = library('generalsearch',
                           ['general_search.c', 'binary_heap.c', 'queues.c', 'unordered_set.c'],
                           c_args: '-Wfloat-conversion',
                           include_directories : inhabit_src_inc,
                           dependencies : [april_common_dep],
                           install : true,
                           install_dir : lib_dir)

general_search_dep = declare_dependency(link_with : general_search_lib,
                                        include_directories : inhabit_src_inc)

executable('heap_compare',
           'heap_compare.c',
           c_args: '-Wfloat-conversion',
           dependencies : [general_search_dep, april_common_dep],
           include_directories : src_inc,
           install : true,
           install_dir : bin_dir)
