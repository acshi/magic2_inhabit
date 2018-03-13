lcmtypes = ['waypoint_cmd_t.lcm']

lcmgen = find_program('lcm-gen')

lcmtype_dir = '../../lcmtypes/'
lcmtype_files = []
src_files = []

foreach l : lcmtypes
    file = l.split('.')
    basename = ''.join([file[0], '.c'])
    baseheader = ''.join([file[0], '.h'])
    src_files += [basename, baseheader]
    lcmtype_files += ''.join([lcmtype_dir, l])
endforeach

lcm_out = custom_target('inhabitlcm',
                        build_by_default : true,
                        output : src_files,
                        input : lcmtype_files,
                        command : [lcmgen,
                                   '--lazy',
                                   '-c',
                                   '--c-cpath=' + meson.current_source_dir(),
                                   '--c-hpath=' + meson.current_source_dir(),
                                   '--cinclude=lcmtypes/',
                                   '@INPUT@'],)

lcm_gen = generator(lcmgen,
                    output : ['@BASENAME@.c', '@BASENAME@.h'],
                    arguments : ['-c',
                                 '--c-cpath=@BUILD_DIR@',
                                 '--c-hpath=@BUILD_DIR@',
                                 '--cinclude=lcmtypes/',
                                 '@INPUT@'])

lcmtype_src = lcm_gen.process(lcmtype_files)

inhabit_lcmtype_lib = library('inhabitlcmtypes',
                            lcmtype_src,
                            dependencies : [lcm_dep],
                            include_directories : inhabit_src_inc,
                            install : true,
                            install_dir : lib_dir)

inhabit_lcmtype_dep = declare_dependency(link_with : inhabit_lcmtype_lib,
                                         include_directories : inhabit_src_inc)

lcm_py_zip = custom_target(
        'applebot_lcm_py_zip',
        build_by_default: true,
        input: lcmtype_files,
        output: 'applebot_lcmtypes.pyz',
        install: true,
        install_dir: bin_dir,
        command: [
                create_py_archive,
                '@BUILD_DIR@',
                '@OUTPUT@',
                '@INPUT@'])

