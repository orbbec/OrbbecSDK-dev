cmake_minimum_required(VERSION 3.1.15)

add_subdirectory(src)

get_target_property(compile_options turbojpeg_static COMPILE_OPTIONS)
list(REMOVE_ITEM compile_options "-Werror" "/WX")
set_target_properties(turbojpeg_static PROPERTIES COMPILE_OPTIONS "${compile_options}")

get_target_property(compile_options jpeg_static COMPILE_OPTIONS)
list(REMOVE_ITEM compile_options "-Werror" "/WX")
set_target_properties(jpeg_static PROPERTIES COMPILE_OPTIONS "${compile_options}")

get_target_property(compile_options simd COMPILE_OPTIONS)
list(REMOVE_ITEM compile_options "-Werror" "/WX")
set_target_properties(simd PROPERTIES COMPILE_OPTIONS "${compile_options}")

get_target_property(compile_options tjbench-static COMPILE_OPTIONS)
list(REMOVE_ITEM compile_options "-Werror" "/WX")
set_target_properties(tjbench-static PROPERTIES COMPILE_OPTIONS "${compile_options}")

add_library(libjpeg::libjpeg ALIAS turbojpeg_static)
ob_source_group(libjpeg::libjpeg)
set_target_properties(turbojpeg_static jpeg_static simd croptest testclean tjbench-static tjtest PROPERTIES FOLDER "dependencies")