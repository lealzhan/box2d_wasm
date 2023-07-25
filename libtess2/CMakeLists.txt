set( LIBNAME libtess2 )

add_library( ${LIBNAME} STATIC
	Source/bucketalloc.c
	Source/dict.c
	Source/geom.c
	Source/mesh.c
	Source/priorityq.c
	Source/sweep.c
	Source/tess.c
)

target_include_directories( ${LIBNAME}
	PUBLIC
	Include
)