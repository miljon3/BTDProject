set(srcs "main.c"
	# "decode_jpeg.c"
	# "decode_png.c"
	# "pngle.c"
	)

idf_component_register(SRCS ${srcs} "activity.c" INCLUDE_DIRS ".")
