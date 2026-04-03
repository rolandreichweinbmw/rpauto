add_executable(rpauto
	main.cpp
)

target_link_libraries(rpauto pico_stdlib hardware_adc)
