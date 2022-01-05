CMAKE_ARGS:=$(CMAKE_ARGS)

apps:
	@mkdir -p build
	@cd build && cmake .. -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGS) && make

clean:
	@rm -rf build*
