all:
	@cmake -Bbuild -H.; cmake --build build --target install -j 12

clean:
	@rm -rf build/
	@echo "All build artifacts removed"

.PHONY: all clean gimbal_control