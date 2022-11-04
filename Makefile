all:
	cmake -Bbuild -H.; cmake --build build

tests:
	cmake -Bbuild -H.; cmake --build build --target test

clean:
	@rm -rf build/
	@echo "All build artifacts removed"

.PHONY: all clean gimbal_control