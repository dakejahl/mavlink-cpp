all:
	@cmake -Bbuild -H.; cmake --build build --target install -j 12

examples:
	@cmake -Bexamples/build -Hexamples; cmake --build examples/build -j 12

clean:
	@rm -rf build/ install/ examples/build
	@echo "All build artifacts removed"

.PHONY: all examples clean