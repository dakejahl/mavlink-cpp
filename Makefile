all:
	@cmake -Bbuild -H.; cmake --build build -j 12

install:
	@cmake -Bbuild -H. -DCMAKE_INSTALL_PREFIX=/usr/local; cmake --build build --target install -j 12

examples:
	@cmake -Bexamples/build -Hexamples; cmake --build examples/build -j 12

clean:
	@rm -rf build/ examples/build
	@echo "All build artifacts removed"

.PHONY: all install examples clean