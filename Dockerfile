FROM ubuntu:20.10

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update
# toolchain
RUN apt-get install -y \
	binutils-arm-none-eabi \
	gcc-arm-none-eabi \
	libnewlib-arm-none-eabi \
	libstdc++-arm-none-eabi-newlib
# tools
RUN apt-get install -y \
	cmake \
	git \
	make \
	python3

WORKDIR /build

CMD cmake -D CMAKE_BUILD_TYPE=MinSizeRel -D CMAKE_TOOLCHAIN_FILE=stm32f4-gcc.cmake \
	-D CF2_PROJECT=${CF2_PROJECT} /firmware && \
	make -j "$(nproc)" cf2_bin && \
	cp ./projects/cf2.bin /out
