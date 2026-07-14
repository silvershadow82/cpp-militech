FROM ubuntu:22.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ \
    cmake \
    make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY CMakeLists.txt .
COPY src/ src/

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --target rd51_build_demo -j

FROM alpine:latest

RUN apk add --no-cache libc6-compat libstdc++

COPY --from=builder /app/build/rd51_build_demo /usr/local/bin/rd51-build-demo

ENTRYPOINT ["/usr/local/bin/rd51-build-demo"]
