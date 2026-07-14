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
    && cmake --build build --target rd51_volume_demo -j

FROM ubuntu:22.04

COPY --from=builder /app/build/rd51_volume_demo /usr/local/bin/rd51-volume-demo

ENTRYPOINT ["/usr/local/bin/rd51-volume-demo"]
