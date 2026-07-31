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
    && cmake --build build --target rd51_run_demo -j

FROM ubuntu:22.04

COPY --from=builder /app/build/ugv-controller /usr/local/bin/ugv-controller

ENTRYPOINT ["/usr/local/bin/ugv-controller"]
