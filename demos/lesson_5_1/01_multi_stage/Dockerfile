FROM ubuntu:22.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ \
    cmake \
    make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY CMakeLists.txt .
COPY src/ src/

RUN cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=/app/bin \
    && cmake --build build --target rd51_multi_stage_app -j

FROM ubuntu:22.04

COPY --from=builder /app/bin/app /usr/local/bin/

ENTRYPOINT ["/usr/local/bin/app"]
