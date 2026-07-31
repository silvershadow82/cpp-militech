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
    && cmake --build build -j

FROM ubuntu:22.04

COPY --from=builder /app/build/rd51_health_service /usr/local/bin/rd51-health-service
COPY --from=builder /app/build/rd51_health_check /usr/local/bin/rd51-health-check
COPY --from=builder /app/build/rd51_healthctl /usr/local/bin/rd51-healthctl

HEALTHCHECK \
  --interval=5s \
  --timeout=2s \
  --start-period=3s \
  --retries=2 \
  CMD /usr/local/bin/rd51-health-check

ENTRYPOINT ["/usr/local/bin/rd51-health-service"]
