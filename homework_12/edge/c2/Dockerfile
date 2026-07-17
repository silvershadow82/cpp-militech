# Етап 1: збирання
# MAVSDK у цьому ДЗ зафiксовано на конкретнiй версiї, щоб не шукати пакет вручну.
ARG MAVSDK_VERSION=2.12.9
# Дистрибутив/архiтектура готового .deb пакета MAVSDK. Тримаємо в одному
# мiсцi, щоб URL у builder та runtime не розходились.
ARG MAVSDK_DEB=ubuntu22.04_amd64

FROM --platform=linux/amd64 ubuntu:22.04 AS builder

ARG MAVSDK_VERSION
ARG MAVSDK_DEB
ENV DEBIAN_FRONTEND=noninteractive

# Блок залежностей надано: iнструменти збирання, CMake та wget для MAVSDK.
RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ cmake git make wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# MAVSDK встановлюється з готового .deb пакета для Ubuntu 22.04 (amd64).
RUN wget -qO /tmp/mavsdk-dev.deb \
    "https://github.com/mavlink/MAVSDK/releases/download/v${MAVSDK_VERSION}/libmavsdk-dev_${MAVSDK_VERSION}_${MAVSDK_DEB}.deb" \
    && dpkg -i /tmp/mavsdk-dev.deb \
    && rm /tmp/mavsdk-dev.deb

WORKDIR /src
COPY . .

# Заповнити команду CMake-збирання.
RUN cmake -S . -B build && cmake --build build

# Етап 2: запуск
# Для простоти ДЗ той самий MAVSDK .deb встановлюється i в образ запуску.
FROM --platform=linux/amd64 ubuntu:22.04

ARG MAVSDK_VERSION
ARG MAVSDK_DEB
ENV DEBIAN_FRONTEND=noninteractive

# apt-get install ./file.deb сам пiдтягує залежностi (libstdc++ тощо),
# на вiдмiну вiд dpkg -i.
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget ca-certificates \
    && wget -qO /tmp/mavsdk.deb \
    "https://github.com/mavlink/MAVSDK/releases/download/v${MAVSDK_VERSION}/libmavsdk-dev_${MAVSDK_VERSION}_${MAVSDK_DEB}.deb" \
    && apt-get install -y --no-install-recommends /tmp/mavsdk.deb \
    && rm /tmp/mavsdk.deb \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/c2 /var/log/c2

WORKDIR /app
COPY --from=builder /src/build/c2_service .

HEALTHCHECK --interval=5s --timeout=2s --retries=3 \
    CMD test -f /tmp/c2_healthy || exit 1

CMD ["./c2_service"]
