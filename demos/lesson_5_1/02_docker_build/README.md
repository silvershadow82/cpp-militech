# DOCKER BUILD

```bash
cd demos/lesson_5_1/02_docker_build
```

```bash
docker build -t rd51-build:latest .
docker run --rm rd51-build:latest
```

```bash
docker buildx inspect --bootstrap | grep Platforms
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx inspect --bootstrap | grep Platforms
```

```bash
docker build --platform linux/arm64 -t rd51-build:arm64 .
```

```bash
docker image inspect rd51-build:arm64 --format '{{.Os}}/{{.Architecture}}'
```
