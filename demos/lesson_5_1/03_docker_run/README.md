# DOCKER RUN

```bash
cd demos/lesson_5_1/03_docker_run
```

```bash
docker build -t rd51-run:latest .
```

```bash
docker rm -f rd51-run 2>/dev/null || true
docker run -d --name rd51-run rd51-run:latest
```

```bash
docker ps --filter name=rd51-run
docker logs rd51-run
docker inspect rd51-run --format '{{.State.Status}} {{.Config.Image}}'
```

```bash
docker exec -it rd51-run /usr/local/bin/ugv-controller --cli
status
exit
```

```bash
docker rm -f rd51-run
```
