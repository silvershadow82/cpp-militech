# HEALTHCHECK: DOCKER ПЕРЕВІРЯЄ СИСТЕМУ

```bash
cd demos/lesson_5_1/05_healthcheck
```

```bash
docker build -t rd51-health:demo .
docker rm -f rd51-health 2>/dev/null || true
docker run -d --name rd51-health rd51-health:demo
```

```bash
docker ps --filter name=rd51-health
docker inspect rd51-health --format '{{.State.Health.Status}}'
docker logs rd51-health
```

```bash
docker exec rd51-health /usr/local/bin/rd51-healthctl fail
sleep 12
docker inspect rd51-health --format '{{.State.Health.Status}}'
docker ps --filter name=rd51-health
```

```bash
docker exec rd51-health /usr/local/bin/rd51-healthctl recover
sleep 12
docker inspect rd51-health --format '{{.State.Health.Status}}'
```

```bash
docker rm -f rd51-health
```
