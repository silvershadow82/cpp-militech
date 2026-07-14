# VOLUME LAYOUT: КОНФІГИ, ЛОГИ, ДАНІ

```bash
cd demos/lesson_5_1/04_volume_layout
```

```bash
docker build -t rd51-volume:demo .
mkdir -p logs data
```

```bash
docker run --rm \
  -v "$(pwd)/config:/etc/ugv:ro" \
  -v "$(pwd)/logs:/var/log/ugv" \
  -v "$(pwd)/data:/var/data/ugv" \
  rd51-volume:demo
```

```bash
cat logs/controller.log
cat data/pose.csv
```
