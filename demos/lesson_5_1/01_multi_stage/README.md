# MULTI-STAGE DOCKERFILE: ПРИНЦИП

```bash
cd demos/lesson_5_1/01_multi_stage
```

```bash
docker build -t rd51-multi-stage:demo .
docker run --rm rd51-multi-stage:demo
```

```dockerfile
COPY --from=builder /app/bin/app /usr/local/bin/
```
