# Dockerを使う

### イメージのビルド
```bash
docker build . -f Dockerfile.add_irsl_ws -t irslrepo/irsl_mobile_arm:noetic
```

### 実行
```bash
docker compose -f <compose-file>.yaml up
```

```<compose-file>.yaml``` は 以下のファイル

- compose-linux-gpu.yaml
  - linux with GPU
- compose-linux.yaml
  - linux without GPU
- compose-win.yaml
  - Windows
- compose-web-linux-gpu.yaml
  - linux with GPU and web-server(for ros_bridge)
- compose-web-linux.yaml
  - linux without GPU and web-server(for ros_bridge)
- compose-web-win.yaml
  - Windows and web-server(for ros_bridge)

