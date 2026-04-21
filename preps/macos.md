# macOS

Без Docker Desktop - через Colima.

```bash
brew install colima docker docker-buildx
```

Підключити buildx як CLI plugin:

```bash
mkdir -p ~/.docker/cli-plugins
ln -sfn "$(brew --prefix)/opt/docker-buildx/bin/docker-buildx" ~/.docker/cli-plugins/docker-buildx
```

Запустити:

```bash
colima start
```

Перший запуск тягне linuxkit image - пару хвилин.

Перевірка:

```bash
docker run hello-world
docker buildx version
```

Автостарт при логіні (опційно):

```bash
brew services start colima
```

На Apple Silicon (M1/M2/M3/M4) все запускається нативно як `arm64`. Якщо раптом попаде образ тільки `amd64` - `docker run --platform linux/amd64 ...` (повільніше, через qemu).
