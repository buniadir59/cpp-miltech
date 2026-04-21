# Linux

Docker Engine через офіційний скрипт:

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
```

Вийти й зайти в сесію заново (або reboot) - нова група `docker` підхопиться.

На Ubuntu/Debian/Fedora daemon стартує сам через systemd. Якщо ні:

```bash
sudo systemctl start docker
sudo systemctl enable docker
```

Перевірка:

```bash
docker run hello-world
```
