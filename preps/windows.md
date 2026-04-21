# Windows 11

WSL2 з Ubuntu, всередині - Docker Engine.

Перед встановленням - у BIOS/UEFI має бути увімкнена віртуалізація. Перевірка: Task Manager -> Performance -> CPU, внизу має бути `Virtualization: Enabled`.

Якщо `Disabled` - reboot, зайти в BIOS/UEFI (F2 / Del / F10 при старті, залежить від вендора), знайти опцію типу `Intel Virtualization Technology` / `VT-x` / `AMD-V` / `SVM Mode` (зазвичай в секціях `Advanced`, `CPU Configuration` або `Security`), увімкнути, зберегти, вийти.

В **admin PowerShell**:

```powershell
wsl --install -d Ubuntu
```

Reboot. Після reboot Ubuntu запуститься - задати username + пароль.

Перевірка (PowerShell):

```powershell
wsl -l -v
```

Має бути `Ubuntu` / `Running` / `VERSION: 2`. Якщо `VERSION: 1` - `wsl --set-version Ubuntu 2`.

Далі всередині Ubuntu:

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
```

Вийти з WSL (`exit`), відкрити Ubuntu знов - підхопиться група `docker`.

Daemon сам не стартує. Або руками:

```bash
sudo service docker start
```

Або один раз увімкнути systemd - додати в `/etc/wsl.conf`:

```
[boot]
systemd=true
```

Потім `wsl --shutdown` з PowerShell і знов відкрити Ubuntu.

Перевірка:

```bash
docker run hello-world
```

**Важливо:** код тримати в `~/projects/...` всередині WSL, не на `C:\`. Через `/mnt/c/` IO в рази повільніше, VS Code буде нестерпно гальмувати.
