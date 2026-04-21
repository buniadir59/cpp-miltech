# Preps

Мінімум щоб devcontainer у цьому проєкті запрацював.

По ОС:

- `linux.md`
- `windows.md`
- `macos.md`

По редактору:

- VS Code - просто extension `Dev Containers`, далі `Reopen in Container`, нічого більше.
- Будь-який інший редактор (Neovim, JetBrains, Zed, Helix, Sublime, Emacs...) - див. `devcontainers-cli.md`.

Перевірка після всього (з кореня проєкту).

Через VS Code - `Reopen in Container`, далі у вбудованому терміналі (він уже всередині контейнера):

```bash
cmake -S . -B build
cmake --build build
./build/app
```

Через CLI (без VS Code) - кожна команда через `devcontainer exec`:

```bash
devcontainer up --workspace-folder .
devcontainer exec --workspace-folder . cmake -S . -B build
devcontainer exec --workspace-folder . cmake --build build
devcontainer exec --workspace-folder . ./build/app
```

Має вивести `Hello, section 2!`.

Якщо щось не те - в чат курсу.
