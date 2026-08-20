ARG VARIANT=bookworm
FROM mcr.microsoft.com/vscode/devcontainers/base:${VARIANT}

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# System packages
RUN apt-get update \
 && apt-get -y install --no-install-recommends \
      clang \
      python3-venv \
      udev \
      pip \
      gdb \
      cppcheck \
      less \
      git \
      procps \
      sudo \
      fzf \
      zsh \
      man-db \
      unzip \
      gnupg2 \
      gh \
      iptables \
      ipset \
      iproute2 \
      dnsutils \
      aggregate \
      jq \
      wget \
      clang-format \
      clangd \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Node.js (for AI agent CLIs)
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
 && apt-get install -y nodejs \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# git-delta (syntax-highlighted diffs in CLI)
ARG GIT_DELTA_VERSION=0.18.2
RUN ARCH=$(dpkg --print-architecture) \
 && wget -q "https://github.com/dandavison/delta/releases/download/${GIT_DELTA_VERSION}/git-delta_${GIT_DELTA_VERSION}_${ARCH}.deb" \
 && dpkg -i "git-delta_${GIT_DELTA_VERSION}_${ARCH}.deb" \
 && rm "git-delta_${GIT_DELTA_VERSION}_${ARCH}.deb"

# udev rules for PlatformIO
RUN curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
      | tee /etc/udev/rules.d/99-platformio-udev.rules
RUN service udev restart

# Optional Fedora host compatibility
ARG FEDORA_COMPAT=0

# Create non-root user matching host UID/GID
# If UID/GID already exists from base image, purge it first
RUN if id -u $USER_UID 2>/dev/null; then \
      userdel -f $(id -un $USER_UID); \
    fi; \
    if getent group $USER_GID 2>/dev/null; then \
      groupdel $(getent group $USER_GID | cut -d: -f1); \
    fi; \
    groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

RUN usermod -a -G dialout $USERNAME
RUN usermod -a -G plugdev $USERNAME

RUN if [ "$FEDORA_COMPAT" = "1" ]; then \
      groupadd -g 18 compat_dialout; \
      usermod -a -G compat_dialout $USERNAME; \
    fi

# npm global prefix (accessible to non-root user)
RUN mkdir -p /usr/local/share/npm-global \
 && chown -R $USERNAME:$USERNAME /usr/local/share/npm-global

# Persistent shell history directory (backed by a named Docker volume)
RUN mkdir -p /home/$USERNAME/.history \
 && touch /home/$USERNAME/.history/.bash_history \
 && touch /home/$USERNAME/.history/.zsh_history \
 && chown -R $USERNAME:$USERNAME /home/$USERNAME/.history

# Workspace, AI config dirs (bind-mounted from host)
RUN mkdir -p /workspace \
      /home/$USERNAME/.claude \
      /home/$USERNAME/.gemini \
      /home/$USERNAME/.qwen \
      /home/$USERNAME/.codex \
      /home/$USERNAME/.pi \
      /home/$USERNAME/.config/opencode \
      /home/$USERNAME/.local/share/opencode \
      /home/$USERNAME/.local/state/opencode \
  && chown -R $USERNAME:$USERNAME \
      /workspace /home/$USERNAME/.claude /home/$USERNAME/.gemini /home/$USERNAME/.qwen \
      /home/$USERNAME/.codex /home/$USERNAME/.pi \
      /home/$USERNAME/.config/opencode /home/$USERNAME/.local/share/opencode \
      /home/$USERNAME/.local/state/opencode

# Environment variables
ENV DEVCONTAINER=true
ENV SHELL=/bin/zsh
ENV EDITOR=nano
ENV VISUAL=nano
ENV NPM_CONFIG_PREFIX=/usr/local/share/npm-global
ENV PATH=$PATH:/usr/local/share/npm-global/bin:/home/${USERNAME}/.platformio/penv/bin

WORKDIR /workspace
# Expose PlatformIO PATH for all users (including root) via profile.d
RUN mkdir -p /etc/profile.d \
 && echo 'export PATH="$PATH:/home/'$USERNAME'/.platformio/penv/bin"' > /etc/profile.d/platformio.sh \
 && chmod +r /etc/profile.d/platformio.sh

# Switch to non-root user for user-level installs
USER $USERNAME

# zsh via zsh-in-docker
ARG ZSH_IN_DOCKER_VERSION=1.2.0
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v${ZSH_IN_DOCKER_VERSION}/zsh-in-docker.sh)" -- \
      -t clean \
      -p git \
      -p fzf \
      -a "source /usr/share/doc/fzf/examples/key-bindings.zsh" \
      -a "source /usr/share/doc/fzf/examples/completion.zsh" \
      -a "export HISTFILE=/home/$USERNAME/.history/.bash_history" \
      -x

# PlatformIO CLI
RUN curl -fsSL -o /tmp/get-platformio.py \
      https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
 && python3 /tmp/get-platformio.py

RUN $HOME/.platformio/penv/bin/pip install flawfinder lizard bitproto pyserial ruff

# Pre-warm toolchain + IDF venv. Without this, PIO Home and first `pio run` race in
# espidf.py's `_create_venv` (rm -rf + recreate); whichever finishes first gets undone.
RUN mkdir -p /tmp/pio-warmup/src \
 && printf '%s\n' 'extern "C" void app_main(void) {}' > /tmp/pio-warmup/src/main.cpp \
 && printf '%s\n' '[platformio]' 'default_envs = esp32dev' '' '[env:esp32dev]' \
        'platform = espressif32@6.13.0' 'framework = espidf' 'board = esp32dev' \
        > /tmp/pio-warmup/platformio.ini \
 && cd /tmp/pio-warmup \
 && $HOME/.platformio/penv/bin/pio run \
 && rm -rf /tmp/pio-warmup

RUN echo 'export PATH="$HOME/.platformio/penv/bin:$PATH"' \
      | tee -a /home/$USERNAME/.bashrc /home/$USERNAME/.zshrc

# Configure zsh to use shared history file on persistent volume
RUN echo "export HISTFILE=/home/$USERNAME/.history/.bash_history" >> /home/$USERNAME/.zshrc \
 && echo "export HISTSIZE=10000" >> /home/$USERNAME/.zshrc \
 && echo "export SAVEHIST=10000" >> /home/$USERNAME/.zshrc \
 && echo "setopt APPEND_HISTORY" >> /home/$USERNAME/.zshrc \
 && echo "setopt INC_APPEND_HISTORY" >> /home/$USERNAME/.zshrc \
 && echo "setopt SHARE_HISTORY" >> /home/$USERNAME/.zshrc

# AI agent CLI tools — install only if flags are set to "1"

# Claude Code CLI
ARG INSTALL_CLAUDE=1
RUN if [ "$INSTALL_CLAUDE" = "1" ]; then \
      npm install -g @anthropic-ai/claude-code; \
    fi

# Gemini CLI
ARG INSTALL_GEMINI=1
RUN if [ "$INSTALL_GEMINI" = "1" ]; then \
      npm install -g @google/gemini-cli; \
    fi

# OpenAI Codex CLI
ARG INSTALL_CODEX=1
RUN if [ "$INSTALL_CODEX" = "1" ]; then \
      npm install -g @openai/codex; \
    fi

# Opencode CLI
ARG INSTALL_OPENCODE=1
RUN if [ "$INSTALL_OPENCODE" = "1" ]; then \
      curl -fsSL https://opencode.ai/install | bash; \
    fi

# Qwen Code CLI
ARG INSTALL_QWEN=1
RUN if [ "$INSTALL_QWEN" = "1" ]; then \
      npm install -g @qwen-code/qwen-code@latest; \
    fi

# Pi Code CLI
ARG INSTALL_PI=1
RUN if [ "$INSTALL_PI" = "1" ]; then \
      npm install -g @earendil-works/pi-coding-agent; \
    fi

USER $USERNAME
