ARG VARIANT=bookworm
FROM mcr.microsoft.com/vscode/devcontainers/base:${VARIANT}

ENV DEBIAN_FRONTEND=noninteractive

# System packages
RUN apt-get update \
 && apt-get -y install --no-install-recommends \
      # PlatformIO deps
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
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Node.js (for AI agent CLIs)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
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
RUN usermod -a -G dialout vscode
RUN usermod -a -G plugdev vscode

# Optional Fedora host compatibility
ARG FEDORA_COMPAT=0
RUN if [ "$FEDORA_COMPAT" = "1" ]; then \
      groupadd -g 18 compat_dialout; \
      usermod -a -G compat_dialout vscode; \
    fi

# npm global prefix (accessible to vscode user)
RUN mkdir -p /usr/local/share/npm-global \
 && chown -R vscode:vscode /usr/local/share/npm-global

# Persistent shell history directory (backed by a named Docker volume)
RUN mkdir -p /home/vscode/.history \
 && touch /home/vscode/.history/.bash_history \
 && touch /home/vscode/.history/.zsh_history \
 && chown -R vscode:vscode /home/vscode/.history

# Workspace, Claude & Gemini config dirs
RUN mkdir -p /workspace /home/vscode/.claude /home/vscode/.gemini /home/vscode/.qwen /home/vscode/.codex \
 && chown -R vscode:vscode /workspace /home/vscode/.claude /home/vscode/.gemini /home/vscode/.qwen /home/vscode/.codex

# Environment variables
ENV DEVCONTAINER=true
ENV SHELL=/bin/zsh
ENV EDITOR=nano
ENV VISUAL=nano
ENV NPM_CONFIG_PREFIX=/usr/local/share/npm-global
ENV PATH=$PATH:/usr/local/share/npm-global/bin

WORKDIR /workspace
# Expose PlatformIO PATH for all users (including root) via profile.d
# Must run as root — write to /etc/profile.d
RUN mkdir -p /etc/profile.d \
 && echo 'export PATH="$PATH:/home/vscode/.platformio/penv/bin"' > /etc/profile.d/platformio.sh \
 && chmod +r /etc/profile.d/platformio.sh

# Switch to non-root user for user-level installs
USER vscode

# zsh via zsh-in-docker
ARG ZSH_IN_DOCKER_VERSION=1.2.0
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v${ZSH_IN_DOCKER_VERSION}/zsh-in-docker.sh)" -- \
      -t clean \
      -p git \
      -p fzf \
      -a "source /usr/share/doc/fzf/examples/key-bindings.zsh" \
      -a "source /usr/share/doc/fzf/examples/completion.zsh" \
      -a "export HISTFILE=/home/vscode/.history/.bash_history" \
      -x

# PlatformIO CLI
RUN curl -fsSL -o /tmp/get-platformio.py \
      https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
 && python3 /tmp/get-platformio.py

RUN $HOME/.platformio/penv/bin/pip install flawfinder lizard bitproto

RUN echo 'export PATH="$PATH:$HOME/.platformio/penv/bin"' \
      | tee -a /home/vscode/.bashrc /home/vscode/.zshrc

# Configure zsh to use shared history file on persistent volume
RUN cat >> /home/vscode/.zshrc << 'EOF'

# Persistent history
export HISTFILE=/home/vscode/.history/.bash_history
export HISTSIZE=10000
export SAVEHIST=10000
setopt APPEND_HISTORY
setopt INC_APPEND_HISTORY
setopt SHARE_HISTORY
EOF

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

USER vscode
