#!/bin/bash -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $DIR

RC_FILE="${HOME}/.$(basename ${SHELL})rc"
if command -v "pyenv" > /dev/null 2>&1; then
  echo -e "\n. ~/.pyenvrc" >> $RC_FILE
  cat <<EOF > "${HOME}/.pyenvrc"
if [ -z "\$PYENV_ROOT" ]; then
  export PATH=\$HOME/.pyenv/bin:\$HOME/.pyenv/shims:\$PATH
  export PYENV_ROOT="\$HOME/.pyenv"
  eval "\$(pyenv init -)"
  eval "\$(pyenv virtualenv-init -)"
fi
EOF
fi
source $RC_FILE

export MAKEFLAGS="-j$(nproc)"

PYENV_PYTHON_VERSION=$(cat .python-version)
if ! pyenv prefix ${PYENV_PYTHON_VERSION} &> /dev/null; then
  echo "pyenv ${PYENV_PYTHON_VERSION} install ..."
  CONFIGURE_OPTS="--enable-shared" pyenv install -f ${PYENV_PYTHON_VERSION}
fi

if ! command -v pipenv &> /dev/null; then
  echo "pipenv install ..."
  pip install pipenv
fi

echo "update pip"
pip install pip==21.3.1
pip install pipenv==2021.11.23

if [ -d "./xx" ]; then
  export PIPENV_SYSTEM=1
  export PIPENV_PIPFILE=./xx/Pipfile
fi

if [ -z "$PIPENV_SYSTEM" ]; then
  echo "PYTHONPATH=${PWD}" > .env
  RUN="pipenv run"
else
  RUN=""
fi

echo "pip packages install ..."
pipenv install --dev --deploy --clear
pyenv rehash

if [ -f "$DIR/.pre-commit-config.yaml" ]; then
  echo "precommit install ..."
  $RUN pre-commit install
  [ -d "./xx" ] && (cd xx && $RUN pre-commit install)
  [ -d "./notebooks" ] && (cd notebooks && $RUN pre-commit install)
fi
