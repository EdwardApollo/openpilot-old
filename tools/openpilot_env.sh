if [ -z "$OPENPILOT_ENV" ]; then
  if [[ "$(uname)" == 'Darwin' ]]; then
    # msgq doesn't work on mac
    export ZMQ=1
    export OBJC_DISABLE_INITIALIZE_FORK_SAFETY=YES
  fi

  export OPENPILOT_ENV=1
fi
