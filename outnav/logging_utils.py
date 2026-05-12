import io
import logging
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional


class _LoggingTee:
    """Mirror writes to the original stream and a logger."""

    def __init__(self, logger: logging.Logger, level: int, stream):
        self._logger = logger
        self._level = level
        self._stream = stream

    def write(self, message: str):
        if message and not message.isspace():
            # Strip trailing newlines so log lines stay single-line.
            self._logger.log(self._level, message.rstrip())
        self._stream.write(message)

    def flush(self):
        self._stream.flush()

    def fileno(self):
        # Needed by some libraries that expect a real file descriptor.
        if hasattr(self._stream, "fileno"):
            return self._stream.fileno()
        raise io.UnsupportedOperation("Underlying stream has no fileno()")

    def isatty(self):
        return False

    @property
    def encoding(self) -> Optional[str]:
        return getattr(self._stream, "encoding", None)


def _log_uncaught(exc_type, exc_value, exc_traceback):
    logging.getLogger("excepthook").error(
        "Uncaught exception",
        exc_info=(exc_type, exc_value, exc_traceback),
    )


def _install_thread_excepthook():
    """
    Route unhandled exceptions in threads into logging.
    Python 3.8+ supports threading.excepthook; older versions will be silent.
    """
    try:
        import threading

        def _hook(args):
            logging.getLogger("threading").error(
                "Unhandled thread exception in %s",
                args.thread.name,
                exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
            )

        threading.excepthook = _hook  # type: ignore[attr-defined]
    except Exception:
        # Best-effort only; do not block startup if missing.
        pass


def setup_logging(log_dir: str = "logs", prefix: str = "outnav") -> str:
    """
    Configure application-wide logging.

    - Ensures the log directory exists.
    - Creates a timestamped log file (prefix-YYYYmmdd-HHMMSS.log).
    - Sends all stdout/stderr output to the log file while still showing it in the terminal.
    """
    base_dir = Path(__file__).resolve().parent
    log_dir_path = Path(log_dir)
    if not log_dir_path.is_absolute():
        log_dir_path = base_dir / log_dir_path
    os.makedirs(log_dir_path, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = log_dir_path / f"{prefix}-{timestamp}.log"

    fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    original_stdout = sys.stdout
    original_stderr = sys.stderr

    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter(fmt))

    console_handler = logging.StreamHandler(original_stdout)
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter(fmt))

    logging.basicConfig(
        level=logging.DEBUG,
        handlers=[file_handler, console_handler],
        force=True,
    )

    # Redirect print()/tracebacks to the log while keeping terminal output.
    sys.stdout = _LoggingTee(logging.getLogger("stdout"), logging.INFO, original_stdout)
    sys.stderr = _LoggingTee(logging.getLogger("stderr"), logging.ERROR, original_stderr)

    # Capture uncaught exceptions (main thread + worker threads).
    sys.excepthook = _log_uncaught
    _install_thread_excepthook()

    logging.getLogger(__name__).info("Logging initialized at %s", log_path)
    return str(log_path)
