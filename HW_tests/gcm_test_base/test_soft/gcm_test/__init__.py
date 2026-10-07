"""Base de testes da GCM-PI2-2026.2: cliente serial, simulador e GUI."""

import os as _os

# Ver comentário em gui.py: evita o pyqtgraph carregar o Qt do PyQt6.
_os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")

__version__ = "1.0.0"
PROTO_VERSION = 1
