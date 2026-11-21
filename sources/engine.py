"""Choose the engine: the Rust module ``nrt_matcher`` (rust/nrt-matcher) or the Python code.

``engine`` is ``"auto"``, ``"rust"`` or ``"python"``. ``None`` (default) = the value of the
environment variable ``NRT_ENGINE``, else ``"auto"``. ``"auto"`` uses Rust when ``nrt_matcher``
is installed, else Python. The two engines give the same results.
"""
import os

try:
    import nrt_matcher as rust
except ImportError:  # not compiled: the Python code is the engine
    rust = None

ENGINES = ("auto", "rust", "python")


def use_rust(engine=None):
    engine = engine or os.environ.get("NRT_ENGINE") or "auto"
    if engine not in ENGINES:
        raise ValueError(f"engine must be one of {ENGINES}, not {engine!r}")
    if engine == "rust" and rust is None:
        raise ImportError("engine='rust' but nrt_matcher is not installed "
                          "(build it with `pip install ./rust/nrt-matcher`)")
    return engine != "python" and rust is not None
