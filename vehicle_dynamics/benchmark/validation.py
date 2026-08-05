"""Phase 5.5 validation entry point."""

from .runner import run_benchmark


def run_phase55_validation() -> bool:
    report = run_benchmark(verbose=True)
    return report["status"] == "PASS"


if __name__ == "__main__":
    ok = run_phase55_validation()
    raise SystemExit(0 if ok else 1)
