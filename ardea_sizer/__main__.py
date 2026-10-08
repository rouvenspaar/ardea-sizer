import sys
import argparse

from pathlib import Path
from .inputs import Inputs

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = ROOT / "inputs.json"

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="ardea_sizer")
    parser.add_argument("input", nargs="?", default=DEFAULT_INPUT,
                        help="input JSON (default: inputs.json)")
    args = parser.parse_args(argv)

    inp = Inputs.from_json(args.input)

    return 0

if __name__ == "__main__":
    sys.exit(main())
