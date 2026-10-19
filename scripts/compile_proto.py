"""
NexusOS Protobuf Compilation Script
Compiles all proto/*.proto into Python gRPC code stubs.
"""

import sys
import subprocess
from pathlib import Path


def compile_protobuf():
    project_root = Path(__file__).parent.parent
    proto_dir = project_root / "proto"
    out_dir = proto_dir / "gen"

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "__init__.py").touch(exist_ok=True)

    proto_files = sorted(proto_dir.glob("*.proto"))
    if not proto_files:
        print("No .proto files found in proto/ directory.")
        return

    for proto_file in proto_files:
        print(f"Compiling {proto_file.name} -> {out_dir} ...")
        cmd = [
            sys.executable,
            "-m",
            "grpc_tools.protoc",
            f"-I{proto_dir}",
            f"--python_out={out_dir}",
            f"--grpc_python_out={out_dir}",
            str(proto_file)
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            print(f"Successfully generated Protobuf stubs for {proto_file.name} in proto/gen!")
        except subprocess.CalledProcessError as e:
            print(f"Error compiling Protobuf {proto_file.name}: {e.stderr}")
        except FileNotFoundError:
            print("grpcio-tools module not found. Run 'pip install grpcio-tools' first.")


if __name__ == "__main__":
    compile_protobuf()
