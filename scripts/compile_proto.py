"""
NexusOS Protobuf Compilation Script
Compiles proto/nexus.proto into Python gRPC code stubs.
"""

import sys
import subprocess
from pathlib import Path


def compile_protobuf():
    project_root = Path(__file__).parent.parent
    proto_file = project_root / "proto" / "nexus.proto"
    out_dir = project_root / "proto" / "gen"

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "__init__.py").touch(exist_ok=True)

    print(f"Compiling {proto_file} -> {out_dir} ...")

    cmd = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"-I{project_root / 'proto'}",
        f"--python_out={out_dir}",
        f"--grpc_python_out={out_dir}",
        str(proto_file)
    ]

    try:
        res = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print("Successfully generated Protobuf stubs in proto/gen!")
    except subprocess.CalledProcessError as e:
        print(f"Error compiling Protobuf: {e.stderr}")
    except FileNotFoundError:
        print("grpcio-tools module not found. Run 'pip install grpcio-tools' first.")


if __name__ == "__main__":
    compile_protobuf()
