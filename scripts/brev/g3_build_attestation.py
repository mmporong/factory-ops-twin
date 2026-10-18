#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.g3_build_attestation import (
    build_attested_image, create_nonphysical_fixture_attestation, references, validate_attestation,
)

parser = argparse.ArgumentParser()
sub = parser.add_subparsers(dest="command", required=True)
refs = sub.add_parser("references")
for command in (refs,):
    command.add_argument("--attestation", type=Path, required=True)
    command.add_argument("--expected-sha256", required=True)
validate = sub.add_parser("validate")
validate.add_argument("--attestation", type=Path, required=True)
validate.add_argument("--expected-sha256", required=True)
validate.add_argument("--source-root", type=Path, required=True)
validate.add_argument("--source-commit", required=True)
validate.add_argument("--run-label", required=True)
validate.add_argument("--base-inspect", type=Path, required=True)
validate.add_argument("--built-inspect", type=Path, required=True)
validate.add_argument("--source-archive", type=Path, required=True)
create = sub.add_parser("fixture-create")
create.add_argument("--output", type=Path, required=True)
create.add_argument("--source-root", type=Path, required=True)
create.add_argument("--source-commit", required=True)
create.add_argument("--run-label", required=True)
create.add_argument("--base-reference", required=True)
create.add_argument("--base-inspect", type=Path, required=True)
create.add_argument("--built-reference", required=True)
create.add_argument("--built-inspect", type=Path, required=True)
create.add_argument("--installed-files", type=Path, required=True)
build = sub.add_parser("build")
build.add_argument("--output-dir", type=Path, required=True)
build.add_argument("--repository", type=Path, required=True)
build.add_argument("--source-commit", required=True)
build.add_argument("--run-label", required=True)
build.add_argument("--base-reference", required=True)
args = parser.parse_args()
if args.command == "references":
    print("\n".join(references(args.attestation, args.expected_sha256)))
elif args.command == "validate":
    result = validate_attestation(args.attestation, expected_sha256=args.expected_sha256,
        source_root=args.source_root, expected_source_commit=args.source_commit,
        expected_run_label=args.run_label,
        base_inspect=json.loads(args.base_inspect.read_text()),
        built_inspect=json.loads(args.built_inspect.read_text()), source_archive=args.source_archive)
    print(json.dumps({"status": "valid", "built_image_id": result["built_image"]["id"]}, sort_keys=True))
elif args.command == "fixture-create":
    result = create_nonphysical_fixture_attestation(args.output, source_root=args.source_root,
        source_commit=args.source_commit, run_label=args.run_label,
        base_reference=args.base_reference,
        base_inspect=json.loads(args.base_inspect.read_text()),
        built_reference=args.built_reference,
        built_inspect=json.loads(args.built_inspect.read_text()),
        installed_files=json.loads(args.installed_files.read_text()))
    print(json.dumps({"status": "created", "sha256": __import__("hashlib").sha256(
        args.output.read_bytes()).hexdigest(), "source_set_sha256": result["source_set_sha256"]}, sort_keys=True))
else:
    result = build_attested_image(args.output_dir, repository=args.repository,
        expected_commit=args.source_commit, run_label=args.run_label,
        base_reference=args.base_reference)
    attestation = args.output_dir / "build-attestation.json"
    print(json.dumps({"status": "built", "sha256": __import__("hashlib").sha256(
        attestation.read_bytes()).hexdigest(), "built_image": result["built_image"]}, sort_keys=True))
