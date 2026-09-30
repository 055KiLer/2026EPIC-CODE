#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path
import xml.etree.ElementTree as ET


def dedupe_keep_order(items: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        if item and item not in seen:
            seen.add(item)
            result.append(item)
    return result


def split_semicolon(value: str | None) -> list[str]:
    if not value:
        return []
    return [part.strip() for part in value.split(";") if part.strip()]


def split_comma(value: str | None) -> list[str]:
    if not value:
        return []
    return [part.strip() for part in value.split(",") if part.strip()]


def text_of(node: ET.Element | None, path: str, default: str = "") -> str:
    if node is None:
        return default
    found = node.find(path)
    if found is None or found.text is None:
        return default
    return found.text.strip()


def resolve_path(base_dir: Path, raw_path: str) -> Path:
    raw_path = raw_path.replace("\\", "/")
    candidate = (base_dir / raw_path).resolve()
    return candidate


def parse_cpu_flags(cpu_text: str) -> list[str]:
    flags: list[str] = ["--target=arm-arm-none-eabi"]

    cpu_match = re.search(r'CPUTYPE\("([^"]+)"\)', cpu_text)
    if cpu_match:
        cpu_name = cpu_match.group(1).strip().lower()
        flags.append(f"-mcpu={cpu_name}")

    if "FPU3(DFPU)" in cpu_text:
        flags.extend(["-mfpu=fpv5-sp-d16", "-mfloat-abi=hard"])

    return flags


def parse_compiler_compat_flags(target_node: ET.Element) -> list[str]:
    pcc_used = text_of(target_node, "pCCUsed")
    uac6 = text_of(target_node, "uAC6")

    pcc_used_upper = pcc_used.upper()
    version_match = re.search(r"\b(\d{6,})\b", pcc_used)
    version = version_match.group(1) if version_match else ""

    flags: list[str] = []
    if "ARMCLANG" in pcc_used_upper or uac6 == "1":
        # clangd does not see armclang's builtin macros unless query-driver is set.
        if version:
            flags.extend(
                [
                    "-U__ARMCC_VERSION",
                    f"-D__ARMCC_VERSION={version}",
                    "-U__ARMCOMPILER_VERSION",
                    f"-D__ARMCOMPILER_VERSION={version}",
                ]
            )
    elif "ARMCC" in pcc_used_upper or "ARMCC" in text_of(target_node, "ToolsetName").upper():
        flags.append("-D__CC_ARM=1")
        if version:
            flags.extend(["-U__ARMCC_VERSION", f"-D__ARMCC_VERSION={version}"])

    return flags


def first_existing_path(candidates: list[str], fallback: str) -> str:
    for candidate in candidates:
        if candidate and Path(candidate).expanduser().exists():
            return candidate
    return fallback


def default_armclang_root() -> str:
    env_root = os.environ.get("ARMCLANG_ROOT")
    return first_existing_path(
        [
            env_root or "",
            r"D:/Keil_v5/ARM/ARMCLANG",
            r"C:/Keil_v5/ARM/ARMCLANG",
            r"D:/Keil/ARM/ARMCLANG",
            r"C:/Keil/ARM/ARMCLANG",
            r"D:/Keil_Arm/ARM/ARMCLANG",
            r"C:/Keil_Arm/ARM/ARMCLANG",
        ],
        r"D:/Keil/ARM/ARMCLANG",
    )


def default_armclang_tool(tool_name: str) -> str:
    return str(Path(default_armclang_root()) / "bin" / tool_name)


def collect_selected_packages(root: ET.Element, target_name: str) -> dict[tuple[str, str], str]:
    selected: dict[tuple[str, str], str] = {}
    components = root.findall("./RTE/components/component")
    for component in components:
        target_infos = component.findall("./targetInfos/targetInfo")
        if target_infos:
            names = {item.attrib.get("name", "").strip() for item in target_infos}
            if target_name not in names:
                continue

        package = component.find("./package")
        if package is None:
            continue

        name = package.attrib.get("name", "").strip()
        vendor = package.attrib.get("vendor", "").strip()
        version = package.attrib.get("version", "").strip()
        if not name:
            continue

        key = (vendor, name)
        if key not in selected:
            selected[key] = version
    return selected


def resolve_package_base(pack_roots: list[str], vendor: str, name: str, version: str) -> Path | None:
    candidates: list[Path] = []
    for pack_root in pack_roots:
        root_path = Path(pack_root).expanduser().resolve()
        if version:
            candidates.append(root_path / name / version)
            if vendor:
                candidates.append(root_path / vendor / name / version)

    for path in candidates:
        if path.exists() and path.is_dir():
            return path

    version_fallback_roots: list[Path] = []
    for pack_root in pack_roots:
        root_path = Path(pack_root).expanduser().resolve()
        version_fallback_roots.append(root_path / name)
        if vendor:
            version_fallback_roots.append(root_path / vendor / name)

    for package_root in version_fallback_roots:
        if not package_root.exists() or not package_root.is_dir():
            continue
        versions = sorted([item for item in package_root.iterdir() if item.is_dir()], key=lambda item: item.name)
        if versions:
            return versions[-1]

    return None


def resolve_includes_from_selected_packages(
    selected_packages: dict[tuple[str, str], str],
    pack_roots: list[str],
) -> tuple[list[str], dict[str, list[str]]]:
    include_paths: list[str] = []
    include_details: dict[str, list[str]] = {}

    for (vendor, name), version in selected_packages.items():
        package_base = resolve_package_base(pack_roots, vendor, name, version)
        package_key = f"{vendor or 'Unknown'}::{name}@{version or 'latest'}"
        if package_base is None:
            include_details[package_key] = []
            continue

        candidate_dirs: list[Path] = []
        if name == "CMSIS-DSP":
            candidate_dirs.extend(
                [
                    package_base / "Include",
                    package_base / "Include" / "dsp",
                    package_base / "PrivateInclude",
                ]
            )
        elif name == "CMSIS":
            candidate_dirs.extend(
                [
                    package_base / "CMSIS" / "Core" / "Include",
                    package_base / "CMSIS" / "RTOS2" / "Include",
                    package_base / "Core" / "Include",
                ]
            )

        package_include_paths: list[str] = []
        for path in candidate_dirs:
            if path.exists() and path.is_dir():
                include_path = str(path.resolve())
                include_paths.append(include_path)
                package_include_paths.append(include_path)

        include_details[package_key] = dedupe_keep_order(package_include_paths)

    return dedupe_keep_order(include_paths), include_details


def inspect_selected_package_includes(
    uvproj_path: Path,
    target_name: str | None,
    pack_roots: list[str],
) -> tuple[str, dict[str, list[str]], list[str]]:
    tree = ET.parse(uvproj_path)
    root = tree.getroot()

    targets = root.findall("./Targets/Target")
    if not targets:
        raise ValueError("未在 uvprojx 中找到任何 Target")

    target_node: ET.Element | None = None
    if target_name:
        for target in targets:
            if text_of(target, "TargetName") == target_name:
                target_node = target
                break
        if target_node is None:
            raise ValueError(f"Target '{target_name}' 不存在")
    else:
        target_node = targets[0]

    selected_target_name = text_of(target_node, "TargetName")
    selected_packages = collect_selected_packages(root, selected_target_name)
    include_paths, include_details = resolve_includes_from_selected_packages(selected_packages, pack_roots)
    return selected_target_name, include_details, include_paths


def std_flags_for_file(path: Path) -> list[str]:
    suffix = path.suffix.lower()
    if suffix == ".c":
        return ["-std=gnu11"]
    if suffix in {".cpp", ".cxx", ".cc"}:
        return ["-std=gnu++17"]
    return []


def collect_file_option_overrides(file_node: ET.Element) -> tuple[list[str], list[str], list[str], list[str], bool]:
    file_option = file_node.find("FileOption")
    if file_option is None:
        return [], [], [], [], True

    include_build_text = text_of(file_option, "CommonProperty/IncludeInBuild", "1")
    include_in_build = include_build_text != "0"

    cads = file_option.find("FileArmAds/Cads/VariousControls")
    aads = file_option.find("FileArmAds/Aads/VariousControls")

    c_defines = split_comma(text_of(cads, "Define"))
    c_includes = split_semicolon(text_of(cads, "IncludePath"))
    c_misc = text_of(cads, "MiscControls")

    a_defines = split_comma(text_of(aads, "Define"))
    a_includes = split_semicolon(text_of(aads, "IncludePath"))
    a_misc = text_of(aads, "MiscControls")

    c_misc_flags = c_misc.split() if c_misc else []
    a_misc_flags = a_misc.split() if a_misc else []

    return c_defines + a_defines, c_includes + a_includes, c_misc_flags, a_misc_flags, include_in_build


def detect_project_root(uvproj_path: Path, explicit_root: Path | None) -> Path:
    if explicit_root is not None:
        return explicit_root.resolve()
    parent = uvproj_path.parent.resolve()
    if parent.name.upper() == "MDK-ARM":
        return parent.parent
    return parent


def build_commands(
    uvproj_path: Path,
    target_name: str | None,
    project_root: Path,
    output_dir: Path,
    cc_path: str,
    asm_path: str,
    sysroot_path: str,
    pack_roots: list[str],
) -> list[dict]:
    tree = ET.parse(uvproj_path)
    root = tree.getroot()

    targets = root.findall("./Targets/Target")
    if not targets:
        raise ValueError("未在 uvprojx 中找到任何 Target")

    target_node: ET.Element | None = None
    if target_name:
        for target in targets:
            if text_of(target, "TargetName") == target_name:
                target_node = target
                break
        if target_node is None:
            raise ValueError(f"Target '{target_name}' 不存在")
    else:
        target_node = targets[0]

    cpu_text = text_of(target_node, "TargetOption/TargetCommonOption/Cpu")
    cpu_flags = parse_cpu_flags(cpu_text)

    c_controls = target_node.find("TargetOption/TargetArmAds/Cads/VariousControls")
    a_controls = target_node.find("TargetOption/TargetArmAds/Aads/VariousControls")
    selected_target_name = text_of(target_node, "TargetName")
    selected_packages = collect_selected_packages(root, selected_target_name)
    selected_package_includes, _ = resolve_includes_from_selected_packages(selected_packages, pack_roots)
    compiler_compat_flags = parse_compiler_compat_flags(target_node)

    c_defines = split_comma(text_of(c_controls, "Define"))
    c_includes_raw = split_semicolon(text_of(c_controls, "IncludePath"))
    c_misc_flags = text_of(c_controls, "MiscControls").split() if text_of(c_controls, "MiscControls") else []

    a_defines = split_comma(text_of(a_controls, "Define"))
    a_includes_raw = split_semicolon(text_of(a_controls, "IncludePath"))
    a_misc_flags = text_of(a_controls, "MiscControls").split() if text_of(a_controls, "MiscControls") else []

    uvproj_dir = uvproj_path.parent.resolve()
    c_includes = [str(resolve_path(uvproj_dir, inc)) for inc in c_includes_raw]
    a_includes = [str(resolve_path(uvproj_dir, inc)) for inc in a_includes_raw]
    c_includes = dedupe_keep_order(c_includes + selected_package_includes)
    sysroot_include = str((Path(sysroot_path) / "include").resolve()) if sysroot_path else ""

    commands: list[dict] = []

    files = target_node.findall("./Groups/Group/Files/File")
    for file_node in files:
        file_path_raw = text_of(file_node, "FilePath")
        if not file_path_raw:
            continue

        file_path = resolve_path(uvproj_dir, file_path_raw)
        suffix = file_path.suffix.lower()
        if suffix not in {".c", ".cpp", ".cxx", ".cc", ".s", ".asm", ".S".lower()}:
            continue

        file_defines_extra, file_includes_extra_raw, file_c_misc, file_a_misc, include_in_build = collect_file_option_overrides(file_node)
        if not include_in_build:
            continue

        file_includes_extra = [str(resolve_path(uvproj_dir, p)) for p in file_includes_extra_raw]

        is_asm = suffix in {".s", ".asm"}

        if is_asm:
            base_asm_defines = a_defines if a_defines else c_defines
            defines = dedupe_keep_order(base_asm_defines + file_defines_extra)
            includes = dedupe_keep_order(a_includes + file_includes_extra)
            args = [asm_path] + [f"--pd={d}" for d in defines] + [f"-I{inc}" for inc in includes] + cpu_flags + a_misc_flags + file_a_misc + ["-c", str(file_path)]
        else:
            defines = dedupe_keep_order(c_defines + file_defines_extra)
            includes = dedupe_keep_order(c_includes + file_includes_extra)
            args = [cc_path] + cpu_flags
            if sysroot_path:
                args.append(f"--sysroot={sysroot_path}")
            if sysroot_include:
                args.append(f"-isystem{sysroot_include}")
            args += compiler_compat_flags + [f"-D{d}" for d in defines] + [f"-I{inc}" for inc in includes] + c_misc_flags + file_c_misc + ["-Wall", "-fdata-sections", "-ffunction-sections"] + std_flags_for_file(file_path) + ["-c", str(file_path)]

        commands.append(
            {
                "directory": str(output_dir),
                "arguments": args,
                "file": str(file_path),
            }
        )

    return commands


def main() -> int:
    env_pack_root = os.environ.get("CMSIS_PACK_ROOT")
    default_pack_roots = [
        r"D:/Keil_Arm/ARM",
        r"D:/Arm/Packs/ARM",
    ]
    if env_pack_root:
        default_pack_roots.insert(0, env_pack_root)

    parser = argparse.ArgumentParser(
        description="将 Keil 的 .uvprojx 工程转换为 compile_commands.json(供 clangd 等工具使用)",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "示例:\n"
            "  1) 使用默认参数(工程在 MDK-ARM/H7_Robot.uvprojx)\n"
            "     python .\\tools\\keil_to_compile_commands.py\n\n"
            "  2) 指定 uvproj / 输出文件 / build 目录\n"
            "     python .\\tools\\keil_to_compile_commands.py "
            "--uvproj .\\MDK-ARM\\H7_Robot.uvprojx "
            "--output .\\compile_commands.json "
            "--build-dir .\\build\n\n"
            "  3) 指定 Target(多 Target 工程时推荐)\n"
            "     python .\\tools\\keil_to_compile_commands.py --target H7_Robot\n\n"
            "  4) 指定 CMSIS Pack 根目录(自动补全 CMSIS-DSP 头路径)\n"
            "     python .\\tools\\keil_to_compile_commands.py --pack-root D:/Keil_Arm/ARM"
        ),
    )
    parser.add_argument(
        "--uvproj",
        default="MDK-ARM/H7_Robot.uvprojx",
        metavar="PATH",
        help="Keil 的 .uvprojx 文件路径(默认: MDK-ARM/H7_Robot.uvprojx)",
    )
    parser.add_argument(
        "--target",
        default=None,
        metavar="NAME",
        help="Keil Target 名称;不填则使用工程中的第一个 Target",
    )
    parser.add_argument(
        "--project-root",
        default=None,
        metavar="DIR",
        help="项目根目录;不填则自动推断(通常为 MDK-ARM 的上级目录)",
    )
    parser.add_argument(
        "--output",
        default="compile_commands.json",
        metavar="PATH",
        help="输出 JSON 路径(相对路径按 project-root 解析)",
    )
    parser.add_argument(
        "--build-dir",
        default=".",
        metavar="DIR",
        help="写入每条记录的 directory 字段(相对路径按 project-root 解析)",
    )
    parser.add_argument(
        "--cc",
        default=default_armclang_tool("armclang.exe"),
        metavar="PATH",
        help="C/C++ 编译器路径(armclang.exe)",
    )
    parser.add_argument(
        "--as",
        dest="asm",
        default=default_armclang_tool("armasm.exe"),
        metavar="PATH",
        help="汇编器路径(armasm.exe)",
    )
    parser.add_argument(
        "--sysroot",
        default=default_armclang_root(),
        metavar="PATH",
        help="ARMClang sysroot 路径(用于标准库头文件，如 math.h)",
    )
    parser.add_argument(
        "--pack-root",
        action="append",
        default=None,
        metavar="PATH",
        help="CMSIS Pack 根目录(可重复传入，多目录按顺序查找); 用于按 Keil 已选包自动补全 Include",
    )
    parser.add_argument(
        "--verbose-pack",
        action="store_true",
        help="打印 Keil 已选包以及自动补全的 Include 路径",
    )
    args = parser.parse_args()

    pack_roots = args.pack_root if args.pack_root else default_pack_roots

    uvproj_path = Path(args.uvproj).resolve()
    if not uvproj_path.exists():
        raise FileNotFoundError(f"uvprojx 文件不存在: {uvproj_path}")

    explicit_root = Path(args.project_root).resolve() if args.project_root else None
    project_root = detect_project_root(uvproj_path, explicit_root)

    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = (project_root / output_path).resolve()

    build_dir = Path(args.build_dir)
    if not build_dir.is_absolute():
        build_dir = (project_root / build_dir).resolve()

    commands = build_commands(
        uvproj_path=uvproj_path,
        target_name=args.target,
        project_root=project_root,
        output_dir=build_dir,
        cc_path=args.cc,
        asm_path=args.asm,
        sysroot_path=args.sysroot,
        pack_roots=pack_roots,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(commands, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"已生成 {len(commands)} 条编译命令 -> {output_path}")

    if args.verbose_pack:
        selected_target_name, include_details, include_paths = inspect_selected_package_includes(uvproj_path, args.target, pack_roots)
        print(f"[verbose-pack] Target: {selected_target_name}")
        print(f"[verbose-pack] Pack roots: {', '.join(pack_roots)}")
        if not include_details:
            print("[verbose-pack] 未检测到任何已选包")
        else:
            for package_key, paths in include_details.items():
                print(f"[verbose-pack] {package_key}")
                if paths:
                    for path in paths:
                        print(f"  + {path}")
                else:
                    print("  + (未找到可注入的 Include 目录)")
        print(f"[verbose-pack] 自动补全 Include 总数: {len(include_paths)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
