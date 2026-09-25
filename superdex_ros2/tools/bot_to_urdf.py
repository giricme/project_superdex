#!/usr/bin/env python3
# Copyright (c) 2026 Team 6. Licensed under the Apache License, Version 2.0.
"""Convert a SuperDex ``.superdex_bot`` robot description into a URDF.

SuperDex describes robots in its own JSON format and ships no URDF for the
arm-hand combos. Nearly every stock ROS 2 tool -- ``robot_state_publisher``,
RViz's RobotModel display, MoveIt -- requires one, so this fills the gap.

Standalone by design: no ROS, no SuperDex engine, no simulation. Pure JSON to
XML, run once, with the result committed alongside the package.

    python3 tools/bot_to_urdf.py \\
        --bot assets/bots/arm_hand_combos/fr3_dg5f_short/right/fr3_dg5f_short_right.superdex_bot \\
        --output-dir superdex_ros2/urdf

Format notes, established by reading the shipped assets rather than any
documentation:

* ``links`` and ``joints`` are parallel arrays. Joint *i* connects
  ``links[links[i]["parentLink"]]`` to ``links[i]``; the root link has no
  ``parentLink`` and its joint entry is a placeholder to be skipped.
* ``minLimit`` / ``maxLimit`` are 3-vectors indexed by the joint's axis, so a
  joint with ``axis = [0, 1, 0]`` carries its limits in component 1. Verified
  across both the arm (all axes ``[0,0,1]``, limits in component 2) and the
  hand (a mix of all three).
* ``momentOfInertia`` is ``(ixx, ixy, ixz, iyy, iyz, izz)``. Determined by
  elimination: the alternative diagonal-first ordering would give ``iyy = 0``
  for FR3 link 0, which is impossible for a real body.
* Rotations are quaternions in ``(x, y, z, w)`` order, matching
  ``geometry_msgs``. URDF wants roll-pitch-yaw, so they are converted here.
* Joint ``type`` is ``Revolute``, ``Hard`` (fixed) or ``Free`` (floating root).
* There are no effort or velocity limits in the source. URDF requires both on
  revolute joints, so defaults are supplied; they are placeholders, not
  manufacturer data, and matter to MoveIt but not to visualization.
* Combos are composition: a ``base`` bot plus ``modifications`` entries, each
  an ``AttachBot`` naming a parent link, a child bot and the fixed joint
  between them.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

# URDF requires these on revolute joints; the SuperDex format has no
# equivalent, so they are invented. Generous enough not to constrain planning,
# explicit enough to be obviously placeholders.
DEFAULT_EFFORT = 100.0  # [N m]
DEFAULT_VELOCITY = 2.0  # [rad/s]

# glTF is a Y-up format; URDF link frames here are Z-up. Neither assimp (which
# RViz loads meshes through) nor trimesh rotates on import, so the correction
# has to be declared. Verified against the shipped assets: fr3_link0 spans
# Y 0 -> 0.14 and fr3_link1 spans Y -0.192 -> 0.055, which are the robot's Z
# extents. A +90 degree rotation about X maps mesh (x, y, z) to link (x, -z, y),
# so 0.14 along mesh Y lands on 0.14 along link Z.
#
# Without it the frames are still correct -- only the geometry is rotated, so
# off-centre links appear to float away from their joints.
GLTF_Y_UP_RPY = (math.pi / 2.0, 0.0, 0.0)

JOINT_TYPE_MAP = {
    "Revolute": "revolute",
    "Hard": "fixed",
    "Prismatic": "prismatic",
    "Continuous": "continuous",
}


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------
def quat_xyzw_to_rpy(q: list[float]) -> tuple[float, float, float]:
    """Convert an (x, y, z, w) quaternion to URDF roll-pitch-yaw.

    Written out rather than pulled from scipy so the converter has no runtime
    dependencies beyond the standard library (trimesh is imported lazily, and
    only when mesh conversion is requested).
    """
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0)
    x, y, z, w = (c / norm for c in (x, y, z, w))

    sin_r_cos_p = 2.0 * (w * x + y * z)
    cos_r_cos_p = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_r_cos_p, cos_r_cos_p)

    # Clamp guards against the domain error that floating-point drift causes at
    # exactly vertical pitch.
    sin_p = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_p)

    sin_y_cos_p = 2.0 * (w * z + x * y)
    cos_y_cos_p = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_y_cos_p, cos_y_cos_p)

    return (roll, pitch, yaw)


def fmt_triplet(values) -> str:
    """Format three floats for a URDF attribute."""
    return " ".join(f"{float(v):.12g}" for v in values)


def origin_from_transform(transform: dict | None) -> tuple[list[float], tuple]:
    """Split a SuperDex transform into URDF (xyz, rpy).

    Either component may be absent -- most hand joints carry only a
    translation, several arm joints only a rotation -- so both default to
    identity.
    """
    transform = transform or {}
    xyz = transform.get("translation", [0.0, 0.0, 0.0])
    rotation = transform.get("rotation")
    rpy = quat_xyzw_to_rpy(rotation) if rotation else (0.0, 0.0, 0.0)
    return list(xyz), rpy


def limits_for_axis(joint: dict) -> tuple[float, float]:
    """Pull a joint's scalar limits out of its axis-indexed 3-vectors.

    The component that matters is the one the axis points along. Falls back to
    the largest-magnitude axis component so a non-unit or diagonal axis still
    produces something sane rather than silently reading zeros.
    """
    axis = joint.get("axis", [0.0, 0.0, 1.0])
    index = max(range(3), key=lambda i: abs(axis[i]))
    lower = float(joint.get("minLimit", [0, 0, 0])[index])
    upper = float(joint.get("maxLimit", [0, 0, 0])[index])
    return lower, upper


# --------------------------------------------------------------------------
# Loading and flattening
# --------------------------------------------------------------------------
@dataclass
class LinkEntry:
    """One URDF link, plus the joint that attaches it to its parent."""

    name: str
    parent: str | None
    joint_name: str
    joint: dict
    link: dict
    mesh_source: Path | None = None  # absolute path to the .glb, if any


@dataclass
class Bot:
    """A flattened robot: links in parent-before-child order."""

    name: str
    entries: list[LinkEntry] = field(default_factory=list)


def resolve_bot_path(reference: str, assets_root: Path, relative_to: Path) -> Path:
    """Resolve a bot path reference.

    A leading ``//`` means "from the bots root"; anything else is relative to
    the referring file.
    """
    if reference.startswith("//"):
        return (assets_root / "bots" / reference[2:]).resolve()
    return (relative_to.parent / reference).resolve()


def load_bot(
    bot_path: Path,
    assets_root: Path,
    parent_link: str | None = None,
    attach_joint: dict | None = None,
) -> list[LinkEntry]:
    """Load one bot file and return its links, resolving any composition.

    ``parent_link`` and ``attach_joint`` are supplied when this bot is being
    attached to another: they replace the bot's own root joint, which is a
    placeholder (``Hard`` for a welded arm, ``Free`` for a hand that would
    otherwise float).
    """
    data = json.loads(bot_path.read_text())
    links = data["links"]
    joints = data["joints"]
    if len(links) != len(joints):
        raise ValueError(
            f"{bot_path.name}: {len(links)} links but {len(joints)} joints; "
            "the parallel-array assumption does not hold for this file"
        )

    entries: list[LinkEntry] = []
    for index, (link, joint) in enumerate(zip(links, joints)):
        parent_index = link.get("parentLink")
        if parent_index is None:
            # Root of this bot: either the robot's true root, or the point
            # where it hangs off a parent bot.
            parent = parent_link
            joint_def = attach_joint or {"type": "Hard"}
            joint_name = (
                attach_joint.get("name", f"{link['name']}_joint")
                if attach_joint
                else f"{link['name']}_joint"
            )
        else:
            parent = links[parent_index]["name"]
            joint_def = joint
            joint_name = joint["name"]

        render_model = link.get("renderModel")
        mesh_source = (
            (bot_path.parent / render_model).resolve() if render_model else None
        )

        entries.append(
            LinkEntry(
                name=link["name"],
                parent=parent,
                joint_name=joint_name,
                joint=joint_def,
                link=link,
                mesh_source=mesh_source,
            )
        )

    # Composition: each AttachBot splices another bot in under a named link.
    for modification in data.get("modifications", []):
        attach = modification.get("AttachBot")
        if not attach or not attach.get("enabled", True):
            continue
        child_path = resolve_bot_path(attach["path"], assets_root, bot_path)
        entries.extend(
            load_bot(
                child_path,
                assets_root,
                parent_link=attach["parentLinkName"],
                attach_joint=attach.get("joint"),
            )
        )

    return entries


def load_robot(bot_path: Path, assets_root: Path) -> Bot:
    """Load a bot, following the ``base`` reference if it is a combo file."""
    data = json.loads(bot_path.read_text())
    name = data.get("name", bot_path.stem)

    if "base" in data:
        # A combo: the real link data lives in the base bot, and this file
        # contributes only a name and the attachments.
        base_path = resolve_bot_path(data["base"], assets_root, bot_path)
        entries = load_bot(base_path, assets_root)
        for modification in data.get("modifications", []):
            attach = modification.get("AttachBot")
            if not attach or not attach.get("enabled", True):
                continue
            child_path = resolve_bot_path(attach["path"], assets_root, bot_path)
            entries.extend(
                load_bot(
                    child_path,
                    assets_root,
                    parent_link=attach["parentLinkName"],
                    attach_joint=attach.get("joint"),
                )
            )
    else:
        entries = load_bot(bot_path, assets_root)

    return Bot(name=name, entries=entries)


# --------------------------------------------------------------------------
# URDF emission
# --------------------------------------------------------------------------
def build_urdf(
    bot: Bot,
    mesh_uris: dict[str, str],
    world_link: bool,
    mesh_rpy: tuple[float, float, float] = GLTF_Y_UP_RPY,
) -> ET.Element:
    """Build the URDF element tree."""
    robot = ET.Element("robot", name=bot.name)

    if world_link:
        # Give the URDF tree the same root as the simulation's /tf tree, so
        # frames from robot_state_publisher and from the sim node are directly
        # comparable rather than living in two disconnected trees.
        ET.SubElement(robot, "link", name="world")

    for entry in bot.entries:
        link_el = ET.SubElement(robot, "link", name=entry.name)

        mass = float(entry.link.get("mass", 0.0))
        if mass > 0.0:
            # A zero-mass link gets no inertial block at all: KDL treats a
            # massless link as a frame, which is what these are, and an
            # all-zero inertia tensor makes some consumers complain.
            inertial = ET.SubElement(link_el, "inertial")
            com = entry.link.get("centerOfMass", [0.0, 0.0, 0.0])
            ET.SubElement(inertial, "origin", xyz=fmt_triplet(com), rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=f"{mass:.12g}")
            moi = entry.link.get("momentOfInertia", [0.0] * 6)
            ixx, ixy, ixz, iyy, iyz, izz = moi
            ET.SubElement(
                inertial,
                "inertia",
                ixx=f"{ixx:.12g}",
                ixy=f"{ixy:.12g}",
                ixz=f"{ixz:.12g}",
                iyy=f"{iyy:.12g}",
                iyz=f"{iyz:.12g}",
                izz=f"{izz:.12g}",
            )

        uri = mesh_uris.get(entry.name)
        if uri:
            visual = ET.SubElement(link_el, "visual")
            ET.SubElement(
                visual, "origin", xyz="0 0 0", rpy=fmt_triplet(mesh_rpy)
            )
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, "mesh", filename=uri)

    if world_link:
        root = next(e for e in bot.entries if e.parent is None)
        joint_el = ET.SubElement(
            robot, "joint", name="world_to_base", type="fixed"
        )
        ET.SubElement(joint_el, "parent", link="world")
        ET.SubElement(joint_el, "child", link=root.name)
        ET.SubElement(joint_el, "origin", xyz="0 0 0", rpy="0 0 0")

    for entry in bot.entries:
        if entry.parent is None:
            continue  # handled by world_to_base, or has no parent at all

        raw_type = entry.joint.get("type", "Hard")
        urdf_type = JOINT_TYPE_MAP.get(raw_type)
        if urdf_type is None:
            raise ValueError(
                f"joint '{entry.joint_name}': unsupported type '{raw_type}'"
            )

        joint_el = ET.SubElement(
            robot, "joint", name=entry.joint_name, type=urdf_type
        )
        ET.SubElement(joint_el, "parent", link=entry.parent)
        ET.SubElement(joint_el, "child", link=entry.name)

        xyz, rpy = origin_from_transform(entry.joint.get("parentLinkFromJoint"))
        ET.SubElement(joint_el, "origin", xyz=fmt_triplet(xyz), rpy=fmt_triplet(rpy))

        if urdf_type in ("revolute", "prismatic", "continuous"):
            axis = entry.joint.get("axis", [0.0, 0.0, 1.0])
            ET.SubElement(joint_el, "axis", xyz=fmt_triplet(axis))
        if urdf_type in ("revolute", "prismatic"):
            lower, upper = limits_for_axis(entry.joint)
            ET.SubElement(
                joint_el,
                "limit",
                lower=f"{lower:.12g}",
                upper=f"{upper:.12g}",
                effort=f"{DEFAULT_EFFORT:.12g}",
                velocity=f"{DEFAULT_VELOCITY:.12g}",
            )
            friction = entry.joint.get("friction", {})
            if friction:
                ET.SubElement(
                    joint_el,
                    "dynamics",
                    damping=f"{float(friction.get('viscous', 0.0)):.12g}",
                    friction=f"{float(friction.get('coulomb', 0.0)):.12g}",
                )

    return robot


def export_meshes(
    bot: Bot, mesh_dir: Path, package: str, mesh_format: str
) -> dict[str, str]:
    """Copy or convert each link's render mesh; return link name -> URDF URI."""
    mesh_dir.mkdir(parents=True, exist_ok=True)
    uris: dict[str, str] = {}

    for entry in bot.entries:
        source = entry.mesh_source
        if source is None:
            continue
        if not source.is_file():
            print(f"  warning: mesh missing for {entry.name}: {source}", file=sys.stderr)
            continue

        if mesh_format == "glb":
            destination = mesh_dir / f"{entry.name}.glb"
            shutil.copyfile(source, destination)
        else:
            # RViz loads meshes through assimp, which handles glTF 2.0 on
            # recent distributions but not universally. Converting to Collada
            # or STL is the fallback when it does not.
            try:
                import trimesh
            except ImportError as exc:  # pragma: no cover
                raise SystemExit(
                    f"--mesh-format {mesh_format} needs trimesh: pip install trimesh"
                ) from exc
            destination = mesh_dir / f"{entry.name}.{mesh_format}"
            mesh = trimesh.load(source, force="mesh")
            mesh.export(destination)

        uris[entry.name] = f"package://{package}/meshes/{destination.name}"

    return uris


def find_assets_root(start: Path) -> Path | None:
    """Walk up from a path looking for a SuperDex ``assets`` directory."""
    for parent in [start, *start.parents]:
        if (parent / "bots").is_dir() and parent.name == "assets":
            return parent
        candidate = parent / "assets"
        if (candidate / "bots").is_dir():
            return candidate
    return None


def main() -> int:
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bot", required=True, type=Path, help="Path to .superdex_bot")
    parser.add_argument(
        "--assets-root",
        type=Path,
        help="SuperDex assets directory (inferred from --bot if omitted)",
    )
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--package",
        default="superdex_ros2",
        help="ROS package name used in package:// mesh URIs",
    )
    parser.add_argument(
        "--mesh-format",
        default="glb",
        choices=("glb", "dae", "stl"),
        help="glb copies as-is; dae and stl convert via trimesh",
    )
    parser.add_argument(
        "--mesh-up",
        default="y",
        choices=("y", "z"),
        help="Up axis of the source meshes. glTF is Y-up, which is what "
        "SuperDex ships; 'z' emits no corrective rotation.",
    )
    parser.add_argument(
        "--no-world-link",
        action="store_true",
        help="Omit the fixed world -> root joint",
    )
    args = parser.parse_args()

    bot_path = args.bot.resolve()
    if not bot_path.is_file():
        parser.error(f"no such file: {bot_path}")

    assets_root = args.assets_root or find_assets_root(bot_path)
    if assets_root is None:
        parser.error("could not locate the assets root; pass --assets-root")
    assets_root = Path(assets_root).resolve()

    bot = load_robot(bot_path, assets_root)
    print(f"Loaded '{bot.name}': {len(bot.entries)} links")

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    mesh_uris = export_meshes(
        bot, output_dir.parent / "meshes", args.package, args.mesh_format
    )
    print(f"Exported {len(mesh_uris)} meshes as .{args.mesh_format}")

    mesh_rpy = GLTF_Y_UP_RPY if args.mesh_up == "y" else (0.0, 0.0, 0.0)
    robot = build_urdf(
        bot,
        mesh_uris,
        world_link=not args.no_world_link,
        mesh_rpy=mesh_rpy,
    )
    ET.indent(robot, space="  ")
    urdf_path = output_dir / f"{bot.name}.urdf"
    urdf_path.write_bytes(
        b'<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="utf-8")
    )

    movable = [
        e.joint_name
        for e in bot.entries
        if JOINT_TYPE_MAP.get(e.joint.get("type", "Hard")) == "revolute"
        and e.parent is not None
    ]
    print(f"Wrote {urdf_path}")
    print(f"  {len(bot.entries)} links, {len(movable)} revolute joints")
    print("  joint names must match /joint_states exactly; first few:")
    for name in movable[:8]:
        print(f"    {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())