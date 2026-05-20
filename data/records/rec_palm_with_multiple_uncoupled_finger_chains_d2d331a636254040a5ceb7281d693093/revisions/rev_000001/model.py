from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A manufacturing-like rounded rectangle block centered on its local frame."""

    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _add_link_visuals(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    material,
    band_material,
    accent_material=None,
    tip: bool = False,
) -> None:
    """Create a compact robotic phalanx that starts at local y=0 and extends +Y."""

    shell = _rounded_box((width, length, thickness), min(width, thickness) * 0.22)
    part.visual(
        mesh_from_cadquery(shell, f"{part.name}_shell", tolerance=0.0007),
        origin=Origin(xyz=(0.0, length * 0.5, 0.0)),
        material=material,
        name="link_shell",
    )
    part.visual(
        Box((width + 0.003, 0.006, thickness + 0.003)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=band_material,
        name="proximal_band",
    )
    if accent_material is not None:
        part.visual(
            Box((width * 0.55, length * 0.34, 0.002)),
            origin=Origin(xyz=(0.0, length * 0.58, thickness * 0.5 + 0.0006)),
            material=accent_material,
            name="top_inlay",
        )
    if tip:
        part.visual(
            Sphere(radius=width * 0.48),
            origin=Origin(xyz=(0.0, length, 0.0)),
            material=band_material,
            name="soft_tip",
        )


def _add_finger_chain(
    model: ArticulatedObject,
    *,
    palm,
    name: str,
    root_xyz: tuple[float, float, float],
    lengths: tuple[float, float, float, float],
    width: float,
    thickness: float,
    splay_limits: tuple[float, float],
    shell_material,
    band_material,
    accent_material,
) -> None:
    """Four-link finger: metacarpal splay base plus three serial flexion joints."""

    base = model.part(f"{name}_base")
    proximal = model.part(f"{name}_proximal")
    middle = model.part(f"{name}_middle")
    distal = model.part(f"{name}_distal")

    for part, length, is_tip in (
        (base, lengths[0], False),
        (proximal, lengths[1], False),
        (middle, lengths[2], False),
        (distal, lengths[3], True),
    ):
        _add_link_visuals(
            part,
            length=length,
            width=width,
            thickness=thickness,
            material=shell_material,
            band_material=band_material,
            accent_material=accent_material,
            tip=is_tip,
        )

    model.articulation(
        f"palm_to_{name}_splay",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=base,
        origin=Origin(xyz=root_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=splay_limits[0], upper=splay_limits[1]),
    )
    model.articulation(
        f"{name}_base_flex",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, lengths[0], 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        f"{name}_middle_flex",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.0, lengths[1], 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        f"{name}_tip_flex",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.0, lengths[2], 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.2, lower=0.0, upper=1.35),
    )


def _add_thumb_chain(
    model: ArticulatedObject,
    *,
    palm,
    shell_material,
    band_material,
    accent_material,
) -> None:
    """Side-mounted thumb with a splay base and two serial flexion joints."""

    base_len = 0.033
    prox_len = 0.040
    distal_len = 0.029
    width = 0.021
    thickness = 0.019

    base = model.part("thumb_base")
    proximal = model.part("thumb_proximal")
    distal = model.part("thumb_distal")
    for part, length, is_tip in (
        (base, base_len, False),
        (proximal, prox_len, False),
        (distal, distal_len, True),
    ):
        _add_link_visuals(
            part,
            length=length,
            width=width,
            thickness=thickness,
            material=shell_material,
            band_material=band_material,
            accent_material=accent_material,
            tip=is_tip,
        )

    model.articulation(
        "palm_to_thumb_splay",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=base,
        origin=Origin(xyz=(-0.079, 0.012, 0.043), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.2, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "thumb_base_flex",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, base_len, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.2, velocity=2.8, lower=0.0, upper=1.15),
    )
    model.articulation(
        "thumb_tip_flex",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        origin=Origin(xyz=(0.0, prox_len, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.1, velocity=3.0, lower=0.0, upper=1.25),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anthropomorphic_robotic_palm")

    titanium = model.material("satin_titanium", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("matte_black_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite_palm", rgba=(0.18, 0.20, 0.22, 1.0))
    blue = model.material("blue_joint_inlay", rgba=(0.04, 0.24, 0.85, 1.0))

    palm = model.part("palm")
    palm.visual(
        mesh_from_cadquery(_rounded_box((0.140, 0.170, 0.036), 0.014), "palm_core"),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=graphite,
        name="palm_block",
    )
    palm.visual(
        mesh_from_cadquery(_rounded_box((0.090, 0.045, 0.030), 0.009), "wrist_cuff"),
        origin=Origin(xyz=(0.0, -0.104, 0.020)),
        material=dark,
        name="wrist_cuff",
    )

    finger_roots = {
        "index": (-0.048, 0.093),
        "middle": (-0.016, 0.093),
        "ring": (0.016, 0.093),
        "little": (0.048, 0.093),
    }
    for root_name, (x, y) in finger_roots.items():
        palm.visual(
            mesh_from_cadquery(_rounded_box((0.024, 0.026, 0.018), 0.004), f"{root_name}_root"),
            origin=Origin(xyz=(x, y, 0.043)),
            material=dark,
            name=f"{root_name}_root",
        )
        palm.visual(
            Box((0.014, 0.004, 0.004)),
            origin=Origin(xyz=(x, 0.106, 0.052)),
            material=blue,
            name=f"{root_name}_axis_mark",
        )

    palm.visual(
        mesh_from_cadquery(_rounded_box((0.018, 0.036, 0.020), 0.004), "thumb_root"),
        origin=Origin(xyz=(-0.070, 0.012, 0.043)),
        material=dark,
        name="thumb_root",
    )
    palm.visual(
        Box((0.004, 0.020, 0.004)),
        origin=Origin(xyz=(-0.079, 0.012, 0.052)),
        material=blue,
        name="thumb_axis_mark",
    )

    finger_specs = {
        "index": {
            "root": (-0.048, 0.106, 0.043),
            "lengths": (0.032, 0.045, 0.030, 0.023),
            "width": 0.018,
            "thickness": 0.018,
            "splay": (-0.42, 0.34),
        },
        "middle": {
            "root": (-0.016, 0.106, 0.043),
            "lengths": (0.034, 0.052, 0.034, 0.025),
            "width": 0.019,
            "thickness": 0.0185,
            "splay": (-0.30, 0.30),
        },
        "ring": {
            "root": (0.016, 0.106, 0.043),
            "lengths": (0.033, 0.048, 0.032, 0.024),
            "width": 0.0185,
            "thickness": 0.018,
            "splay": (-0.34, 0.36),
        },
        "little": {
            "root": (0.048, 0.106, 0.043),
            "lengths": (0.030, 0.039, 0.027, 0.021),
            "width": 0.0165,
            "thickness": 0.017,
            "splay": (-0.36, 0.46),
        },
    }
    for finger_name, spec in finger_specs.items():
        _add_finger_chain(
            model,
            palm=palm,
            name=finger_name,
            root_xyz=spec["root"],
            lengths=spec["lengths"],
            width=spec["width"],
            thickness=spec["thickness"],
            splay_limits=spec["splay"],
            shell_material=titanium,
            band_material=dark,
            accent_material=blue,
        )

    _add_thumb_chain(
        model,
        palm=palm,
        shell_material=titanium,
        band_material=dark,
        accent_material=blue,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    digits = ("index", "middle", "ring", "little", "thumb")
    splay_joints = [object_model.get_articulation(f"palm_to_{digit}_splay") for digit in digits]
    flex_joints = [
        joint
        for joint in object_model.articulations
        if joint.name.endswith("_flex")
    ]

    ctx.check(
        "five independently splayed digit roots",
        len(splay_joints) == 5
        and all(j.axis == (0.0, 0.0, 1.0) for j in splay_joints)
        and all(j.motion_limits.lower < 0.0 < j.motion_limits.upper for j in splay_joints),
        details=f"splay_joints={[j.name for j in splay_joints]}",
    )
    ctx.check(
        "serial transverse flexion joints",
        len(flex_joints) == 14
        and all(j.axis == (1.0, 0.0, 0.0) for j in flex_joints)
        and all(j.motion_limits.lower == 0.0 and j.motion_limits.upper >= 1.15 for j in flex_joints),
        details=f"flex_joints={[j.name for j in flex_joints]}",
    )

    palm = object_model.get_part("palm")
    for digit in ("index", "middle", "ring", "little"):
        base = object_model.get_part(f"{digit}_base")
        ctx.expect_gap(
            base,
            palm,
            axis="y",
            min_gap=-0.0005,
            max_gap=0.002,
            positive_elem="link_shell",
            negative_elem=f"{digit}_root",
            name=f"{digit} base seats at palm root",
        )

    thumb_base = object_model.get_part("thumb_base")
    ctx.expect_gap(
        palm,
        thumb_base,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.002,
        positive_elem="thumb_root",
        negative_elem="link_shell",
        name="thumb base seats at side root",
    )

    index_distal = object_model.get_part("index_distal")
    rest_index_pos = ctx.part_world_position(index_distal)
    with ctx.pose({"palm_to_index_splay": 0.30}):
        splayed_index_pos = ctx.part_world_position(index_distal)
    ctx.check(
        "index splay moves the distal chain laterally",
        rest_index_pos is not None
        and splayed_index_pos is not None
        and abs(splayed_index_pos[0] - rest_index_pos[0]) > 0.012,
        details=f"rest={rest_index_pos}, splayed={splayed_index_pos}",
    )

    middle_tip = object_model.get_part("middle_distal")
    rest_tip_pos = ctx.part_world_position(middle_tip)
    with ctx.pose({"middle_base_flex": 0.85, "middle_middle_flex": 0.70, "middle_tip_flex": 0.55}):
        curled_tip_pos = ctx.part_world_position(middle_tip)
    ctx.check(
        "positive flexion curls the finger out of the palm plane",
        rest_tip_pos is not None
        and curled_tip_pos is not None
        and curled_tip_pos[2] > rest_tip_pos[2] + 0.035,
        details=f"rest={rest_tip_pos}, curled={curled_tip_pos}",
    )

    return ctx.report()


object_model = build_object_model()
