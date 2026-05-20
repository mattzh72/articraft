from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HINGE_AXIS = (0.0, 1.0, 0.0)
CHEEK_Y = 0.021
CHEEK_THICKNESS = 0.012
CHEEK_LENGTH = 0.036
CHEEK_HEIGHT = 0.040
CHEEK_CENTER_X_OFFSET = -0.010
BOSS_RADIUS = 0.0105
BOSS_LENGTH = 0.024


def _signed_power(value: float, power: float) -> float:
    return math.copysign(abs(value) ** power, value)


def _superellipse_section(
    x: float,
    width: float,
    height: float,
    *,
    segments: int = 36,
    exponent: float = 2.7,
) -> list[tuple[float, float, float]]:
    """Rounded-rect phalanx section.

    LoftGeometry expects every section as an XY loop at constant Z.  The mesh is
    therefore authored along local +Z, then _phalanx_body rotates it so local +Z
    becomes the phalanx +X axis.
    """
    power = 2.0 / exponent
    points: list[tuple[float, float, float]] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        y = 0.5 * width * _signed_power(math.cos(theta), power)
        z = 0.5 * height * _signed_power(math.sin(theta), power)
        points.append((-z, y, x))
    return points


def _phalanx_body(length: float, width: float, height: float) -> LoftGeometry:
    """Tapered, softly rectangular body that clears the side cheeks at each hinge."""
    sections = [
        _superellipse_section(0.006, width * 0.72, height * 0.76),
        _superellipse_section(0.022, width * 0.86, height * 0.90),
        _superellipse_section(length * 0.42, width, height),
        _superellipse_section(length - 0.032, width * 0.92, height * 0.90),
        _superellipse_section(length - 0.016, width * 0.70, height * 0.74),
    ]
    return LoftGeometry(sections, cap=True, closed=True).rotate_y(math.pi / 2.0)


def _add_hinge_boss(part, *, material, visual_name: str) -> None:
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=visual_name,
    )


def _add_joint_cheeks(part, joint_x: float, prefix: str, *, cheek_material, cap_material) -> None:
    """Add the paired side plates and outer pin caps carried by the parent link."""
    center_x = joint_x + CHEEK_CENTER_X_OFFSET
    part.visual(
        Box((0.014, 0.046, 0.014)),
        origin=Origin(xyz=(joint_x - 0.025, 0.0, -0.012)),
        material=cheek_material,
        name=f"{prefix}_cheek_bridge",
    )
    part.visual(
        Cylinder(radius=0.0042, length=0.058),
        origin=Origin(xyz=(joint_x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cap_material,
        name=f"{prefix}_pin",
    )
    for suffix, y in (("pos", CHEEK_Y), ("neg", -CHEEK_Y)):
        part.visual(
            Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
            origin=Origin(xyz=(center_x, y, 0.0)),
            material=cheek_material,
            name=f"{prefix}_cheek_{suffix}",
        )

        cap_y = math.copysign(CHEEK_Y + 0.007, y)
        part.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(joint_x, cap_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cap_material,
            name=f"{prefix}_pin_cap_{suffix}",
        )


def _add_phalanx(
    model: ArticulatedObject,
    name: str,
    *,
    length: float,
    width: float,
    height: float,
    body_material,
    cheek_material=None,
    cap_material=None,
    distal_cheek_prefix: str | None = None,
):
    part = model.part(name)
    part.visual(
        mesh_from_geometry(_phalanx_body(length, width, height), f"{name}_body"),
        material=body_material,
        name="body",
    )
    _add_hinge_boss(part, material=body_material, visual_name=f"{name}_boss")
    if distal_cheek_prefix is not None:
        _add_joint_cheeks(
            part,
            length,
            distal_cheek_prefix,
            cheek_material=cheek_material,
            cap_material=cap_material,
        )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="segmented_phalanx_chain")

    polymer = model.material("warm_ivory_polymer", color=(0.82, 0.76, 0.64, 1.0))
    side_plate = model.material("brushed_side_plates", color=(0.30, 0.34, 0.36, 1.0))
    dark_pin = model.material("dark_pivot_caps", color=(0.06, 0.065, 0.07, 1.0))
    base_metal = model.material("matte_black_base", color=(0.09, 0.10, 0.105, 1.0))
    rubber = model.material("soft_fingertip_rubber", color=(0.12, 0.105, 0.095, 1.0))

    root_knuckle = model.part("root_knuckle")
    root_knuckle.visual(
        Box((0.085, 0.080, 0.015)),
        origin=Origin(xyz=(-0.030, 0.0, -0.060)),
        material=base_metal,
        name="base_plate",
    )
    root_knuckle.visual(
        Box((0.026, 0.054, 0.046)),
        origin=Origin(xyz=(-0.027, 0.0, -0.037)),
        material=base_metal,
        name="upright_block",
    )
    _add_joint_cheeks(
        root_knuckle,
        0.0,
        "root",
        cheek_material=side_plate,
        cap_material=dark_pin,
    )
    for x in (-0.058, -0.002):
        for y in (-0.026, 0.026):
            root_knuckle.visual(
                Cylinder(radius=0.0045, length=0.003),
                origin=Origin(xyz=(x, y, -0.051), rpy=(0.0, 0.0, 0.0)),
                material=dark_pin,
                name=f"bolt_{x:+.3f}_{y:+.3f}",
            )

    proximal_len = 0.082
    middle_len = 0.058
    distal_len = 0.044

    proximal = _add_phalanx(
        model,
        "proximal",
        length=proximal_len,
        width=0.032,
        height=0.028,
        body_material=polymer,
        cheek_material=side_plate,
        cap_material=dark_pin,
        distal_cheek_prefix="middle",
    )
    middle = _add_phalanx(
        model,
        "middle",
        length=middle_len,
        width=0.028,
        height=0.025,
        body_material=polymer,
        cheek_material=side_plate,
        cap_material=dark_pin,
        distal_cheek_prefix="distal",
    )
    distal = _add_phalanx(
        model,
        "distal",
        length=distal_len,
        width=0.024,
        height=0.022,
        body_material=polymer,
    )
    fingertip = CapsuleGeometry(radius=0.0135, length=0.014, radial_segments=32, height_segments=8)
    fingertip.rotate_y(math.pi / 2.0).translate(distal_len + 0.001, 0.0, 0.0)
    distal.visual(
        mesh_from_geometry(fingertip, "fingertip_pad"),
        material=rubber,
        name="fingertip_pad",
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root_knuckle,
        child=proximal,
        origin=Origin(),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(proximal_len, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(middle_len, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_knuckle")
    proximal = object_model.get_part("proximal")
    middle = object_model.get_part("middle")
    distal = object_model.get_part("distal")

    joint_names = ("root_to_proximal", "proximal_to_middle", "middle_to_distal")
    for joint_name in joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a one-plane revolute hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == HINGE_AXIS
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 1.0,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
        )

    # At the straight pose, every boss is centered between visible side cheeks.
    for parent, child, cheek_prefix, boss_name in (
        (root, proximal, "root", "proximal_boss"),
        (proximal, middle, "middle", "middle_boss"),
        (middle, distal, "distal", "distal_boss"),
    ):
        ctx.allow_overlap(
            parent,
            child,
            elem_a=f"{cheek_prefix}_pin",
            elem_b=boss_name,
            reason="The visible hinge pin is intentionally captured through the phalanx boss.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=f"{cheek_prefix}_pin",
            outer_elem=boss_name,
            margin=0.001,
            name=f"{cheek_prefix} pin is centered in {boss_name}",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=f"{cheek_prefix}_pin",
            elem_b=boss_name,
            min_overlap=0.020,
            name=f"{cheek_prefix} pin passes through {boss_name}",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="xz",
            elem_a=boss_name,
            elem_b=f"{cheek_prefix}_cheek_pos",
            min_overlap=0.012,
            name=f"{boss_name} aligns with positive cheek",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem=f"{cheek_prefix}_cheek_pos",
            negative_elem=boss_name,
            min_gap=0.001,
            max_gap=0.006,
            name=f"{boss_name} clears positive cheek",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            positive_elem=boss_name,
            negative_elem=f"{cheek_prefix}_cheek_neg",
            min_gap=0.001,
            max_gap=0.006,
            name=f"{boss_name} clears negative cheek",
        )

    ctx.expect_overlap(
        distal,
        distal,
        axes="x",
        elem_a="body",
        elem_b="fingertip_pad",
        min_overlap=0.002,
        name="rubber fingertip is seated on distal phalanx",
    )

    distal_rest = ctx.part_world_position(distal)
    with ctx.pose(
        {
            object_model.get_articulation("root_to_proximal"): 0.85,
            object_model.get_articulation("proximal_to_middle"): 0.85,
            object_model.get_articulation("middle_to_distal"): 0.70,
        }
    ):
        distal_curled = ctx.part_world_position(distal)

    ctx.check(
        "chain curls in the x-z bending plane",
        distal_rest is not None
        and distal_curled is not None
        and distal_curled[2] < distal_rest[2] - 0.035
        and abs(distal_curled[1] - distal_rest[1]) < 0.003,
        details=f"rest={distal_rest}, curled={distal_curled}",
    )

    return ctx.report()


object_model = build_object_model()
