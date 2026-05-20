from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HINGE_RADIUS = 0.024
PIN_RADIUS = 0.009
BARREL_LEN = 0.062
OUTER_KNUCKLE_LEN = 0.032
OUTER_KNUCKLE_Y = 0.052
HINGE_TOTAL_WIDTH = 0.150
CYL_ALONG_Y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _cyl_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_parent_hinge(part, *, x: float, z: float, stem_height: float, name: str, material: str) -> None:
    """Two outside knuckles and a visible full-width pin on a fixed bracket."""
    for side, y in (("a", -OUTER_KNUCKLE_Y), ("b", OUTER_KNUCKLE_Y)):
        part.visual(
            Box((0.056, OUTER_KNUCKLE_LEN, stem_height)),
            origin=Origin(xyz=(x, y, z + stem_height * 0.32)),
            material=material,
            name=f"{name}_lug_{side}",
        )
        part.visual(
            Cylinder(radius=HINGE_RADIUS, length=OUTER_KNUCKLE_LEN),
            origin=_cyl_y(x, y, z),
            material=material,
            name=f"{name}_knuckle_{side}",
        )
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=HINGE_TOTAL_WIDTH),
        origin=_cyl_y(x, 0.0, z),
        material="pin_steel",
        name=f"{name}_pin",
    )


def _add_finger_link(
    part,
    *,
    length: float,
    width: float,
    material: str,
    base_name: str,
    has_distal_hinge: bool,
    pad_kind: str | None = None,
) -> None:
    """Rigid hanging link with a central proximal barrel and optional forked distal hinge."""
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=BARREL_LEN),
        origin=CYL_ALONG_Y,
        material=material,
        name="proximal_barrel",
    )
    part.visual(
        Box((width, 0.043, length - 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * length + 0.012)),
        material=material,
        name=f"{base_name}_web",
    )

    if has_distal_hinge:
        part.visual(
            Box((width * 1.12, 0.136, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -length + 0.047)),
            material=material,
            name=f"{base_name}_fork_bridge",
        )
        for side, y in (("a", -OUTER_KNUCKLE_Y), ("b", OUTER_KNUCKLE_Y)):
            part.visual(
                Box((width * 1.08, OUTER_KNUCKLE_LEN, 0.065)),
                origin=Origin(xyz=(0.0, y, -length + 0.012)),
                material=material,
                name=f"{base_name}_fork_lug_{side}",
            )
            part.visual(
                Cylinder(radius=HINGE_RADIUS, length=OUTER_KNUCKLE_LEN),
                origin=_cyl_y(0.0, y, -length),
                material=material,
                name=f"{base_name}_distal_knuckle_{side}",
            )
        part.visual(
            Cylinder(radius=PIN_RADIUS, length=HINGE_TOTAL_WIDTH),
            origin=_cyl_y(0.0, 0.0, -length),
            material="pin_steel",
            name=f"{base_name}_distal_pin",
        )
    else:
        if pad_kind == "rounded":
            part.visual(
                Cylinder(radius=0.036, length=0.060),
                origin=_cyl_y(0.0, 0.0, -length - 0.012),
                material="rubber",
                name=f"{base_name}_round_pad",
            )
            part.visual(
                Box((width * 0.95, 0.050, 0.052)),
                origin=Origin(xyz=(0.0, 0.0, -length + 0.020)),
                material=material,
                name=f"{base_name}_pad_shank",
            )
        else:
            part.visual(
                Box((0.080, 0.072, 0.048)),
                origin=Origin(xyz=(0.0, 0.0, -length - 0.006)),
                material="rubber",
                name=f"{base_name}_flat_pad",
            )
            part.visual(
                Box((width, 0.050, 0.052)),
                origin=Origin(xyz=(0.0, 0.0, -length + 0.020)),
                material=material,
                name=f"{base_name}_pad_shank",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_dual_finger_rig")
    model.material("dark_anodized", rgba=(0.055, 0.060, 0.070, 1.0))
    model.material("palm_blue", rgba=(0.08, 0.18, 0.34, 1.0))
    model.material("link_warm_aluminum", rgba=(0.72, 0.67, 0.56, 1.0))
    model.material("link_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("pin_steel", rgba=(0.84, 0.82, 0.76, 1.0))
    model.material("rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    model.material("fastener", rgba=(0.12, 0.12, 0.13, 1.0))

    top = model.part("top_support")
    top.visual(
        Box((0.560, 0.160, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material="dark_anodized",
        name="top_rail",
    )
    top.visual(
        Box((0.250, 0.115, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material="dark_anodized",
        name="center_drop",
    )
    top.visual(
        Box((0.390, 0.190, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material="palm_blue",
        name="palm_block",
    )
    top.visual(
        Box((0.440, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, -0.095, 0.080)),
        material="dark_anodized",
        name="rear_edge_rail",
    )
    top.visual(
        Box((0.440, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, 0.095, 0.080)),
        material="dark_anodized",
        name="front_edge_rail",
    )
    for i, x in enumerate((-0.205, -0.070, 0.070, 0.205)):
        top.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.252)),
            material="fastener",
            name=f"top_bolt_{i}",
        )

    finger_x = (-0.110, 0.125)
    _add_parent_hinge(top, x=finger_x[0], z=0.000, stem_height=0.068, name="finger_0_base", material="palm_blue")
    _add_parent_hinge(top, x=finger_x[1], z=0.000, stem_height=0.068, name="finger_1_base", material="palm_blue")

    f0_base = model.part("finger_0_base")
    _add_finger_link(
        f0_base,
        length=0.155,
        width=0.052,
        material="link_warm_aluminum",
        base_name="finger_0_base",
        has_distal_hinge=True,
    )
    f0_mid = model.part("finger_0_mid")
    _add_finger_link(
        f0_mid,
        length=0.130,
        width=0.048,
        material="link_warm_aluminum",
        base_name="finger_0_mid",
        has_distal_hinge=True,
    )
    f0_tip = model.part("finger_0_tip")
    _add_finger_link(
        f0_tip,
        length=0.115,
        width=0.044,
        material="link_warm_aluminum",
        base_name="finger_0_tip",
        has_distal_hinge=False,
        pad_kind="flat",
    )

    f1_base = model.part("finger_1_base")
    _add_finger_link(
        f1_base,
        length=0.135,
        width=0.064,
        material="link_graphite",
        base_name="finger_1_base",
        has_distal_hinge=True,
    )
    f1_mid = model.part("finger_1_mid")
    _add_finger_link(
        f1_mid,
        length=0.105,
        width=0.058,
        material="link_graphite",
        base_name="finger_1_mid",
        has_distal_hinge=True,
    )
    f1_tip = model.part("finger_1_tip")
    _add_finger_link(
        f1_tip,
        length=0.090,
        width=0.052,
        material="link_graphite",
        base_name="finger_1_tip",
        has_distal_hinge=False,
        pad_kind="rounded",
    )

    model.articulation(
        "finger_0_knuckle",
        ArticulationType.REVOLUTE,
        parent=top,
        child=f0_base,
        origin=Origin(xyz=(finger_x[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.5, lower=-0.55, upper=0.95),
    )
    model.articulation(
        "finger_0_middle",
        ArticulationType.REVOLUTE,
        parent=f0_base,
        child=f0_mid,
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.8, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "finger_0_tip_joint",
        ArticulationType.REVOLUTE,
        parent=f0_mid,
        child=f0_tip,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.20, upper=0.95),
    )
    model.articulation(
        "finger_1_knuckle",
        ArticulationType.REVOLUTE,
        parent=top,
        child=f1_base,
        origin=Origin(xyz=(finger_x[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "finger_1_middle",
        ArticulationType.REVOLUTE,
        parent=f1_base,
        child=f1_mid,
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.8, lower=-0.35, upper=0.90),
    )
    model.articulation(
        "finger_1_tip_joint",
        ArticulationType.REVOLUTE,
        parent=f1_mid,
        child=f1_tip,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.25, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top = object_model.get_part("top_support")

    pin_pairs = (
        ("top_support", "finger_0_base", "finger_0_base_pin", "proximal_barrel"),
        ("finger_0_base", "finger_0_mid", "finger_0_base_distal_pin", "proximal_barrel"),
        ("finger_0_mid", "finger_0_tip", "finger_0_mid_distal_pin", "proximal_barrel"),
        ("top_support", "finger_1_base", "finger_1_base_pin", "proximal_barrel"),
        ("finger_1_base", "finger_1_mid", "finger_1_base_distal_pin", "proximal_barrel"),
        ("finger_1_mid", "finger_1_tip", "finger_1_mid_distal_pin", "proximal_barrel"),
    )
    for parent, child, pin_elem, barrel_elem in pin_pairs:
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin_elem,
            elem_b=barrel_elem,
            reason="The visible hinge pin is intentionally modeled passing through the child barrel proxy.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=pin_elem,
            outer_elem=barrel_elem,
            margin=0.001,
            name=f"{pin_elem} runs inside {child} barrel",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=pin_elem,
            elem_b=barrel_elem,
            min_overlap=0.055,
            name=f"{pin_elem} spans {child} barrel",
        )

    revolutes = [joint for joint in object_model.articulations if joint.articulation_type == ArticulationType.REVOLUTE]
    ctx.check(
        "two independent three-joint fingers",
        len(revolutes) == 6 and all(joint.mimic is None for joint in revolutes),
        details=f"revolute_count={len(revolutes)}",
    )
    for prefix in ("finger_0", "finger_1"):
        chain = [object_model.get_articulation(f"{prefix}_knuckle")]
        chain.append(object_model.get_articulation(f"{prefix}_middle"))
        chain.append(object_model.get_articulation(f"{prefix}_tip_joint"))
        ctx.check(
            f"{prefix} has three revolute joints",
            all(joint.articulation_type == ArticulationType.REVOLUTE for joint in chain),
            details=str([joint.name for joint in chain]),
        )

    for name in ("finger_0_base", "finger_1_base"):
        ctx.expect_gap(
            top,
            name,
            axis="z",
            min_gap=0.005,
            positive_elem="palm_block",
            negative_elem="proximal_barrel",
            name=f"{name} hangs under palm block",
        )

    f0_tip = object_model.get_part("finger_0_tip")
    f1_tip = object_model.get_part("finger_1_tip")
    rest_0 = ctx.part_world_position(f0_tip)
    rest_1 = ctx.part_world_position(f1_tip)
    with ctx.pose({"finger_0_knuckle": 0.55, "finger_0_middle": 0.45, "finger_0_tip_joint": 0.30}):
        curled_0 = ctx.part_world_position(f0_tip)
        steady_1 = ctx.part_world_position(f1_tip)
    ctx.check(
        "finger_0 curls without driving finger_1",
        rest_0 is not None
        and rest_1 is not None
        and curled_0 is not None
        and steady_1 is not None
        and curled_0[0] < rest_0[0] - 0.035
        and abs(steady_1[0] - rest_1[0]) < 0.001,
        details=f"rest_0={rest_0}, curled_0={curled_0}, rest_1={rest_1}, steady_1={steady_1}",
    )

    return ctx.report()


object_model = build_object_model()
