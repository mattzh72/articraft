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


PIN_RPY = (-math.pi / 2.0, 0.0, 0.0)
ROOT_HINGE_Z = 0.14


def _y_cylinder(part, name, radius, length, xyz, material):
    """Add a cylinder whose axis follows the local/world Y direction."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=PIN_RPY),
        material=material,
        name=name,
    )


def _add_fold_link(
    part,
    *,
    length: float,
    eye_radius: float,
    eye_width: float,
    web_width: float,
    web_height: float,
    fork_gap: float,
    fork_thickness: float,
    pin_radius: float,
    body_material,
    pin_material,
):
    """Build one tapered-size link with a proximal tongue and distal clevis fork."""
    fork_outer_width = fork_gap + 2.0 * fork_thickness
    cheek_y = fork_gap / 2.0 + fork_thickness / 2.0

    _y_cylinder(
        part,
        "proximal_eye",
        eye_radius,
        eye_width,
        (0.0, 0.0, 0.0),
        body_material,
    )

    web_start = eye_radius * 0.62
    web_end = length - 0.066
    part.visual(
        Box((web_end - web_start, web_width, web_height)),
        origin=Origin(xyz=((web_start + web_end) / 2.0, 0.0, 0.0)),
        material=body_material,
        name="tapered_web",
    )

    part.visual(
        Box((0.030, fork_outer_width, web_height * 0.92)),
        origin=Origin(xyz=(length - 0.060, 0.0, 0.0)),
        material=body_material,
        name="fork_bridge",
    )

    for suffix, y in (("0", cheek_y), ("1", -cheek_y)):
        part.visual(
            Box((0.078, fork_thickness, web_height)),
            origin=Origin(xyz=(length - 0.006, y, 0.0)),
            material=body_material,
            name=f"fork_cheek_{suffix}",
        )
        _y_cylinder(
            part,
            f"fork_boss_{suffix}",
            web_height * 0.55,
            fork_thickness,
            (length, y, 0.0),
            body_material,
        )

    _y_cylinder(
        part,
        "distal_pin",
        pin_radius,
        fork_outer_width + 0.018,
        (length, 0.0, 0.0),
        pin_material,
    )
    for suffix, y in (("0", fork_outer_width / 2.0 + 0.006), ("1", -fork_outer_width / 2.0 - 0.006)):
        _y_cylinder(
            part,
            f"pin_head_{suffix}",
            pin_radius * 1.75,
            0.006,
            (length, y, 0.0),
            pin_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_fold_out_arm")

    bridge_paint = model.material("warm_galvanized_bridge", rgba=(0.47, 0.50, 0.48, 1.0))
    dark_steel = model.material("dark_phosphate_pins", rgba=(0.04, 0.045, 0.045, 1.0))
    root_paint = model.material("black_clevis_paint", rgba=(0.015, 0.017, 0.018, 1.0))
    link_0_paint = model.material("deep_blue_powdercoat", rgba=(0.05, 0.13, 0.30, 1.0))
    link_1_paint = model.material("blue_grey_powdercoat", rgba=(0.11, 0.20, 0.33, 1.0))
    link_2_paint = model.material("pale_blue_grey_powdercoat", rgba=(0.22, 0.31, 0.42, 1.0))
    platform_paint = model.material("matte_black_platform", rgba=(0.02, 0.022, 0.024, 1.0))

    support = model.part("bridge_support")
    support.visual(
        Box((0.30, 0.42, 0.050)),
        origin=Origin(xyz=(-0.155, 0.0, 0.320)),
        material=bridge_paint,
        name="bridge_beam",
    )
    support.visual(
        Box((0.145, 0.465, 0.018)),
        origin=Origin(xyz=(-0.155, 0.0, 0.354)),
        material=root_paint,
        name="upper_clamp_plate",
    )
    support.visual(
        Box((0.145, 0.465, 0.018)),
        origin=Origin(xyz=(-0.155, 0.0, 0.280)),
        material=root_paint,
        name="lower_clamp_plate",
    )
    support.visual(
        Box((0.050, 0.110, 0.176)),
        origin=Origin(xyz=(-0.073, 0.0, 0.210)),
        material=root_paint,
        name="drop_web",
    )
    for suffix, y in (("0", 0.052), ("1", -0.052)):
        support.visual(
            Box((0.110, 0.014, 0.090)),
            origin=Origin(xyz=(-0.006, y, ROOT_HINGE_Z)),
            material=root_paint,
            name=f"root_cheek_{suffix}",
        )
        support.visual(
            Box((0.072, 0.012, 0.090)),
            origin=Origin(xyz=(-0.060, y, ROOT_HINGE_Z)),
            material=root_paint,
            name=f"cheek_backbone_{suffix}",
        )
    support.visual(
        Box((0.050, 0.108, 0.032)),
        origin=Origin(xyz=(-0.073, 0.0, ROOT_HINGE_Z - 0.046)),
        material=root_paint,
        name="clevis_saddle",
    )
    _y_cylinder(
        support,
        "root_pin",
        0.0065,
        0.132,
        (0.0, 0.0, ROOT_HINGE_Z),
        dark_steel,
    )
    for suffix, y in (("0", 0.0695), ("1", -0.0695)):
        _y_cylinder(
            support,
            f"root_pin_head_{suffix}",
            0.012,
            0.007,
            (0.0, y, ROOT_HINGE_Z),
            dark_steel,
        )
    for x in (-0.215, -0.095):
        for y in (-0.155, 0.155):
            support.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, y, 0.366)),
                material=dark_steel,
                name=f"clamp_bolt_{'p' if x > -0.15 else 'n'}_{'p' if y > 0 else 'n'}",
            )

    link_0 = model.part("link_0")
    _add_fold_link(
        link_0,
        length=0.380,
        eye_radius=0.034,
        eye_width=0.026,
        web_width=0.024,
        web_height=0.052,
        fork_gap=0.032,
        fork_thickness=0.015,
        pin_radius=0.0058,
        body_material=link_0_paint,
        pin_material=dark_steel,
    )

    link_1 = model.part("link_1")
    _add_fold_link(
        link_1,
        length=0.300,
        eye_radius=0.028,
        eye_width=0.021,
        web_width=0.020,
        web_height=0.043,
        fork_gap=0.027,
        fork_thickness=0.013,
        pin_radius=0.0052,
        body_material=link_1_paint,
        pin_material=dark_steel,
    )

    link_2 = model.part("link_2")
    _add_fold_link(
        link_2,
        length=0.230,
        eye_radius=0.023,
        eye_width=0.017,
        web_width=0.016,
        web_height=0.035,
        fork_gap=0.023,
        fork_thickness=0.011,
        pin_radius=0.0045,
        body_material=link_2_paint,
        pin_material=dark_steel,
    )

    platform = model.part("platform_bracket")
    _y_cylinder(
        platform,
        "proximal_eye",
        0.0185,
        0.013,
        (0.0, 0.0, 0.0),
        platform_paint,
    )
    platform.visual(
        Box((0.092, 0.013, 0.026)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=platform_paint,
        name="short_neck",
    )
    platform.visual(
        Box((0.118, 0.082, 0.012)),
        origin=Origin(xyz=(0.136, 0.0, -0.009)),
        material=platform_paint,
        name="platform_plate",
    )
    for suffix, y in (("0", 0.045), ("1", -0.045)):
        platform.visual(
            Box((0.104, 0.008, 0.028)),
            origin=Origin(xyz=(0.136, y, 0.003)),
            material=platform_paint,
            name=f"platform_flange_{suffix}",
        )
    for ix, x in enumerate((0.106, 0.166)):
        for iy, y in enumerate((-0.024, 0.024)):
            platform.visual(
                Cylinder(radius=0.0055, length=0.004),
                origin=Origin(xyz=(x, y, -0.001)),
                material=dark_steel,
                name=f"platform_bolt_{ix}_{iy}",
            )

    model.articulation(
        "bridge_to_link_0",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, ROOT_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=36.0, velocity=1.4, lower=-0.35, upper=1.75),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.7, lower=-2.25, upper=2.25),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.9, lower=-2.30, upper=2.30),
    )
    model.articulation(
        "link_2_to_platform",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=platform,
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-1.80, upper=1.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hinge_pairs = (
        ("bridge_support", "link_0", "root_pin"),
        ("link_0", "link_1", "distal_pin"),
        ("link_1", "link_2", "distal_pin"),
        ("link_2", "platform_bracket", "distal_pin"),
    )
    for parent_name, child_name, pin_name in hinge_pairs:
        ctx.allow_overlap(
            parent_name,
            child_name,
            elem_a=pin_name,
            elem_b="proximal_eye",
            reason="The hinge shaft is intentionally shown captured through a simplified solid eye/bushing.",
        )
        ctx.expect_within(
            parent_name,
            child_name,
            axes="xz",
            inner_elem=pin_name,
            outer_elem="proximal_eye",
            margin=0.001,
            name=f"{parent_name} pin is centered in {child_name} eye",
        )
        ctx.expect_overlap(
            parent_name,
            child_name,
            axes="xyz",
            elem_a=pin_name,
            elem_b="proximal_eye",
            min_overlap=0.006,
            name=f"{parent_name} pin passes through {child_name} eye",
        )

    joint_names = (
        "bridge_to_link_0",
        "link_0_to_link_1",
        "link_1_to_link_2",
        "link_2_to_platform",
    )
    ctx.check(
        "four revolute joints in series",
        len(object_model.articulations) == 4
        and all(object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE for name in joint_names),
    )

    def z_size(part_name: str):
        aabb = ctx.part_world_aabb(object_model.get_part(part_name))
        if aabb is None:
            return None
        lo, hi = aabb
        return hi[2] - lo[2]

    taper = [z_size(name) for name in ("link_0", "link_1", "link_2", "platform_bracket")]
    ctx.check(
        "visible sections taper distally",
        all(value is not None for value in taper)
        and taper[0] > taper[1] > taper[2] > taper[3],
        details=f"z extents were {taper}",
    )

    platform = object_model.get_part("platform_bracket")
    rest_pos = ctx.part_world_position(platform)
    with ctx.pose(
        {
            "bridge_to_link_0": 0.75,
            "link_0_to_link_1": -1.05,
            "link_1_to_link_2": 0.95,
            "link_2_to_platform": -0.55,
        }
    ):
        folded_pos = ctx.part_world_position(platform)

    ctx.check(
        "folded pose moves distal bracket upward and inward",
        rest_pos is not None
        and folded_pos is not None
        and folded_pos[2] > rest_pos[2] + 0.14
        and folded_pos[0] < rest_pos[0] - 0.08,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
