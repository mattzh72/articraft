from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_hinge_tv_wall_mount")

    black = model.material("black_powdercoat", rgba=(0.015, 0.016, 0.017, 1.0))
    dark = model.material("dark_graphite", rgba=(0.09, 0.095, 0.10, 1.0))
    edge = model.material("worn_edge_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    screw = model.material("black_recess", rgba=(0.0, 0.0, 0.0, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.014, 0.280, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="wall_plate_shell",
    )
    # Flush dark screw pockets on the exposed front face of the wall plate.
    for i, (y, z) in enumerate(((-0.075, 0.145), (0.075, 0.145), (-0.075, -0.145), (0.075, -0.145))):
        wall_plate.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(0.009, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=screw,
            name=f"wall_screw_{i}",
        )

    wall_hinge_x = 0.045
    wall_hinge_y = 0.155
    for hinge_name, web_name, z in (
        ("wall_hinge_top", "wall_hinge_web_top", 0.047),
        ("wall_hinge_bottom", "wall_hinge_web_bottom", -0.047),
    ):
        wall_plate.visual(
            Cylinder(radius=0.033, length=0.042),
            origin=Origin(xyz=(wall_hinge_x, wall_hinge_y, z)),
            material=dark,
            name=hinge_name,
        )
        wall_plate.visual(
            Box((0.052, 0.052, 0.026)),
            origin=Origin(xyz=(0.019, 0.143, z)),
            material=dark,
            name=web_name,
        )

    outer_arm = model.part("outer_arm")
    _add_arm_link(outer_arm, length=0.320, material=dark, cap_material=edge)

    inner_arm = model.part("inner_arm")
    _add_arm_link(inner_arm, length=0.270, material=dark, cap_material=edge)

    swivel_support = model.part("swivel_support")
    swivel_support.visual(
        Cylinder(radius=0.027, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="swivel_boss",
    )
    for cap_name, z in (
        ("swivel_pin_cap_top", 0.0225),
        ("swivel_pin_cap_bottom", -0.0225),
    ):
        swivel_support.visual(
            Cylinder(radius=0.020, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=edge,
            name=cap_name,
        )
    swivel_support.visual(
        Box((0.075, 0.052, 0.032)),
        origin=Origin(xyz=(0.0375, 0.0, 0.0)),
        material=dark,
        name="swivel_neck",
    )
    swivel_support.visual(
        Box((0.040, 0.105, 0.030)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=dark,
        name="tilt_yoke_bridge",
    )
    for cheek_name, cap_name, y in (
        ("tilt_cheek_upper", "tilt_pivot_cap_upper", 0.043),
        ("tilt_cheek_lower", "tilt_pivot_cap_lower", -0.043),
    ):
        swivel_support.visual(
            Box((0.060, 0.018, 0.080)),
            origin=Origin(xyz=(0.105, y, 0.0)),
            material=dark,
            name=cheek_name,
        )
        swivel_support.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(0.126, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=edge,
            name=cap_name,
        )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.020, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tilt_barrel",
    )
    tilt_head.visual(
        Box((0.080, 0.052, 0.040)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=dark,
        name="head_neck",
    )
    tilt_head.visual(
        Box((0.014, 0.220, 0.160)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=black,
        name="vesa_plate",
    )
    for i, (y, z) in enumerate(((-0.050, 0.035), (0.050, 0.035), (-0.050, -0.035), (0.050, -0.035))):
        tilt_head.visual(
            Cylinder(radius=0.008, length=0.003),
            origin=Origin(xyz=(0.0935, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=screw,
            name=f"vesa_hole_{i}",
        )
    tilt_head.visual(
        Box((0.012, 0.180, 0.016)),
        origin=Origin(xyz=(0.095, 0.0, 0.067)),
        material=edge,
        name="top_lip",
    )
    tilt_head.visual(
        Box((0.012, 0.180, 0.016)),
        origin=Origin(xyz=(0.095, 0.0, -0.067)),
        material=edge,
        name="bottom_lip",
    )

    model.articulation(
        "wall_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=outer_arm,
        origin=Origin(xyz=(wall_hinge_x, wall_hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=inner_arm,
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=swivel_support,
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_support,
        child=tilt_head,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-0.35, upper=0.35),
    )

    return model


def _add_arm_link(part, *, length: float, material, cap_material) -> None:
    """Dog-bone steel arm with interleaved vertical hinge knuckles at each end."""
    bar_length = length - 0.040
    part.visual(
        Box((bar_length, 0.064, 0.040)),
        origin=Origin(xyz=(bar_length / 2.0, 0.0, 0.0)),
        material=material,
        name="arm_bar",
    )
    part.visual(
        Cylinder(radius=0.027, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="prox_boss",
    )
    # Small metal caps make the retained hinge pin legible without adding a separate joint.
    for cap_name, z in (
        ("prox_pin_cap_top", 0.0225),
        ("prox_pin_cap_bottom", -0.0225),
    ):
        part.visual(
            Cylinder(radius=0.020, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=cap_material,
            name=cap_name,
        )

    for hinge_name, z, strap_z in (
        ("distal_hinge_top", 0.047, 0.027),
        ("distal_hinge_bottom", -0.047, -0.027),
    ):
        part.visual(
            Cylinder(radius=0.032, length=0.042),
            origin=Origin(xyz=(length, 0.0, z)),
            material=material,
            name=hinge_name,
        )
        for side, y in (("a", 0.038), ("b", -0.038)):
            part.visual(
                Box((0.085, 0.016, 0.018)),
                origin=Origin(xyz=(length - 0.032, y, strap_z)),
                material=material,
                name=f"{hinge_name}_strap_{side}",
            )
    for cap_name, z in (
        ("distal_pin_cap_top", 0.071),
        ("distal_pin_cap_bottom", -0.071),
    ):
        part.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(length, 0.0, z)),
            material=cap_material,
            name=cap_name,
        )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    outer_arm = object_model.get_part("outer_arm")
    inner_arm = object_model.get_part("inner_arm")
    swivel_support = object_model.get_part("swivel_support")
    tilt_head = object_model.get_part("tilt_head")

    wall_hinge = object_model.get_articulation("wall_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")
    joints = (wall_hinge, elbow_hinge, head_swivel, head_tilt)

    ctx.check(
        "four revolute mount axes",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    with ctx.pose({wall_hinge: 0.0, elbow_hinge: 0.0, head_swivel: 0.0, head_tilt: 0.0}):
        ctx.expect_gap(
            wall_plate,
            outer_arm,
            axis="z",
            positive_elem="wall_hinge_top",
            negative_elem="prox_pin_cap_top",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="wall hinge knuckles meet",
        )
        ctx.expect_gap(
            outer_arm,
            inner_arm,
            axis="z",
            positive_elem="distal_hinge_top",
            negative_elem="prox_pin_cap_top",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="elbow hinge knuckles meet",
        )
        ctx.expect_gap(
            inner_arm,
            swivel_support,
            axis="z",
            positive_elem="distal_hinge_top",
            negative_elem="swivel_pin_cap_top",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="swivel hinge knuckles meet",
        )
        ctx.expect_gap(
            swivel_support,
            tilt_head,
            axis="y",
            positive_elem="tilt_cheek_upper",
            negative_elem="tilt_barrel",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="tilt barrel is captured by yoke",
        )
        rest_head = ctx.part_world_position(tilt_head)
        ctx.check(
            "head projects away from wall without screen",
            rest_head is not None and rest_head[0] > 0.70,
            details=f"tilt head origin at {rest_head}",
        )

    rest_head = ctx.part_world_position(tilt_head)
    with ctx.pose({elbow_hinge: 1.0}):
        folded_head = ctx.part_world_position(tilt_head)
    ctx.check(
        "elbow folds head sideways",
        rest_head is not None
        and folded_head is not None
        and folded_head[1] > rest_head[1] + 0.12,
        details=f"rest={rest_head}, folded={folded_head}",
    )

    with ctx.pose({head_swivel: 0.85}):
        swiveled_head = ctx.part_world_position(tilt_head)
    ctx.check(
        "swivel turns compact head",
        rest_head is not None
        and swiveled_head is not None
        and swiveled_head[1] > rest_head[1] + 0.06,
        details=f"rest={rest_head}, swiveled={swiveled_head}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(tilt_head, elem="vesa_plate")
    with ctx.pose({head_tilt: 0.35}):
        tilted_plate_aabb = ctx.part_element_world_aabb(tilt_head, elem="vesa_plate")
    rest_plate_z = None if rest_plate_aabb is None else (rest_plate_aabb[0][2] + rest_plate_aabb[1][2]) / 2.0
    tilted_plate_z = None if tilted_plate_aabb is None else (tilted_plate_aabb[0][2] + tilted_plate_aabb[1][2]) / 2.0
    ctx.check(
        "tilt pitches vesa plate",
        rest_plate_z is not None and tilted_plate_z is not None and abs(tilted_plate_z - rest_plate_z) > 0.025,
        details=f"rest_z={rest_plate_z}, tilted_z={tilted_plate_z}",
    )

    return ctx.report()


object_model = build_object_model()
