from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_prism_mesh(size: tuple[float, float, float], radius: float, name: str):
    """A low, stamped-looking rounded rectangular prism."""
    profile = rounded_rect_profile(size[0], size[1], radius, corner_segments=8)
    geom = ExtrudeGeometry(profile, size[2], center=True)
    return mesh_from_geometry(geom, name)


def _add_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_tv_wall_mount")

    powder_black = model.material("powder_black", rgba=(0.01, 0.012, 0.012, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.038, 0.04, 1.0))
    dark_cover = model.material("dark_polymer_cover", rgba=(0.006, 0.007, 0.008, 1.0))
    cast_grey = model.material("cast_grey", rgba=(0.18, 0.19, 0.19, 1.0))
    fastener = model.material("zinc_fasteners", rgba=(0.62, 0.62, 0.58, 1.0))

    # Commercial long-reach proportions: a heavy wall plate, a long first arm,
    # a shorter second arm, then the VESA-style head and tilt cradle.
    wall_pivot_x = 0.140
    first_len = 0.720
    second_len = 0.420
    head_x = 0.190

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.035, 0.220, 0.500)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        material=powder_black,
        name="wall_back_plate",
    )
    wall_plate.visual(
        Box((0.034, 0.122, 0.390)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=satin_black,
        name="raised_spine",
    )
    wall_plate.visual(
        Box((0.020, 0.082, 0.300)),
        origin=Origin(xyz=(0.034, 0.0, -0.012)),
        material=dark_cover,
        name="wall_cable_cover",
    )
    for i, (y, z) in enumerate(((-0.068, -0.185), (0.068, -0.185), (-0.068, 0.185), (0.068, 0.185))):
        _add_cylinder(
            wall_plate,
            name=f"lag_bolt_{i}",
            radius=0.011,
            length=0.014,
            xyz=(0.030, y, z),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=fastener,
        )
    # Split, fixed knuckles and twin standoff shelves leave a real center gap
    # for the first arm's rotating barrel.
    wall_plate.visual(
        Box((wall_pivot_x - 0.010, 0.118, 0.032)),
        origin=Origin(xyz=((wall_pivot_x + 0.020) / 2.0, 0.0, 0.085)),
        material=cast_grey,
        name="wall_upper_standoff",
    )
    wall_plate.visual(
        Cylinder(radius=0.054, length=0.060),
        origin=Origin(xyz=(wall_pivot_x, 0.0, 0.085)),
        material=cast_grey,
        name="wall_upper_knuckle",
    )
    wall_plate.visual(
        Box((wall_pivot_x - 0.010, 0.118, 0.032)),
        origin=Origin(xyz=((wall_pivot_x + 0.020) / 2.0, 0.0, -0.085)),
        material=cast_grey,
        name="wall_lower_standoff",
    )
    wall_plate.visual(
        Cylinder(radius=0.054, length=0.060),
        origin=Origin(xyz=(wall_pivot_x, 0.0, -0.085)),
        material=cast_grey,
        name="wall_lower_knuckle",
    )

    first_arm = model.part("first_arm")
    first_arm.visual(
        Cylinder(radius=0.047, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_grey,
        name="wall_barrel",
    )
    first_arm.visual(
        Box((0.170, 0.086, 0.054)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=cast_grey,
        name="wall_elbow_casting",
    )
    first_arm.visual(
        Box((0.660, 0.030, 0.030)),
        origin=Origin(xyz=(0.330, 0.0, 0.0)),
        material=satin_black,
        name="first_core_web",
    )
    first_arm.visual(
        _rounded_prism_mesh((first_len - 0.230, 0.074, 0.046), 0.016, "first_channel_mesh"),
        origin=Origin(xyz=(first_len / 2.0 - 0.010, 0.0, 0.0)),
        material=powder_black,
        name="first_channel",
    )
    first_arm.visual(
        Box((0.470, 0.030, 0.010)),
        origin=Origin(xyz=(0.360, 0.0, 0.027)),
        material=dark_cover,
        name="first_cable_lid",
    )
    for side, y in (("side_0", -0.055), ("side_1", 0.055)):
        first_arm.visual(
            Box((0.360, 0.011, 0.012)),
            origin=Origin(xyz=(0.365, y, 0.031)),
            material=satin_black,
            name=f"first_stamped_rib_{side}",
        )
    # Elbow yoke for the second arm: side cheeks are outside the rotating barrel.
    for side, y in (("side_0", -0.061), ("side_1", 0.061)):
        first_arm.visual(
            Box((0.160, 0.018, 0.140)),
            origin=Origin(xyz=(first_len - 0.065, y, 0.0)),
            material=cast_grey,
            name=f"elbow_cheek_{side}",
        )
        first_arm.visual(
            Box((0.070, 0.030, 0.052)),
            origin=Origin(xyz=(first_len - 0.155, y * 0.86, 0.0)),
            material=cast_grey,
            name=f"elbow_web_{side}",
        )
    first_arm.visual(
        Box((0.070, 0.120, 0.030)),
        origin=Origin(xyz=(first_len - 0.095, 0.0, 0.0)),
        material=cast_grey,
        name="elbow_center_web",
    )
    first_arm.visual(
        Cylinder(radius=0.055, length=0.056),
        origin=Origin(xyz=(first_len, 0.0, 0.085)),
        material=cast_grey,
        name="elbow_upper_knuckle",
    )
    first_arm.visual(
        Cylinder(radius=0.055, length=0.056),
        origin=Origin(xyz=(first_len, 0.0, -0.085)),
        material=cast_grey,
        name="elbow_lower_knuckle",
    )

    second_arm = model.part("second_arm")
    second_arm.visual(
        Cylinder(radius=0.040, length=0.114),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_grey,
        name="elbow_barrel",
    )
    second_arm.visual(
        Box((0.150, 0.074, 0.050)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=cast_grey,
        name="elbow_inner_casting",
    )
    second_arm.visual(
        Box((0.350, 0.026, 0.028)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material=satin_black,
        name="second_core_web",
    )
    second_arm.visual(
        _rounded_prism_mesh((second_len - 0.150, 0.064, 0.044), 0.014, "second_channel_mesh"),
        origin=Origin(xyz=(second_len / 2.0 - 0.005, 0.0, 0.0)),
        material=powder_black,
        name="second_channel",
    )
    second_arm.visual(
        Box((0.255, 0.027, 0.009)),
        origin=Origin(xyz=(0.235, 0.0, 0.027)),
        material=dark_cover,
        name="second_cable_lid",
    )
    for side, y in (("side_0", -0.056), ("side_1", 0.056)):
        second_arm.visual(
            Box((0.135, 0.018, 0.128)),
            origin=Origin(xyz=(second_len - 0.055, y, 0.0)),
            material=cast_grey,
            name=f"wrist_cheek_{side}",
        )
        second_arm.visual(
            Box((0.055, 0.028, 0.048)),
            origin=Origin(xyz=(second_len - 0.120, y * 0.84, 0.0)),
            material=cast_grey,
            name=f"wrist_web_{side}",
        )
    second_arm.visual(
        Box((0.060, 0.110, 0.028)),
        origin=Origin(xyz=(second_len - 0.090, 0.0, 0.0)),
        material=cast_grey,
        name="wrist_center_web",
    )
    second_arm.visual(
        Cylinder(radius=0.048, length=0.052),
        origin=Origin(xyz=(second_len, 0.0, 0.077)),
        material=cast_grey,
        name="wrist_upper_knuckle",
    )
    second_arm.visual(
        Cylinder(radius=0.048, length=0.052),
        origin=Origin(xyz=(second_len, 0.0, -0.077)),
        material=cast_grey,
        name="wrist_lower_knuckle",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Cylinder(radius=0.036, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_grey,
        name="wrist_barrel",
    )
    head_frame.visual(
        Box((0.165, 0.075, 0.048)),
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
        material=cast_grey,
        name="head_neck_casting",
    )
    head_frame.visual(
        Box((0.105, 0.048, 0.042)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=cast_grey,
        name="head_center_spine",
    )
    head_frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.220, 0.220),
                (0.260, 0.260),
                0.026,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.010,
                outer_corner_radius=0.020,
            ),
            "square_head_frame_mesh",
        ),
        origin=Origin(xyz=(head_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="square_head_frame",
    )
    # Cross bars keep the visible square frame mechanically tied to the neck.
    head_frame.visual(
        Box((0.030, 0.140, 0.024)),
        origin=Origin(xyz=(head_x, 0.0, 0.0)),
        material=powder_black,
        name="head_horizontal_crossbar",
    )
    head_frame.visual(
        Box((0.030, 0.024, 0.220)),
        origin=Origin(xyz=(head_x, 0.0, 0.0)),
        material=powder_black,
        name="head_vertical_crossbar",
    )
    head_frame.visual(
        Box((0.045, 0.030, 0.075)),
        origin=Origin(xyz=(head_x, -0.118, 0.0)),
        material=cast_grey,
        name="tilt_pad_side_0",
    )
    head_frame.visual(
        Box((0.045, 0.030, 0.075)),
        origin=Origin(xyz=(head_x, 0.118, 0.0)),
        material=cast_grey,
        name="tilt_pad_side_1",
    )
    for suffix, z in (("upper_stop", 0.112), ("lower_stop", -0.112)):
        head_frame.visual(
            Box((0.020, 0.082, 0.014)),
            origin=Origin(xyz=(head_x + 0.024, 0.0, z)),
            material=dark_cover,
            name=f"tilt_{suffix}",
        )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        Box((0.020, 0.156, 0.020)),
        origin=Origin(xyz=(0.034, 0.0, 0.088)),
        material=powder_black,
        name="cradle_top_rail",
    )
    tilt_cradle.visual(
        Box((0.020, 0.156, 0.020)),
        origin=Origin(xyz=(0.034, 0.0, -0.088)),
        material=powder_black,
        name="cradle_bottom_rail",
    )
    tilt_cradle.visual(
        Box((0.020, 0.022, 0.210)),
        origin=Origin(xyz=(0.034, -0.064, 0.0)),
        material=powder_black,
        name="vesa_rail_side_0",
    )
    tilt_cradle.visual(
        Box((0.026, 0.032, 0.034)),
        origin=Origin(xyz=(0.038, -0.07552, 0.0)),
        material=cast_grey,
        name="hinge_bridge_side_0",
    )
    tilt_cradle.visual(
        Box((0.030, 0.030, 0.060)),
        origin=Origin(xyz=(0.014, -0.088, 0.0)),
        material=cast_grey,
        name="hinge_lug_side_0",
    )
    tilt_cradle.visual(
        Box((0.020, 0.022, 0.210)),
        origin=Origin(xyz=(0.034, 0.064, 0.0)),
        material=powder_black,
        name="vesa_rail_side_1",
    )
    tilt_cradle.visual(
        Box((0.026, 0.032, 0.034)),
        origin=Origin(xyz=(0.038, 0.07552, 0.0)),
        material=cast_grey,
        name="hinge_bridge_side_1",
    )
    tilt_cradle.visual(
        Box((0.030, 0.030, 0.060)),
        origin=Origin(xyz=(0.014, 0.088, 0.0)),
        material=cast_grey,
        name="hinge_lug_side_1",
    )
    tilt_cradle.visual(
        Box((0.028, 0.112, 0.042)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=cast_grey,
        name="tilt_center_boss",
    )
    for i, (y, z) in enumerate(((-0.050, -0.050), (0.050, -0.050), (-0.050, 0.050), (0.050, 0.050))):
        _add_cylinder(
            tilt_cradle,
            name=f"vesa_slot_bolt_{i}",
            radius=0.006,
            length=0.012,
            xyz=(0.046, y, z),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=fastener,
        )

    model.articulation(
        "wall_to_first",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_arm,
        origin=Origin(xyz=(wall_pivot_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.65, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(first_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=135.0, velocity=0.75, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "second_to_head",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=head_frame,
        origin=Origin(xyz=(second_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.8, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "head_to_cradle",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=tilt_cradle,
        origin=Origin(xyz=(head_x, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.55, lower=-0.35, upper=0.28),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_plate")
    first = object_model.get_part("first_arm")
    second = object_model.get_part("second_arm")
    head = object_model.get_part("head_frame")
    cradle = object_model.get_part("tilt_cradle")
    wall_joint = object_model.get_articulation("wall_to_first")
    elbow_joint = object_model.get_articulation("first_to_second")
    swivel_joint = object_model.get_articulation("second_to_head")
    tilt_joint = object_model.get_articulation("head_to_cradle")

    ctx.check(
        "four requested revolute axes",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (wall_joint, elbow_joint, swivel_joint, tilt_joint))
        and wall_joint.axis == (0.0, 0.0, 1.0)
        and elbow_joint.axis == (0.0, 0.0, 1.0)
        and swivel_joint.axis == (0.0, 0.0, 1.0)
        and tilt_joint.axis == (0.0, 1.0, 0.0),
        details="The two arm joints and head swivel must be vertical; the tilt cradle must be horizontal.",
    )

    # Interleaved knuckles are separated along Z rather than hidden by broad
    # overlap allowances.
    ctx.expect_gap(
        wall,
        first,
        axis="z",
        positive_elem="wall_upper_knuckle",
        negative_elem="wall_barrel",
        min_gap=0.0,
        max_gap=0.002,
        name="wall upper knuckle bears on rotating barrel",
    )
    ctx.expect_gap(
        first,
        second,
        axis="z",
        positive_elem="elbow_upper_knuckle",
        negative_elem="elbow_barrel",
        min_gap=0.0,
        max_gap=0.002,
        name="elbow upper knuckle bears on rotating barrel",
    )
    ctx.expect_gap(
        second,
        head,
        axis="z",
        positive_elem="wrist_upper_knuckle",
        negative_elem="wrist_barrel",
        min_gap=0.0,
        max_gap=0.002,
        name="wrist upper knuckle bears on rotating barrel",
    )
    ctx.expect_contact(
        head,
        cradle,
        elem_a="tilt_pad_side_1",
        elem_b="hinge_lug_side_1",
        contact_tol=0.001,
        name="tilt lug bears against yoke pad",
    )

    rest_elbow = ctx.part_world_position(second)
    rest_head = ctx.part_world_position(head)
    rest_cradle_aabb = ctx.part_world_aabb(cradle)
    with ctx.pose({wall_joint: 0.80}):
        swung_elbow = ctx.part_world_position(second)
    ctx.check(
        "long first arm swings clear of wall plate",
        rest_elbow is not None
        and swung_elbow is not None
        and swung_elbow[1] > rest_elbow[1] + 0.45
        and swung_elbow[0] > 0.48,
        details=f"rest={rest_elbow}, swung={swung_elbow}",
    )
    with ctx.pose({elbow_joint: 1.05}):
        swung_head = ctx.part_world_position(head)
    ctx.check(
        "short second arm folds as a separate link",
        rest_head is not None
        and swung_head is not None
        and swung_head[1] > rest_head[1] + 0.32
        and swung_head[0] > rest_elbow[0] + 0.15,
        details=f"rest_head={rest_head}, swung_head={swung_head}",
    )
    with ctx.pose({swivel_joint: 0.65}):
        swiveled_cradle = ctx.part_world_position(cradle)
    rest_cradle = ctx.part_world_position(cradle)
    ctx.check(
        "head assembly swivels after the arms",
        rest_cradle is not None
        and swiveled_cradle is not None
        and swiveled_cradle[1] > rest_cradle[1] + 0.10,
        details=f"rest={rest_cradle}, swiveled={swiveled_cradle}",
    )
    with ctx.pose({tilt_joint: 0.28}):
        tilted_cradle_aabb = ctx.part_world_aabb(cradle)
    ctx.check(
        "tilt cradle pitches about horizontal trunnions",
        rest_cradle_aabb is not None
        and tilted_cradle_aabb is not None
        and tilted_cradle_aabb[1][0] > rest_cradle_aabb[1][0] + 0.015,
        details=f"rest_aabb={rest_cradle_aabb}, tilted_aabb={tilted_cradle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
