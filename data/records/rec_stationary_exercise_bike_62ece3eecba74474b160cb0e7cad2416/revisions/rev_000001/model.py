from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    superellipse_side_loft,
    tube_from_spline_points,
    wire_from_points,
)


FLYWHEEL_CENTER = (0.36, 0.0, 0.42)
BOTTOM_BRACKET = (-0.08, 0.0, 0.42)
SEAT_AXIS_X = -0.42
SEAT_SLEEVE_TOP_Z = 0.82
SEAT_TRAVEL = 0.18


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _straight_tube(start, end, *, radius: float, name: str):
    return _mesh(
        wire_from_points(
            [start, end],
            radius=radius,
            radial_segments=20,
            cap_ends=True,
        ),
        name,
    )


def _flywheel_annulus():
    rim_profile = [
        (0.165, -0.040),
        (0.260, -0.040),
        (0.260, 0.040),
        (0.165, 0.040),
        (0.165, -0.040),
    ]
    return LatheGeometry(rim_profile, segments=96).rotate_x(pi / 2.0)


def _saddle_shape():
    sections = [
        (-0.20, -0.025, 0.030, 0.16),
        (-0.08, -0.030, 0.045, 0.23),
        (0.06, -0.028, 0.043, 0.20),
        (0.18, -0.022, 0.032, 0.12),
    ]
    # The loft helper's long axis is local +Y; rotate it so the saddle length
    # runs front-to-back in the bike frame.
    return superellipse_side_loft(sections, exponents=2.4, segments=48).rotate_z(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    model.material("powder_black", rgba=(0.035, 0.038, 0.040, 1.0))
    model.material("charcoal_plastic", rgba=(0.095, 0.100, 0.108, 1.0))
    model.material("dark_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("seat_vinyl", rgba=(0.060, 0.055, 0.052, 1.0))
    model.material("red_accent", rgba=(0.72, 0.06, 0.04, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.40, 0.66, 1.20)),
        mass=34.0,
        origin=Origin(xyz=(0.10, 0.0, 0.56)),
    )

    # Fewer, larger welded frame members: floor rail, two stabilizers, one
    # continuous main spine, one seat brace, and one handlebar mast.
    frame.visual(
        _straight_tube((-0.62, 0.0, 0.070), (0.76, 0.0, 0.070), radius=0.036, name="floor_rail_mesh"),
        material="powder_black",
        name="floor_rail",
    )
    frame.visual(
        _straight_tube((-0.62, -0.33, 0.070), (-0.62, 0.33, 0.070), radius=0.037, name="rear_stabilizer_mesh"),
        material="powder_black",
        name="rear_stabilizer",
    )
    frame.visual(
        _straight_tube((0.76, -0.33, 0.070), (0.76, 0.33, 0.070), radius=0.037, name="front_stabilizer_mesh"),
        material="powder_black",
        name="front_stabilizer",
    )
    frame.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (-0.62, 0.0, 0.090),
                    (-0.30, 0.0, 0.210),
                    (BOTTOM_BRACKET[0], 0.0, BOTTOM_BRACKET[2] - 0.094),
                    (0.10, 0.0, 0.180),
                    (FLYWHEEL_CENTER[0], 0.0, FLYWHEEL_CENTER[2] - 0.315),
                    (0.76, 0.0, 0.090),
                ],
                radius=0.043,
                samples_per_segment=18,
                radial_segments=24,
                cap_ends=True,
            ),
            "main_spine_mesh",
        ),
        material="powder_black",
        name="main_spine",
    )
    frame.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (-0.58, 0.0, 0.100),
                    (-0.48, 0.0, 0.360),
                    (SEAT_AXIS_X, 0.0, 0.470),
                ],
                radius=0.038,
                samples_per_segment=18,
                radial_segments=24,
                cap_ends=True,
            ),
            "seat_column_mesh",
        ),
        material="powder_black",
        name="seat_column",
    )
    frame.visual(
        Cylinder(radius=0.044, length=0.47),
        origin=Origin(xyz=(SEAT_AXIS_X, 0.0, 0.585)),
        material="powder_black",
        name="seat_sleeve",
    )
    frame.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (0.76, 0.0, 0.090),
                    (0.72, 0.0, 0.500),
                    (0.63, 0.0, 0.840),
                    (0.58, 0.0, 1.120),
                ],
                radius=0.037,
                samples_per_segment=18,
                radial_segments=24,
                cap_ends=True,
            ),
            "handlebar_mast_mesh",
        ),
        material="powder_black",
        name="handlebar_mast",
    )
    frame.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (0.55, -0.33, 1.120),
                    (0.59, -0.18, 1.180),
                    (0.59, 0.0, 1.170),
                    (0.59, 0.18, 1.180),
                    (0.55, 0.33, 1.120),
                ],
                radius=0.022,
                samples_per_segment=16,
                radial_segments=20,
                cap_ends=True,
            ),
            "handlebar_mesh",
        ),
        material="dark_steel",
        name="handlebar",
    )
    frame.visual(
        Box((0.105, 0.105, 0.095)),
        origin=Origin(xyz=(0.580, 0.0, 1.125)),
        material="dark_steel",
        name="handlebar_clamp",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.14),
        origin=Origin(xyz=(0.55, -0.35, 1.120), rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="grip_0",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.14),
        origin=Origin(xyz=(0.55, 0.35, 1.120), rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="grip_1",
    )

    # Static flywheel housing: two side covers tied together by an axle boss.
    for side, name in [(-1.0, "housing_cover_0"), (1.0, "housing_cover_1")]:
        frame.visual(
            Cylinder(radius=0.325, length=0.030),
            origin=Origin(xyz=(FLYWHEEL_CENTER[0], side * 0.115, FLYWHEEL_CENTER[2]), rpy=(pi / 2.0, 0.0, 0.0)),
            material="charcoal_plastic",
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.065, length=0.300),
        origin=Origin(xyz=FLYWHEEL_CENTER, rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="flywheel_axle_boss",
    )
    frame.visual(
        Box((0.18, 0.34, 0.040)),
        origin=Origin(xyz=(FLYWHEEL_CENTER[0], 0.0, FLYWHEEL_CENTER[2] - 0.295)),
        material="powder_black",
        name="housing_cradle",
    )
    frame.visual(
        Cylinder(radius=0.058, length=0.320),
        origin=Origin(xyz=BOTTOM_BRACKET, rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="bottom_bracket_boss",
    )
    frame.visual(
        Box((0.18, 0.035, 0.070)),
        origin=Origin(xyz=(0.16, -0.135, 0.62)),
        material="red_accent",
        name="housing_badge",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.26, length=0.08),
        mass=10.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    flywheel.visual(_mesh(_flywheel_annulus(), "flywheel_annulus"), material="brushed_steel", name="flywheel_ring")
    flywheel.visual(
        Cylinder(radius=0.074, length=0.085),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="bearing_bushing",
    )
    flywheel.visual(Box((0.096, 0.035, 0.030)), origin=Origin(xyz=(0.120, 0.0, 0.0)), material="brushed_steel", name="spoke_0")
    flywheel.visual(Box((0.096, 0.035, 0.030)), origin=Origin(xyz=(-0.120, 0.0, 0.0)), material="brushed_steel", name="spoke_1")
    flywheel.visual(Box((0.030, 0.035, 0.096)), origin=Origin(xyz=(0.0, 0.0, 0.120)), material="brushed_steel", name="spoke_2")
    flywheel.visual(Box((0.030, 0.035, 0.096)), origin=Origin(xyz=(0.0, 0.0, -0.120)), material="brushed_steel", name="spoke_3")
    flywheel.visual(
        Box((0.035, 0.086, 0.140)),
        origin=Origin(xyz=(0.0, -0.002, 0.175)),
        material="red_accent",
        name="balance_mark",
    )

    crank = model.part("crank")
    crank.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.58),
        mass=2.4,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    crank.visual(
        Cylinder(radius=0.024, length=0.590),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="spindle",
    )
    crank.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, -0.185, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="drive_disk",
    )
    crank.visual(Box((0.034, 0.060, 0.340)), origin=Origin(xyz=(0.0, 0.255, -0.170)), material="brushed_steel", name="crank_arm_0")
    crank.visual(Box((0.034, 0.060, 0.340)), origin=Origin(xyz=(0.0, -0.255, 0.170)), material="brushed_steel", name="crank_arm_1")
    crank.visual(Box((0.030, 0.170, 0.026)), origin=Origin(xyz=(0.0, 0.355, -0.340)), material="dark_steel", name="pedal_axle_0")
    crank.visual(Box((0.030, 0.170, 0.026)), origin=Origin(xyz=(0.0, -0.355, 0.340)), material="dark_steel", name="pedal_axle_1")
    crank.visual(Box((0.100, 0.110, 0.034)), origin=Origin(xyz=(0.0, 0.455, -0.340)), material="rubber_black", name="pedal_0")
    crank.visual(Box((0.100, 0.110, 0.034)), origin=Origin(xyz=(0.0, -0.455, 0.340)), material="rubber_black", name="pedal_1")

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, 0.72)),
        mass=4.0,
        origin=Origin(xyz=(-0.06, 0.0, 0.06)),
    )
    seat_post.visual(
        Cylinder(radius=0.029, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material="brushed_steel",
        name="inner_post",
    )
    seat_post.visual(
        Box((0.095, 0.070, 0.045)),
        origin=Origin(xyz=(-0.055, 0.0, 0.350)),
        material="dark_steel",
        name="saddle_clamp",
    )
    seat_post.visual(
        Box((0.240, 0.016, 0.018)),
        origin=Origin(xyz=(-0.080, -0.040, 0.376)),
        material="brushed_steel",
        name="saddle_rail_0",
    )
    seat_post.visual(
        Box((0.240, 0.016, 0.018)),
        origin=Origin(xyz=(-0.080, 0.040, 0.376)),
        material="brushed_steel",
        name="saddle_rail_1",
    )
    seat_post.visual(
        _mesh(_saddle_shape(), "saddle_mesh"),
        origin=Origin(xyz=(-0.080, 0.0, 0.405)),
        material="seat_vinyl",
        name="saddle",
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=FLYWHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=35.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=BOTTOM_BRACKET),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=20.0),
    )
    model.articulation(
        "seat_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(SEAT_AXIS_X, 0.0, SEAT_SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=SEAT_TRAVEL, effort=120.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank")
    seat_post = object_model.get_part("seat_post")
    flywheel_spin = object_model.get_articulation("flywheel_spin")
    crank_spin = object_model.get_articulation("crank_spin")
    seat_slide = object_model.get_articulation("seat_slide")

    ctx.check(
        "primary joints are present",
        flywheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and crank_spin.articulation_type == ArticulationType.CONTINUOUS
        and seat_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"flywheel={flywheel_spin.articulation_type}, crank={crank_spin.articulation_type}, seat={seat_slide.articulation_type}",
    )

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bottom_bracket_boss",
        elem_b="spindle",
        reason="The crank spindle is intentionally captured inside the bottom-bracket bearing boss.",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="spindle",
        outer_elem="bottom_bracket_boss",
        margin=0.002,
        name="crank spindle stays centered in the bottom bracket",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="spindle",
        elem_b="bottom_bracket_boss",
        min_overlap=0.20,
        name="crank spindle remains captured across the bearing width",
    )

    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_post",
        reason="The inner seat post is represented as sliding inside the outer sleeve proxy.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="bearing_bushing",
        reason="The rotating bearing bushing is intentionally captured around the fixed flywheel axle boss.",
    )
    ctx.expect_within(
        frame,
        flywheel,
        axes="xz",
        inner_elem="flywheel_axle_boss",
        outer_elem="bearing_bushing",
        margin=0.002,
        name="flywheel bearing surrounds the fixed axle boss",
    )
    ctx.expect_overlap(
        flywheel,
        frame,
        axes="y",
        elem_a="bearing_bushing",
        elem_b="flywheel_axle_boss",
        min_overlap=0.075,
        name="flywheel bearing stays engaged across the axle width",
    )
    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.002,
        name="seat post stays centered in the column",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.26,
        name="lower seat post remains inserted at the low setting",
    )

    rest_seat_position = ctx.part_world_position(seat_post)
    with ctx.pose({seat_slide: SEAT_TRAVEL}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.002,
            name="raised seat post stays centered in the column",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.10,
            name="raised seat post retains safe insertion",
        )
        raised_seat_position = ctx.part_world_position(seat_post)

    ctx.check(
        "seat slide raises the saddle assembly",
        rest_seat_position is not None
        and raised_seat_position is not None
        and raised_seat_position[2] > rest_seat_position[2] + 0.15,
        details=f"rest={rest_seat_position}, raised={raised_seat_position}",
    )

    ctx.expect_overlap(
        flywheel,
        frame,
        axes="xz",
        elem_a="flywheel_ring",
        elem_b="housing_cover_0",
        min_overlap=0.18,
        name="flywheel sits concentrically inside its housing silhouette",
    )

    crank_origin_rest = ctx.part_world_position(crank)
    with ctx.pose({crank_spin: pi / 2.0, flywheel_spin: pi / 3.0}):
        crank_origin_rotated = ctx.part_world_position(crank)
        flywheel_origin_rotated = ctx.part_world_position(flywheel)
    ctx.check(
        "rotating mechanisms keep their axle origins fixed",
        crank_origin_rest is not None
        and crank_origin_rotated is not None
        and flywheel_origin_rotated is not None
        and abs(crank_origin_rotated[0] - crank_origin_rest[0]) < 1e-6
        and abs(crank_origin_rotated[1] - crank_origin_rest[1]) < 1e-6
        and abs(crank_origin_rotated[2] - crank_origin_rest[2]) < 1e-6,
        details=f"crank_rest={crank_origin_rest}, crank_rotated={crank_origin_rotated}, flywheel_rotated={flywheel_origin_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
