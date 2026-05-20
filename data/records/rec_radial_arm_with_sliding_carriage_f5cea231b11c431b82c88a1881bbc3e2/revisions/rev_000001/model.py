from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_SIZE = (0.42, 0.42, 0.03)
PLINTH_SIZE = (0.22, 0.22, 0.16)
COLUMN_RADIUS = 0.075
COLUMN_TOP_Z = 1.24
SUPPORT_COLLAR_RADIUS = 0.11
SUPPORT_COLLAR_HEIGHT = 0.08
JOINT_Z = 1.38
ARM_HUB_OUTER_RADIUS = 0.070
ARM_HUB_INNER_RADIUS = 0.036
POST_SPINDLE_RADIUS = 0.032
POST_SPINDLE_HEIGHT = 0.20

ARM_RAIL_LENGTH = 0.60
ARM_RAIL_WIDTH = 0.020
ARM_RAIL_HEIGHT = 0.018
ARM_RAIL_Y = 0.043
ARM_RAIL_TOP_Z = -0.054
ARM_RAIL_CENTER_Z = ARM_RAIL_TOP_Z - (ARM_RAIL_HEIGHT / 2.0)

COVER_STRIP_LENGTH = 0.58
COVER_STRIP_WIDTH = 0.026
COVER_STRIP_THICKNESS = 0.004
COVER_STRIP_CENTER_Z = ARM_RAIL_TOP_Z - (COVER_STRIP_THICKNESS / 2.0)

SLIDE_HOME_X = 0.27
SLIDE_LOWER = 0.0
SLIDE_UPPER = 0.46
CARRIAGE_SHOE_LENGTH = 0.098
CARRIAGE_SHOE_WIDTH = 0.020
CARRIAGE_SHOE_HEIGHT = 0.010
CARRIAGE_ORIGIN_Z = ARM_RAIL_CENTER_Z - (ARM_RAIL_HEIGHT / 2.0)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _ring(
    outer_radius: float,
    inner_radius: float,
    height: float,
    center_z: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, center_z - (height / 2.0)))
    )


def _cylinder(radius: float, height: float, center_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, center_z - (height / 2.0)))
    )


def _fuse(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _post_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(*BASE_SIZE)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, BASE_SIZE[2] / 2.0))
    )
    plinth = (
        cq.Workplane("XY")
        .box(*PLINTH_SIZE)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, BASE_SIZE[2] + (PLINTH_SIZE[2] / 2.0)))
    )
    column = _cylinder(
        COLUMN_RADIUS,
        COLUMN_TOP_Z - (BASE_SIZE[2] + PLINTH_SIZE[2]),
        (BASE_SIZE[2] + PLINTH_SIZE[2] + COLUMN_TOP_Z) / 2.0,
    )
    lower_clamp_ring = _ring(
        0.098,
        COLUMN_RADIUS,
        0.032,
        COLUMN_TOP_Z - 0.07,
    )
    lower_bearing_housing = (
        cq.Workplane("XY")
        .circle(0.064)
        .extrude(0.074)
        .translate((0.0, 0.0, JOINT_Z - 0.144))
    )
    thrust_washer = _ring(
        0.072,
        POST_SPINDLE_RADIUS + 0.002,
        0.014,
        JOINT_Z - 0.042,
    )
    retainer_ring = _ring(
        0.045,
        0.0,
        0.022,
        JOINT_Z + 0.073,
    )
    spindle = _cylinder(
        POST_SPINDLE_RADIUS,
        0.160,
        JOINT_Z + 0.030,
    )
    return _fuse(
        base_plate,
        plinth,
        column,
        lower_clamp_ring,
        lower_bearing_housing,
        thrust_washer,
        spindle,
        retainer_ring,
    )


def _arm_beam_shape() -> cq.Workplane:
    bearing_cartridge = _ring(0.050, 0.041, 0.014, -0.028)
    beam_shell = (
        cq.Workplane("XY")
        .polyline(
            [
                (0.16, -0.055),
                (0.22, -0.095),
                (0.78, -0.072),
                (0.97, -0.058),
                (0.97, 0.058),
                (0.78, 0.072),
                (0.22, 0.095),
                (0.16, 0.055),
            ]
        )
        .close()
        .extrude(0.078)
        .translate((0.0, 0.0, -0.028))
    )
    left_side_web = _box((0.11, 0.018, 0.030), (0.120, 0.058, 0.010))
    right_side_web = _box((0.11, 0.018, 0.030), (0.120, -0.058, 0.010))
    root_block = _box((0.13, 0.150, 0.066), (0.205, 0.0, 0.006))
    rail_pad = _box((0.68, 0.12, 0.024), (0.52, 0.0, -0.042))
    end_cap = _box((0.050, 0.132, 0.078), (0.945, 0.0, -0.002))
    root_gusset = _box((0.17, 0.10, 0.032), (0.27, 0.0, -0.018))
    return _fuse(
        bearing_cartridge,
        beam_shell,
        left_side_web,
        right_side_web,
        root_block,
        rail_pad,
        end_cap,
        root_gusset,
    )


def _carriage_shape() -> cq.Workplane:
    body = (
        _box((0.115, 0.078, 0.165), (0.0, 0.0, -0.122))
        .edges("|Z")
        .fillet(0.008)
    )
    upper_saddle = _box((0.135, 0.050, 0.020), (0.0, 0.0, -0.020))
    left_cheek = _box((0.11, 0.020, 0.070), (0.0, ARM_RAIL_Y, -0.045))
    right_cheek = _box((0.11, 0.020, 0.070), (0.0, -ARM_RAIL_Y, -0.045))
    lower_plate = _box((0.090, 0.060, 0.020), (0.0, 0.0, -0.215))
    nose_boss = (
        cq.Workplane("YZ")
        .circle(0.018)
        .extrude(0.048)
        .translate((0.057, 0.0, -0.185))
    )
    return _fuse(body, upper_saddle, left_cheek, right_cheek, lower_plate, nose_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_boom")

    model.material("painted_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("cast_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("hardened_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("machine_gray", rgba=(0.63, 0.65, 0.69, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("brushed_stainless", rgba=(0.76, 0.78, 0.80, 1.0))

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_post_shape(), "post_body"),
        material="painted_steel",
        name="post_body",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, JOINT_Z + 0.12)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, (JOINT_Z + 0.12) / 2.0)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_beam_shape(), "arm_body"),
        material="cast_aluminum",
        name="arm_body",
    )
    arm.visual(
        Box((ARM_RAIL_LENGTH, ARM_RAIL_WIDTH, ARM_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.52, ARM_RAIL_Y, ARM_RAIL_CENTER_Z)),
        material="hardened_steel",
        name="left_rail",
    )
    arm.visual(
        Box((ARM_RAIL_LENGTH, ARM_RAIL_WIDTH, ARM_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.52, -ARM_RAIL_Y, ARM_RAIL_CENTER_Z)),
        material="hardened_steel",
        name="right_rail",
    )
    arm.visual(
        Box((COVER_STRIP_LENGTH, COVER_STRIP_WIDTH, COVER_STRIP_THICKNESS)),
        origin=Origin(xyz=(0.52, 0.0, COVER_STRIP_CENTER_Z)),
        material="brushed_stainless",
        name="cover_strip",
    )
    arm.visual(
        Box((0.016, 0.092, 0.016)),
        origin=Origin(xyz=(0.195, 0.0, -0.062)),
        material="hardened_steel",
        name="inboard_stop",
    )
    arm.visual(
        Box((0.016, 0.092, 0.016)),
        origin=Origin(xyz=(0.823, 0.0, -0.062)),
        material="hardened_steel",
        name="outboard_stop",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.98, 0.20, 0.15)),
        mass=28.0,
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_body"),
        material="machine_gray",
        name="carriage_body",
    )
    carriage.visual(
        Box((CARRIAGE_SHOE_LENGTH, CARRIAGE_SHOE_WIDTH, CARRIAGE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                ARM_RAIL_Y,
                -(CARRIAGE_SHOE_HEIGHT / 2.0),
            )
        ),
        material="hardened_steel",
        name="left_shoe",
    )
    carriage.visual(
        Box((CARRIAGE_SHOE_LENGTH, CARRIAGE_SHOE_WIDTH, CARRIAGE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -ARM_RAIL_Y,
                -(CARRIAGE_SHOE_HEIGHT / 2.0),
            )
        ),
        material="hardened_steel",
        name="right_shoe",
    )
    for sign, y in (("left", ARM_RAIL_Y), ("right", -ARM_RAIL_Y)):
        for end_name, x in (("front", 0.046), ("rear", -0.046)):
            carriage.visual(
                Box((0.006, CARRIAGE_SHOE_WIDTH, 0.010)),
                origin=Origin(xyz=(x, y, -0.015)),
                material="rubber_black",
                name=f"{sign}_{end_name}_wiper",
            )
    carriage.inertial = Inertial.from_geometry(
        Box((0.135, 0.11, 0.26)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
    )

    model.articulation(
        "post_to_arm",
        ArticulationType.REVOLUTE,
        parent=post,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.55,
            upper=1.55,
            effort=260.0,
            velocity=0.9,
        ),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, CARRIAGE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=1400.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    swing = object_model.get_articulation("post_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    left_rail = arm.get_visual("left_rail")
    right_rail = arm.get_visual("right_rail")
    cover_strip = arm.get_visual("cover_strip")
    inboard_stop = arm.get_visual("inboard_stop")
    outboard_stop = arm.get_visual("outboard_stop")

    carriage_body = carriage.get_visual("carriage_body")
    left_shoe = carriage.get_visual("left_shoe")
    right_shoe = carriage.get_visual("right_shoe")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        post,
        arm,
        reason=(
            "Arm root uses a nested slewing-bearing style support around the post spindle; "
            "the simplified visual envelopes intentionally share bearing-support volume at the hub."
        ),
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "primary articulation axes",
        tuple(round(v, 6) for v in swing.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"swing axis={swing.axis}, slide axis={slide.axis}",
    )

    with ctx.pose({swing: 0.0, slide: 0.0}):
        ctx.expect_contact(post, arm, name="arm hub is supported on post thrust collar")
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=left_shoe,
            elem_b=left_rail,
            name="left shoe bears on left rail",
        )
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=right_shoe,
            elem_b=right_rail,
            name="right shoe bears on right rail",
        )
        ctx.expect_gap(
            arm,
            carriage,
            axis="z",
            positive_elem=cover_strip,
            negative_elem=carriage_body,
            min_gap=0.018,
            max_gap=0.035,
            name="cover strip clears carriage body",
        )
        ctx.expect_gap(
            carriage,
            arm,
            axis="x",
            positive_elem=left_shoe,
            negative_elem=inboard_stop,
            min_gap=0.018,
            max_gap=0.030,
            name="inner travel stop leaves service clearance",
        )

    with ctx.pose({swing: 0.0, slide: slide.motion_limits.upper}):
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=left_shoe,
            elem_b=left_rail,
            name="left shoe stays supported at outer travel",
        )
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=right_shoe,
            elem_b=right_rail,
            name="right shoe stays supported at outer travel",
        )
        ctx.expect_gap(
            arm,
            carriage,
            axis="x",
            positive_elem=outboard_stop,
            negative_elem=right_shoe,
            min_gap=0.035,
            max_gap=0.050,
            name="outer stop clears carriage at full reach",
        )

    with ctx.pose({swing: swing.motion_limits.lower, slide: slide.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping at inboard swing extreme")
    with ctx.pose({swing: swing.motion_limits.upper, slide: slide.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping at outboard swing extreme")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
