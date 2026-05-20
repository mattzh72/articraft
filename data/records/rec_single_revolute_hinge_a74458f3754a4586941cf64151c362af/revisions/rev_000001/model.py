from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LEAF_THICKNESS = 0.010
LEAF_LENGTH = 0.205
LEAF_HEIGHT = 0.150
ROOT_COLLAR_LENGTH = 0.024
ROOT_COLLAR_HEIGHT = 0.142
ROOT_ATTACH_X = 0.006
TIP_RADIUS = 0.025
MOUNT_HOLE_RADIUS = 0.0065

BARREL_OUTER_RADIUS = 0.014
PIN_RADIUS = 0.006
FREE_BORE_RADIUS = 0.0075
WASHER_RADIUS = 0.0105
BARREL_CENTER_Y = BARREL_OUTER_RADIUS + LEAF_THICKNESS / 2.0

PARENT_KNUCKLE_LENGTH = 0.032
CHILD_KNUCKLE_LENGTH = 0.026
WASHER_THICKNESS = 0.0025
END_CLEAR = 0.005
BARREL_LENGTH = (
    3.0 * PARENT_KNUCKLE_LENGTH
    + 2.0 * CHILD_KNUCKLE_LENGTH
    + 4.0 * WASHER_THICKNESS
    + 2.0 * END_CLEAR
)

PIN_EXPOSED = 0.004
PIN_HEAD_RADIUS = 0.0095
PIN_HEAD_LENGTH = 0.005
PIN_TIP_RADIUS = 0.0052
PIN_TIP_LENGTH = 0.005

FLAT_START_X = 0.036
RIB_PLATE_OVERLAP = 0.006

HINGE_UPPER_LIMIT = 1.55

LEAF_HOLE_POSITIONS = (
    (0.055, 0.042),
    (0.112, -0.040),
    (0.160, 0.000),
)


def _cylinder_on_axis(
    radius: float,
    length: float,
    center_z: float,
    *,
    center_y: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, center_y, center_z - length / 2.0))
    )


def _tube_on_axis(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center_z: float,
    *,
    center_y: float = 0.0,
) -> cq.Workplane:
    outer = _cylinder_on_axis(outer_radius, length, center_z, center_y=center_y)
    inner = _cylinder_on_axis(inner_radius, length + 0.002, center_z, center_y=center_y)
    return outer.cut(inner)


def _knuckle_layout() -> tuple[list[tuple[float, float]], list[tuple[float, float]], list[tuple[float, float]]]:
    parent_segments: list[tuple[float, float]] = []
    child_segments: list[tuple[float, float]] = []
    washers: list[tuple[float, float]] = []

    cursor = -BARREL_LENGTH / 2.0 + END_CLEAR
    for index in range(5):
        is_parent = index % 2 == 0
        seg_length = PARENT_KNUCKLE_LENGTH if is_parent else CHILD_KNUCKLE_LENGTH
        center_z = cursor + seg_length / 2.0
        if is_parent:
            parent_segments.append((center_z, seg_length))
        else:
            child_segments.append((center_z, seg_length))
        cursor += seg_length
        if index < 4:
            washers.append((cursor + WASHER_THICKNESS / 2.0, WASHER_THICKNESS))
            cursor += WASHER_THICKNESS

    return parent_segments, child_segments, washers


PARENT_KNUCKLES, CHILD_KNUCKLES, WASHER_STACK = _knuckle_layout()


def _mount_hole(x_pos: float, z_pos: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_pos, z_pos)
        .circle(MOUNT_HOLE_RADIUS)
        .extrude(LEAF_THICKNESS + 0.004)
        .translate((0.0, -LEAF_THICKNESS / 2.0 - 0.002, 0.0))
    )


def _knuckle_rib(direction: float, center_z: float, seg_length: float) -> cq.Workplane:
    outer_x = direction * BARREL_OUTER_RADIUS
    plate_x = direction * (FLAT_START_X + RIB_PLATE_OVERLAP)

    profile = (
        cq.Workplane("XY")
        .moveTo(outer_x, BARREL_CENTER_Y - BARREL_OUTER_RADIUS * 0.62)
        .lineTo(direction * (FLAT_START_X * 0.55), LEAF_THICKNESS / 2.0 + 0.001)
        .lineTo(plate_x, LEAF_THICKNESS / 2.0)
        .lineTo(plate_x, -LEAF_THICKNESS / 2.0)
        .lineTo(direction * (FLAT_START_X * 0.42), -LEAF_THICKNESS / 2.0)
        .lineTo(outer_x * 0.82, BARREL_CENTER_Y - BARREL_OUTER_RADIUS * 0.98)
        .close()
    )

    return profile.extrude(seg_length).translate((0.0, 0.0, center_z - seg_length / 2.0))


def _leaf_plate(direction: float, knuckles: list[tuple[float, float]]) -> cq.Workplane:
    main_length = LEAF_LENGTH - TIP_RADIUS - FLAT_START_X
    main = cq.Workplane("XY").box(main_length, LEAF_THICKNESS, LEAF_HEIGHT)
    main = main.translate((direction * (FLAT_START_X + main_length / 2.0), 0.0, 0.0))

    tip = (
        cq.Workplane("XZ")
        .circle(TIP_RADIUS)
        .extrude(LEAF_THICKNESS)
        .translate((direction * (LEAF_LENGTH - TIP_RADIUS), -LEAF_THICKNESS / 2.0, 0.0))
    )

    plate = main.union(tip)

    for center_z, seg_length in knuckles:
        plate = plate.union(_knuckle_rib(direction, center_z, seg_length))

    for x_pos, z_pos in LEAF_HOLE_POSITIONS:
        plate = plate.cut(_mount_hole(direction * x_pos, z_pos))

    return plate


def _parent_leaf_body() -> cq.Workplane:
    body = _leaf_plate(1.0, PARENT_KNUCKLES)
    for center_z, seg_length in PARENT_KNUCKLES:
        body = body.union(
            _tube_on_axis(
                BARREL_OUTER_RADIUS,
                PIN_RADIUS,
                seg_length,
                center_z,
                center_y=BARREL_CENTER_Y,
            )
        )
    return body


def _child_leaf_body() -> cq.Workplane:
    body = _leaf_plate(-1.0, CHILD_KNUCKLES)
    for center_z, seg_length in CHILD_KNUCKLES:
        body = body.union(
            _tube_on_axis(
                BARREL_OUTER_RADIUS,
                FREE_BORE_RADIUS,
                seg_length,
                center_z,
                center_y=BARREL_CENTER_Y,
            )
        )
    return body


def _pin_assembly() -> cq.Workplane:
    top_parent_center = PARENT_KNUCKLES[-1][0]
    top_parent_length = PARENT_KNUCKLES[-1][1]
    bottom_parent_center = PARENT_KNUCKLES[0][0]
    bottom_parent_length = PARENT_KNUCKLES[0][1]

    assembly = cq.Workplane("XY")

    for center_z, seg_length in PARENT_KNUCKLES:
        assembly = assembly.union(
            _cylinder_on_axis(
                PIN_RADIUS,
                seg_length,
                center_z,
                center_y=BARREL_CENTER_Y,
            )
        )

    top_exposed_length = END_CLEAR + PIN_EXPOSED
    assembly = assembly.union(
        _cylinder_on_axis(
            PIN_RADIUS,
            top_exposed_length,
            top_parent_center + top_parent_length / 2.0 + top_exposed_length / 2.0,
            center_y=BARREL_CENTER_Y,
        )
    )
    bottom_exposed_length = END_CLEAR + PIN_EXPOSED
    assembly = assembly.union(
        _cylinder_on_axis(
            PIN_RADIUS,
            bottom_exposed_length,
            bottom_parent_center - bottom_parent_length / 2.0 - bottom_exposed_length / 2.0,
            center_y=BARREL_CENTER_Y,
        )
    )

    head = _cylinder_on_axis(
        PIN_HEAD_RADIUS,
        PIN_HEAD_LENGTH,
        top_parent_center + top_parent_length / 2.0 + top_exposed_length + PIN_HEAD_LENGTH / 2.0,
        center_y=BARREL_CENTER_Y,
    )

    turned_tip = _cylinder_on_axis(
        PIN_TIP_RADIUS,
        PIN_TIP_LENGTH,
        bottom_parent_center - bottom_parent_length / 2.0 - bottom_exposed_length - PIN_TIP_LENGTH / 2.0,
        center_y=BARREL_CENTER_Y,
    )

    assembly = assembly.union(head).union(turned_tip)
    for center_z, seg_length in WASHER_STACK:
        assembly = assembly.union(
            _cylinder_on_axis(WASHER_RADIUS, seg_length, center_z, center_y=BARREL_CENTER_Y)
        )

    return assembly


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_strap_hinge")

    model.material("oxide_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("machined_steel", rgba=(0.71, 0.73, 0.76, 1.0))

    parent_leaf = model.part("parent_leaf")
    parent_leaf.visual(
        mesh_from_cadquery(_parent_leaf_body(), "parent_leaf_body"),
        material="oxide_steel",
        name="leaf_body",
    )
    parent_leaf.visual(
        mesh_from_cadquery(_pin_assembly(), "pin_assembly"),
        material="machined_steel",
        name="pin_assembly",
    )
    parent_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_LENGTH, 2.0 * BARREL_OUTER_RADIUS, BARREL_LENGTH + PIN_HEAD_LENGTH + PIN_TIP_LENGTH)),
        mass=2.6,
        origin=Origin(xyz=(LEAF_LENGTH / 2.0, 0.0, 0.0)),
    )

    child_leaf = model.part("child_leaf")
    child_leaf.visual(
        mesh_from_cadquery(_child_leaf_body(), "child_leaf_body"),
        material="oxide_steel",
        name="leaf_body",
    )
    child_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_LENGTH, 2.0 * BARREL_OUTER_RADIUS, BARREL_LENGTH)),
        mass=2.2,
        origin=Origin(xyz=(-LEAF_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=parent_leaf,
        child=child_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=0.0,
            upper=HINGE_UPPER_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    parent_leaf = object_model.get_part("parent_leaf")
    child_leaf = object_model.get_part("child_leaf")
    hinge = object_model.get_articulation("leaf_hinge")
    parent_leaf.get_visual("leaf_body")
    parent_leaf.get_visual("pin_assembly")
    child_leaf.get_visual("leaf_body")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
    ctx.allow_overlap(
        child_leaf,
        parent_leaf,
        elem_a="leaf_body",
        elem_b="pin_assembly",
        reason=(
            "captured hinge-pin fit inside the child barrel bores; the visible "
            "mechanical intent is a retained pin-and-knuckle assembly rather than "
            "free external body clipping"
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "hinge_axis_follows_barrel",
        tuple(round(value, 6) for value in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected barrel axis (0, 0, 1), got {hinge.axis}",
    )
    ctx.check(
        "hinge_has_heavy_working_range",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 1.35,
        details="hinge should open from a flat spread position through a substantial working swing",
    )
    ctx.expect_origin_distance(
        parent_leaf,
        child_leaf,
        axes="xyz",
        max_dist=1e-6,
        name="leaf_origins_share_hinge_axis",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            child_leaf,
            parent_leaf,
            elem_a="leaf_body",
            elem_b="pin_assembly",
            contact_tol=0.0008,
            name="rest_pose_leaf_is_supported_by_pin_and_washers",
        )

    with ctx.pose({hinge: 0.90}):
        ctx.fail_if_parts_overlap_in_current_pose(name="mid_swing_no_part_overlap")
        ctx.expect_contact(
            child_leaf,
            parent_leaf,
            elem_a="leaf_body",
            elem_b="pin_assembly",
            contact_tol=0.0008,
            name="mid_swing_leaf_stays_on_pin_stack",
        )

    with ctx.pose({hinge: HINGE_UPPER_LIMIT - 0.05}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_limit_pose_no_part_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
