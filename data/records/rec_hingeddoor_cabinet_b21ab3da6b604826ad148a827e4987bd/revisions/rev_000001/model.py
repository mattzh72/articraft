from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.760
CABINET_DEPTH = 0.580
CABINET_HEIGHT = 0.870
SIDE_THICKNESS = 0.018
TOP_THICKNESS = 0.018
BOTTOM_THICKNESS = 0.018
BACK_THICKNESS = 0.006

DOOR_THICKNESS = 0.020
DOOR_SIDE_REVEAL = 0.010
DOOR_TOP_BOTTOM_REVEAL = 0.016
MEETING_GAP = 0.004
DOOR_WIDTH = (CABINET_WIDTH - (2.0 * DOOR_SIDE_REVEAL) - MEETING_GAP) / 2.0
DOOR_HEIGHT = CABINET_HEIGHT - (2.0 * DOOR_TOP_BOTTOM_REVEAL)
DOOR_BOTTOM_Z = DOOR_TOP_BOTTOM_REVEAL
DOOR_FRONT_CENTER_Y = (CABINET_DEPTH * 0.5) + (DOOR_THICKNESS * 0.5)

FRAME_STILE_WIDTH = 0.070
FRAME_RAIL_WIDTH = 0.070
PANEL_THICKNESS = 0.010
PANEL_CENTER_Y = -0.004

HINGE_RADIUS = 0.005
HINGE_DOOR_KNUCKLE_LENGTH = 0.024
HINGE_CARCASS_KNUCKLE_LENGTH = 0.030
HINGE_STACK_HEIGHT = (
    (2.0 * HINGE_DOOR_KNUCKLE_LENGTH) + HINGE_CARCASS_KNUCKLE_LENGTH
)
HINGE_LEAF_HEIGHT = HINGE_STACK_HEIGHT
HINGE_LEAF_WIDTH_DOOR = 0.003
HINGE_LEAF_WIDTH_CARCASS = 0.003
HINGE_LEAF_DEPTH_DOOR = 0.016
HINGE_LEAF_DEPTH_CARCASS = 0.032
HINGE_BRIDGE_WIDTH = 0.002
HINGE_BRIDGE_DEPTH = 0.006
HINGE_KNUCKLE_OFFSET_Z = (
    HINGE_CARCASS_KNUCKLE_LENGTH + HINGE_DOOR_KNUCKLE_LENGTH
) * 0.5
HINGE_OFFSET_FROM_BOTTOM = 0.185
HINGE_OFFSET_FROM_TOP = 0.185
HINGE_AXIS_X = (CABINET_WIDTH * 0.5) - DOOR_SIDE_REVEAL + HINGE_RADIUS

KNOB_BACKPLATE_RADIUS = 0.010
KNOB_BACKPLATE_DEPTH = 0.003
KNOB_STEM_RADIUS = 0.004
KNOB_STEM_DEPTH = 0.012
KNOB_HEAD_RADIUS = 0.014
KNOB_MEETING_INSET = 0.055
KNOB_HEIGHT_RATIO = 0.53


def _door_local_x(side_sign: float, x_from_outer_edge: float) -> float:
    return side_sign * (HINGE_RADIUS + x_from_outer_edge)


def _add_carcass_hinge_hardware(
    carcass,
    *,
    side_name: str,
    side_sign: float,
    center_z: float,
    hardware_material,
) -> None:
    carcass.visual(
        Box((HINGE_LEAF_WIDTH_CARCASS, HINGE_LEAF_DEPTH_CARCASS, HINGE_LEAF_HEIGHT)),
        origin=Origin(
            xyz=(
                side_sign * ((CABINET_WIDTH * 0.5) + (HINGE_LEAF_WIDTH_CARCASS * 0.5)),
                DOOR_FRONT_CENTER_Y
                - HINGE_RADIUS
                - (HINGE_LEAF_DEPTH_CARCASS * 0.5)
                + 0.0005,
                center_z,
            )
        ),
        material=hardware_material,
        name=f"{side_name}_hinge_leaf_{'upper' if center_z > CABINET_HEIGHT * 0.5 else 'lower'}",
    )
    carcass.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_CARCASS_KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(side_sign * HINGE_AXIS_X, DOOR_FRONT_CENTER_Y, center_z),
        ),
        material=hardware_material,
        name=f"{side_name}_{'upper' if center_z > CABINET_HEIGHT * 0.5 else 'lower'}_hinge_knuckle",
    )
    carcass.visual(
        Box((HINGE_BRIDGE_WIDTH, HINGE_BRIDGE_DEPTH, HINGE_CARCASS_KNUCKLE_LENGTH)),
        origin=Origin(
            xyz=(
                side_sign * (CABINET_WIDTH * 0.5),
                DOOR_FRONT_CENTER_Y - (HINGE_RADIUS * 0.6),
                center_z,
            )
        ),
        material=hardware_material,
        name=f"{side_name}_{'upper' if center_z > CABINET_HEIGHT * 0.5 else 'lower'}_hinge_bridge",
    )


def _build_door_part(model: ArticulatedObject, *, name: str, side_sign: float, paint_material, panel_material, hardware_material):
    door = model.part(name)

    door.visual(
        Box((FRAME_STILE_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(_door_local_x(side_sign, FRAME_STILE_WIDTH * 0.5), 0.0, DOOR_HEIGHT * 0.5)
        ),
        material=paint_material,
        name="outer_stile",
    )
    door.visual(
        Box((FRAME_STILE_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                _door_local_x(side_sign, DOOR_WIDTH - (FRAME_STILE_WIDTH * 0.5)),
                0.0,
                DOOR_HEIGHT * 0.5,
            )
        ),
        material=paint_material,
        name="inner_stile",
    )
    door.visual(
        Box((DOOR_WIDTH - (2.0 * FRAME_STILE_WIDTH), DOOR_THICKNESS, FRAME_RAIL_WIDTH)),
        origin=Origin(
            xyz=(_door_local_x(side_sign, DOOR_WIDTH * 0.5), 0.0, FRAME_RAIL_WIDTH * 0.5)
        ),
        material=paint_material,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - (2.0 * FRAME_STILE_WIDTH), DOOR_THICKNESS, FRAME_RAIL_WIDTH)),
        origin=Origin(
            xyz=(
                _door_local_x(side_sign, DOOR_WIDTH * 0.5),
                0.0,
                DOOR_HEIGHT - (FRAME_RAIL_WIDTH * 0.5),
            )
        ),
        material=paint_material,
        name="top_rail",
    )
    door.visual(
        Box(
            (
                DOOR_WIDTH - (2.0 * FRAME_STILE_WIDTH),
                PANEL_THICKNESS,
                DOOR_HEIGHT - (2.0 * FRAME_RAIL_WIDTH),
            )
        ),
        origin=Origin(
            xyz=(
                _door_local_x(side_sign, DOOR_WIDTH * 0.5),
                PANEL_CENTER_Y,
                DOOR_HEIGHT * 0.5,
            )
        ),
        material=panel_material,
        name="center_panel",
    )

    hinge_centers = {
        "upper_hinge": DOOR_HEIGHT - HINGE_OFFSET_FROM_TOP,
        "lower_hinge": HINGE_OFFSET_FROM_BOTTOM,
    }
    for hinge_name, hinge_center_z in hinge_centers.items():
        door.visual(
            Box((HINGE_LEAF_WIDTH_DOOR, HINGE_LEAF_DEPTH_DOOR, HINGE_LEAF_HEIGHT)),
            origin=Origin(
                xyz=(
                    side_sign * (HINGE_LEAF_WIDTH_DOOR * 0.5),
                    -0.001,
                    hinge_center_z,
                )
            ),
            material=hardware_material,
            name=f"{hinge_name}_leaf",
        )
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_DOOR_KNUCKLE_LENGTH),
            origin=Origin(
                xyz=(0.0, 0.0, hinge_center_z + HINGE_KNUCKLE_OFFSET_Z),
            ),
            material=hardware_material,
            name=f"{hinge_name}_knuckle_top",
        )
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_DOOR_KNUCKLE_LENGTH),
            origin=Origin(
                xyz=(0.0, 0.0, hinge_center_z - HINGE_KNUCKLE_OFFSET_Z),
            ),
            material=hardware_material,
            name=f"{hinge_name}_knuckle_bottom",
        )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH + HINGE_RADIUS, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=7.5,
        origin=Origin(
            xyz=(
                _door_local_x(side_sign, DOOR_WIDTH * 0.5),
                0.0,
                DOOR_HEIGHT * 0.5,
            )
        ),
    )
    return door


def _build_knob_part(model: ArticulatedObject, *, name: str, hardware_material):
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=KNOB_BACKPLATE_RADIUS, length=KNOB_BACKPLATE_DEPTH),
        origin=Origin(
            xyz=(0.0, KNOB_BACKPLATE_DEPTH * 0.5, 0.0),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=hardware_material,
        name="backplate",
    )
    knob.visual(
        Cylinder(radius=KNOB_STEM_RADIUS, length=KNOB_STEM_DEPTH),
        origin=Origin(
            xyz=(0.0, KNOB_BACKPLATE_DEPTH + (KNOB_STEM_DEPTH * 0.5), 0.0),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=hardware_material,
        name="stem",
    )
    knob.visual(
        Sphere(radius=KNOB_HEAD_RADIUS),
        origin=Origin(
            xyz=(0.0, KNOB_BACKPLATE_DEPTH + KNOB_STEM_DEPTH + (KNOB_HEAD_RADIUS * 0.78), 0.0)
        ),
        material=hardware_material,
        name="knob_head",
    )
    knob.inertial = Inertial.from_geometry(
        Box(
            (
                KNOB_HEAD_RADIUS * 2.0,
                KNOB_BACKPLATE_DEPTH + KNOB_STEM_DEPTH + (KNOB_HEAD_RADIUS * 2.0),
                KNOB_HEAD_RADIUS * 2.0,
            )
        ),
        mass=0.09,
        origin=Origin(
            xyz=(
                0.0,
                (KNOB_BACKPLATE_DEPTH + KNOB_STEM_DEPTH + (KNOB_HEAD_RADIUS * 2.0)) * 0.5,
                0.0,
            )
        ),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="base_cabinet_double_shaker_doors")

    painted_case = model.material("painted_case", rgba=(0.89, 0.88, 0.84, 1.0))
    recessed_panel = model.material("recessed_panel", rgba=(0.83, 0.82, 0.78, 1.0))
    interior_shadow = model.material("interior_shadow", rgba=(0.66, 0.62, 0.56, 1.0))
    hardware = model.material("hardware", rgba=(0.65, 0.67, 0.70, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -((CABINET_WIDTH * 0.5) - (SIDE_THICKNESS * 0.5)),
                0.0,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=painted_case,
        name="left_side",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH * 0.5) - (SIDE_THICKNESS * 0.5),
                0.0,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=painted_case,
        name="right_side",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), CABINET_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)
        ),
        material=painted_case,
        name="bottom_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), CABINET_DEPTH, TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, CABINET_HEIGHT - (TOP_THICKNESS * 0.5))
        ),
        material=painted_case,
        name="top_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), BACK_THICKNESS, CABINET_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -((CABINET_DEPTH * 0.5) - (BACK_THICKNESS * 0.5)),
                BOTTOM_THICKNESS + ((CABINET_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS) * 0.5),
            )
        ),
        material=interior_shadow,
        name="back_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), CABINET_DEPTH - BACK_THICKNESS, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                -BACK_THICKNESS * 0.5,
                CABINET_HEIGHT - TOP_THICKNESS - 0.005,
            )
        ),
        material=interior_shadow,
        name="cabinet_ceiling",
    )

    upper_hinge_z = DOOR_BOTTOM_Z + DOOR_HEIGHT - HINGE_OFFSET_FROM_TOP
    lower_hinge_z = DOOR_BOTTOM_Z + HINGE_OFFSET_FROM_BOTTOM
    _add_carcass_hinge_hardware(
        carcass,
        side_name="left",
        side_sign=-1.0,
        center_z=upper_hinge_z,
        hardware_material=hardware,
    )
    _add_carcass_hinge_hardware(
        carcass,
        side_name="left",
        side_sign=-1.0,
        center_z=lower_hinge_z,
        hardware_material=hardware,
    )
    _add_carcass_hinge_hardware(
        carcass,
        side_name="right",
        side_sign=1.0,
        center_z=upper_hinge_z,
        hardware_material=hardware,
    )
    _add_carcass_hinge_hardware(
        carcass,
        side_name="right",
        side_sign=1.0,
        center_z=lower_hinge_z,
        hardware_material=hardware,
    )
    carcass.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT * 0.5)),
    )

    left_door = _build_door_part(
        model,
        name="left_door",
        side_sign=1.0,
        paint_material=painted_case,
        panel_material=recessed_panel,
        hardware_material=hardware,
    )
    right_door = _build_door_part(
        model,
        name="right_door",
        side_sign=-1.0,
        paint_material=painted_case,
        panel_material=recessed_panel,
        hardware_material=hardware,
    )

    left_hinge = model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(-HINGE_AXIS_X, DOOR_FRONT_CENTER_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.8,
            lower=0.0,
            upper=1.85,
        ),
    )
    right_hinge = model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(HINGE_AXIS_X, DOOR_FRONT_CENTER_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.8,
            lower=-1.85,
            upper=0.0,
        ),
    )

    left_knob = _build_knob_part(model, name="left_knob", hardware_material=hardware)
    right_knob = _build_knob_part(model, name="right_knob", hardware_material=hardware)

    model.articulation(
        "left_door_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=left_door,
        child=left_knob,
        origin=Origin(
            xyz=(
                _door_local_x(1.0, DOOR_WIDTH - KNOB_MEETING_INSET),
                DOOR_THICKNESS * 0.5,
                DOOR_HEIGHT * KNOB_HEIGHT_RATIO,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=8.0,
        ),
    )
    model.articulation(
        "right_door_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=right_door,
        child=right_knob,
        origin=Origin(
            xyz=(
                _door_local_x(-1.0, DOOR_WIDTH - KNOB_MEETING_INSET),
                DOOR_THICKNESS * 0.5,
                DOOR_HEIGHT * KNOB_HEIGHT_RATIO,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")
    left_knob_joint = object_model.get_articulation("left_door_to_left_knob")
    right_knob_joint = object_model.get_articulation("right_door_to_right_knob")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    ctx.fail_if_isolated_parts(
        max_pose_samples=8,
        name="articulated parts remain support-connected through sampled poses",
    )
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

    ctx.expect_origin_gap(
        left_door,
        carcass,
        axis="y",
        min_gap=0.299,
        max_gap=0.301,
        name="left door hinge line sits at cabinet front",
    )
    ctx.expect_origin_gap(
        right_door,
        carcass,
        axis="y",
        min_gap=0.299,
        max_gap=0.301,
        name="right door hinge line sits at cabinet front",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.003,
        max_gap=0.006,
        name="door meeting gap stays narrow and even",
    )
    ctx.expect_contact(
        left_knob,
        left_door,
        name="left knob is mounted to left door",
    )
    ctx.expect_contact(
        right_knob,
        right_door,
        name="right knob is mounted to right door",
    )

    def _is_vertical(joint) -> bool:
        return (
            abs(joint.axis[0]) < 1e-9
            and abs(joint.axis[1]) < 1e-9
            and abs(abs(joint.axis[2]) - 1.0) < 1e-9
        )

    def _is_depth_axis(joint) -> bool:
        return (
            abs(joint.axis[0]) < 1e-9
            and abs(abs(joint.axis[1]) - 1.0) < 1e-9
            and abs(joint.axis[2]) < 1e-9
        )

    ctx.check(
        "door hinges use vertical axes",
        _is_vertical(left_hinge) and _is_vertical(right_hinge),
        details=f"left axis={left_hinge.axis}, right axis={right_hinge.axis}",
    )
    ctx.check(
        "knob joints spin on front-to-back axes",
        _is_depth_axis(left_knob_joint) and _is_depth_axis(right_knob_joint),
        details=f"left axis={left_knob_joint.axis}, right axis={right_knob_joint.axis}",
    )

    with ctx.pose({left_hinge: 1.15, right_hinge: -1.15}):
        ctx.expect_contact(
            left_door,
            carcass,
            elem_a="upper_hinge_knuckle_top",
            elem_b="left_upper_hinge_knuckle",
            name="left upper hinge stays clipped while open",
        )
        ctx.expect_contact(
            left_door,
            carcass,
            elem_a="lower_hinge_knuckle_top",
            elem_b="left_lower_hinge_knuckle",
            name="left lower hinge stays clipped while open",
        )
        ctx.expect_contact(
            right_door,
            carcass,
            elem_a="upper_hinge_knuckle_top",
            elem_b="right_upper_hinge_knuckle",
            name="right upper hinge stays clipped while open",
        )
        ctx.expect_contact(
            right_door,
            carcass,
            elem_a="lower_hinge_knuckle_top",
            elem_b="right_lower_hinge_knuckle",
            name="right lower hinge stays clipped while open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
