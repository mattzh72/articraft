from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


CABINET_WIDTH = 0.36
CABINET_HEIGHT = 0.50
CABINET_DEPTH = 0.14
SIDE_PANEL_THICKNESS = 0.012
BACK_PANEL_THICKNESS = 0.006
SHELF_THICKNESS = 0.008

DOOR_WIDTH = 0.35
DOOR_HEIGHT = 0.488
DOOR_THICKNESS = 0.014
MIRROR_THICKNESS = 0.004

HINGE_LEAF_THICKNESS = 0.0025
HINGE_RADIUS = 0.006
HINGE_TOTAL_HEIGHT = 0.054
HINGE_CARCASS_SEGMENT = 0.019
HINGE_DOOR_SEGMENT = 0.016
HINGE_PLATE_DEPTH = 0.024

HINGE_AXIS_X = -CABINET_WIDTH / 2.0 - HINGE_LEAF_THICKNESS - HINGE_RADIUS
HINGE_AXIS_Y = CABINET_DEPTH / 2.0 - HINGE_RADIUS - 0.003
DOOR_LEFT_OFFSET = HINGE_RADIUS + HINGE_LEAF_THICKNESS
DOOR_Y_OFFSET = 0.016

UPPER_HINGE_Z = 0.165
LOWER_HINGE_Z = -0.165

PULL_PIVOT_X = DOOR_LEFT_OFFSET + DOOR_WIDTH - 0.014
PULL_PIVOT_Y = 0.028

DOOR_OPEN_TEST_ANGLE = 1.20
PULL_TEST_ANGLE = 0.22


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medicine_cabinet")

    carcass_finish = model.material("carcass_finish", rgba=(0.95, 0.95, 0.96, 1.0))
    interior_finish = model.material("interior_finish", rgba=(0.92, 0.92, 0.91, 1.0))
    mirror_finish = model.material("mirror_finish", rgba=(0.80, 0.85, 0.90, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.71, 0.73, 0.76, 1.0))

    carcass = model.part("carcass")
    door = model.part("door")
    finger_pull = model.part("finger_pull")

    side_depth = CABINET_DEPTH - BACK_PANEL_THICKNESS
    side_center_y = (-CABINET_DEPTH / 2.0 + BACK_PANEL_THICKNESS + CABINET_DEPTH / 2.0) / 2.0
    inner_width = CABINET_WIDTH - 2.0 * SIDE_PANEL_THICKNESS
    shelf_depth = side_depth - 0.010
    shelf_center_y = (-CABINET_DEPTH / 2.0 + BACK_PANEL_THICKNESS + CABINET_DEPTH / 2.0 - 0.010) / 2.0

    carcass.visual(
        Box((CABINET_WIDTH, BACK_PANEL_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.0, -CABINET_DEPTH / 2.0 + BACK_PANEL_THICKNESS / 2.0, 0.0)),
        material=interior_finish,
        name="back_panel",
    )
    carcass.visual(
        Box((SIDE_PANEL_THICKNESS, side_depth, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + SIDE_PANEL_THICKNESS / 2.0,
                side_center_y,
                0.0,
            )
        ),
        material=carcass_finish,
        name="left_side",
    )
    carcass.visual(
        Box((SIDE_PANEL_THICKNESS, side_depth, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - SIDE_PANEL_THICKNESS / 2.0,
                side_center_y,
                0.0,
            )
        ),
        material=carcass_finish,
        name="right_side",
    )
    carcass.visual(
        Box((inner_width, side_depth, SIDE_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, side_center_y, CABINET_HEIGHT / 2.0 - SIDE_PANEL_THICKNESS / 2.0)),
        material=carcass_finish,
        name="top_panel",
    )
    carcass.visual(
        Box((inner_width, side_depth, SIDE_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, side_center_y, -CABINET_HEIGHT / 2.0 + SIDE_PANEL_THICKNESS / 2.0)),
        material=carcass_finish,
        name="bottom_panel",
    )
    carcass.visual(
        Box((inner_width, shelf_depth, SHELF_THICKNESS)),
        origin=Origin(xyz=(0.0, shelf_center_y, 0.03)),
        material=interior_finish,
        name="center_shelf",
    )
    carcass.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=6.0,
        origin=Origin(),
    )

    door_body_center = (DOOR_LEFT_OFFSET + DOOR_WIDTH / 2.0, DOOR_Y_OFFSET, 0.0)
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=door_body_center),
        material=carcass_finish,
        name="door_body",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.040, MIRROR_THICKNESS, DOOR_HEIGHT - 0.040)),
        origin=Origin(
            xyz=(
                door_body_center[0],
                DOOR_Y_OFFSET + DOOR_THICKNESS / 2.0 + MIRROR_THICKNESS / 2.0,
                0.0,
            )
        ),
        material=mirror_finish,
        name="mirror_panel",
    )
    door.visual(
        Box((0.012, 0.006, 0.030)),
        origin=Origin(xyz=(PULL_PIVOT_X, 0.024, 0.0)),
        material=hinge_finish,
        name="pull_mount_pad",
    )
    door.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(PULL_PIVOT_X, PULL_PIVOT_Y, -0.006)),
        material=hinge_finish,
        name="pull_pivot_lower",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS + MIRROR_THICKNESS, DOOR_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(door_body_center[0], DOOR_Y_OFFSET + MIRROR_THICKNESS / 2.0, 0.0)),
    )

    finger_pull.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=hinge_finish,
        name="pull_pivot_upper",
    )
    finger_pull.visual(
        Box((0.012, 0.012, 0.040)),
        origin=Origin(xyz=(0.006, 0.010, -0.010)),
        material=hinge_finish,
        name="finger_tab",
    )
    finger_pull.inertial = Inertial.from_geometry(
        Box((0.012, 0.016, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.006, 0.008, -0.010)),
    )

    def add_hinge(prefix: str, z_center: float) -> None:
        hinge_start_z = z_center - HINGE_TOTAL_HEIGHT / 2.0

        carcass.visual(
            Box((HINGE_LEAF_THICKNESS, HINGE_PLATE_DEPTH, HINGE_TOTAL_HEIGHT)),
            origin=Origin(
                xyz=(
                    -CABINET_WIDTH / 2.0 - HINGE_LEAF_THICKNESS / 2.0,
                    HINGE_AXIS_Y - 0.010,
                    z_center,
                )
            ),
            material=hinge_finish,
            name=f"{prefix}_hinge_leaf",
        )
        carcass.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_CARCASS_SEGMENT),
            origin=Origin(
                xyz=(
                    HINGE_AXIS_X,
                    HINGE_AXIS_Y,
                    hinge_start_z + HINGE_CARCASS_SEGMENT / 2.0,
                )
            ),
            material=hinge_finish,
            name=f"{prefix}_hinge_knuckle_bottom",
        )
        carcass.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_CARCASS_SEGMENT),
            origin=Origin(
                xyz=(
                    HINGE_AXIS_X,
                    HINGE_AXIS_Y,
                    hinge_start_z
                    + HINGE_CARCASS_SEGMENT
                    + HINGE_DOOR_SEGMENT
                    + HINGE_CARCASS_SEGMENT / 2.0,
                )
            ),
            material=hinge_finish,
            name=f"{prefix}_hinge_knuckle_top",
        )

        door.visual(
            Box((0.010, 0.022, HINGE_TOTAL_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.005,
                    0.0135,
                    z_center,
                )
            ),
            material=hinge_finish,
            name=f"{prefix}_door_hinge_leaf",
        )
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_DOOR_SEGMENT),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    hinge_start_z + HINGE_CARCASS_SEGMENT + HINGE_DOOR_SEGMENT / 2.0,
                )
            ),
            material=hinge_finish,
            name=f"{prefix}_hinge_knuckle_center",
        )

    add_hinge("upper", UPPER_HINGE_Z)
    add_hinge("lower", LOWER_HINGE_Z)

    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.95),
    )
    model.articulation(
        "door_to_finger_pull",
        ArticulationType.REVOLUTE,
        parent=door,
        child=finger_pull,
        origin=Origin(xyz=(PULL_PIVOT_X, PULL_PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part_names = {part.name for part in object_model.parts}
    articulation_names = {articulation.name for articulation in object_model.articulations}

    ctx.check(
        "required_parts_present",
        {"carcass", "door", "finger_pull"} <= part_names,
        f"found parts: {sorted(part_names)}",
    )
    ctx.check(
        "required_articulations_present",
        {"carcass_to_door", "door_to_finger_pull"} <= articulation_names,
        f"found articulations: {sorted(articulation_names)}",
    )

    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    finger_pull = object_model.get_part("finger_pull")
    door_hinge = object_model.get_articulation("carcass_to_door")
    pull_pivot = object_model.get_articulation("door_to_finger_pull")

    door_body = door.get_visual("door_body")
    upper_door_knuckle = door.get_visual("upper_hinge_knuckle_center")
    lower_door_knuckle = door.get_visual("lower_hinge_knuckle_center")
    upper_hinge_bottom = carcass.get_visual("upper_hinge_knuckle_bottom")
    upper_hinge_top = carcass.get_visual("upper_hinge_knuckle_top")
    lower_hinge_bottom = carcass.get_visual("lower_hinge_knuckle_bottom")
    lower_hinge_top = carcass.get_visual("lower_hinge_knuckle_top")
    pull_pivot_lower = door.get_visual("pull_pivot_lower")
    pull_pivot_upper = finger_pull.get_visual("pull_pivot_upper")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door_hinge_axis_vertical",
        door_hinge.axis == (0.0, 0.0, 1.0),
        f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "door_hinge_has_real_swing",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= 1.8,
        f"door hinge limits were {door_hinge.motion_limits}",
    )
    ctx.check(
        "pull_pivot_axis_vertical",
        pull_pivot.axis == (0.0, 0.0, 1.0),
        f"pull pivot axis was {pull_pivot.axis}",
    )
    ctx.check(
        "pull_pivot_is_small_motion",
        pull_pivot.motion_limits is not None
        and pull_pivot.motion_limits.lower is not None
        and pull_pivot.motion_limits.upper is not None
        and pull_pivot.motion_limits.lower <= -0.2
        and pull_pivot.motion_limits.upper >= 0.2
        and pull_pivot.motion_limits.upper <= 0.4,
        f"pull pivot limits were {pull_pivot.motion_limits}",
    )

    ctx.expect_gap(
        door,
        carcass,
        axis="y",
        positive_elem=door_body,
        max_gap=0.004,
        max_penetration=1e-6,
        name="door_closes_against_carcass_front",
    )
    ctx.expect_overlap(
        door,
        carcass,
        axes="xz",
        elem_a=door_body,
        min_overlap=0.30,
        name="door_covers_cabinet_opening",
    )
    ctx.expect_origin_gap(
        finger_pull,
        door,
        axis="x",
        min_gap=0.32,
        max_gap=0.36,
        name="finger_pull_sits_at_free_edge",
    )
    ctx.expect_contact(
        finger_pull,
        door,
        elem_a=pull_pivot_upper,
        elem_b=pull_pivot_lower,
        name="finger_pull_is_captured_on_pivot",
    )

    with ctx.pose({door_hinge: DOOR_OPEN_TEST_ANGLE, pull_pivot: PULL_TEST_ANGLE}):
        ctx.expect_contact(
            door,
            carcass,
            elem_a=upper_door_knuckle,
            elem_b=upper_hinge_bottom,
            name="upper_hinge_stays_captured_low",
        )
        ctx.expect_contact(
            door,
            carcass,
            elem_a=upper_door_knuckle,
            elem_b=upper_hinge_top,
            name="upper_hinge_stays_captured_high",
        )
        ctx.expect_contact(
            door,
            carcass,
            elem_a=lower_door_knuckle,
            elem_b=lower_hinge_bottom,
            name="lower_hinge_stays_captured_low",
        )
        ctx.expect_contact(
            door,
            carcass,
            elem_a=lower_door_knuckle,
            elem_b=lower_hinge_top,
            name="lower_hinge_stays_captured_high",
        )
        ctx.expect_contact(
            finger_pull,
            door,
            elem_a=pull_pivot_upper,
            elem_b=pull_pivot_lower,
            name="finger_pull_stays_captured_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
