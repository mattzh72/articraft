from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

WIDTH = 0.76
DEPTH = 0.62
BODY_HEIGHT = 0.91
TOTAL_HEIGHT = 1.08
SIDE_WALL = 0.03
BACK_WALL = 0.025
COOKTOP_THICKNESS = 0.03
PANEL_THICKNESS = 0.03
PLINTH_HEIGHT = 0.07
PLINTH_FRONT_SETBACK = 0.07
PLINTH_DEPTH = DEPTH - PLINTH_FRONT_SETBACK
OVEN_CAVITY_FRONT_SETBACK = 0.055
OVEN_CAVITY_DEPTH = DEPTH - BACK_WALL - OVEN_CAVITY_FRONT_SETBACK

DOOR_WIDTH = WIDTH - 2.0 * SIDE_WALL
DOOR_THICKNESS = 0.04
DOOR_BOTTOM = 0.075
DOOR_HEIGHT = 0.56
DOOR_TOP = DOOR_BOTTOM + DOOR_HEIGHT
HINGE_RADIUS = 0.008
DOOR_WINDOW_WIDTH = 0.44
DOOR_WINDOW_HEIGHT = 0.23
DOOR_WINDOW_THICKNESS = 0.006
DOOR_WINDOW_Z = 0.29
HANDLE_BAR_RADIUS = 0.011
HANDLE_MOUNT_WIDTH = 0.03
HANDLE_MOUNT_DEPTH = 0.016
HANDLE_MOUNT_HEIGHT = 0.036

KNOB_RADIUS = 0.022
KNOB_LENGTH = 0.036
KNOB_FACE_RADIUS = 0.018
KNOB_FACE_LENGTH = 0.016
KNOB_SKIRT_LENGTH = 0.02
KNOB_Z = 0.76
KNOB_XS = (-0.24, -0.08, 0.08, 0.24)

BURNER_XS = (-0.18, 0.18)
BURNER_YS = (-0.14, 0.12)
BURNER_RADIUS = 0.05
BURNER_HEIGHT = 0.01
BURNER_CAP_RADIUS = 0.022
BURNER_CAP_HEIGHT = 0.014


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_gas_stove", assets=ASSETS)

    enamel = model.material("enamel_white", rgba=(0.95, 0.95, 0.93, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    burner_black = model.material("burner_black", rgba=(0.08, 0.08, 0.09, 1.0))
    stainless = model.material("stainless", rgba=(0.64, 0.66, 0.68, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.15, 0.18, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        Box((SIDE_WALL, DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + SIDE_WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="left_side",
    )
    body.visual(
        Box((SIDE_WALL, DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - SIDE_WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="right_side",
    )
    body.visual(
        Box((WIDTH - 2.0 * SIDE_WALL, BACK_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - BACK_WALL / 2.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="back_panel",
    )
    body.visual(
        Box((WIDTH, PLINTH_DEPTH, PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + PLINTH_FRONT_SETBACK + PLINTH_DEPTH / 2.0,
                PLINTH_HEIGHT / 2.0,
            )
        ),
        material=dark_steel,
        name="plinth",
    )
    body.visual(
        Box((WIDTH, DEPTH, COOKTOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - COOKTOP_THICKNESS / 2.0)),
        material=stainless,
        name="cooktop",
    )
    body.visual(
        Box((WIDTH, PANEL_THICKNESS, BODY_HEIGHT - COOKTOP_THICKNESS - DOOR_TOP)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + PANEL_THICKNESS / 2.0,
                (DOOR_TOP + (BODY_HEIGHT - COOKTOP_THICKNESS)) / 2.0,
            )
        ),
        material=enamel,
        name="control_panel",
    )
    body.visual(
        Box((WIDTH, 0.03, TOTAL_HEIGHT - BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.015, BODY_HEIGHT + (TOTAL_HEIGHT - BODY_HEIGHT) / 2.0)),
        material=enamel,
        name="backguard",
    )
    body.visual(
        Box((DOOR_WIDTH, OVEN_CAVITY_DEPTH, 0.03)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + OVEN_CAVITY_FRONT_SETBACK + OVEN_CAVITY_DEPTH / 2.0,
                (PLINTH_HEIGHT + 0.10) / 2.0,
            )
        ),
        material=glass_dark,
        name="oven_floor",
    )
    body.visual(
        Box((DOOR_WIDTH, OVEN_CAVITY_DEPTH, 0.045)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + OVEN_CAVITY_FRONT_SETBACK + OVEN_CAVITY_DEPTH / 2.0,
                DOOR_TOP - 0.045 / 2.0,
            )
        ),
        material=glass_dark,
        name="oven_ceiling",
    )

    burner_names = (
        "burner_front_left",
        "burner_front_right",
        "burner_rear_left",
        "burner_rear_right",
    )
    burner_positions = (
        (BURNER_XS[0], BURNER_YS[0]),
        (BURNER_XS[1], BURNER_YS[0]),
        (BURNER_XS[0], BURNER_YS[1]),
        (BURNER_XS[1], BURNER_YS[1]),
    )
    for burner_name, (bx, by) in zip(burner_names, burner_positions):
        body.visual(
            Cylinder(radius=BURNER_RADIUS, length=BURNER_HEIGHT),
            origin=Origin(xyz=(bx, by, BODY_HEIGHT + BURNER_HEIGHT / 2.0)),
            material=burner_black,
            name=burner_name,
        )
        body.visual(
            Cylinder(radius=BURNER_CAP_RADIUS, length=BURNER_CAP_HEIGHT),
            origin=Origin(xyz=(bx, by, BODY_HEIGHT + BURNER_HEIGHT + BURNER_CAP_HEIGHT / 2.0 - 0.002)),
            material=dark_steel,
            name=f"{burner_name}_cap",
        )
        body.visual(
            Box((0.125, 0.012, 0.006)),
            origin=Origin(xyz=(bx, by, BODY_HEIGHT + BURNER_HEIGHT + BURNER_CAP_HEIGHT - 0.001)),
            material=dark_steel,
            name=f"{burner_name}_grate_x",
        )
        body.visual(
            Box((0.012, 0.125, 0.006)),
            origin=Origin(xyz=(bx, by, BODY_HEIGHT + BURNER_HEIGHT + BURNER_CAP_HEIGHT - 0.001)),
            material=dark_steel,
            name=f"{burner_name}_grate_y",
        )

    door = model.part("oven_door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0 - HINGE_RADIUS)),
        material=enamel,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_WINDOW_WIDTH, DOOR_WINDOW_THICKNESS, DOOR_WINDOW_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -DOOR_THICKNESS - DOOR_WINDOW_THICKNESS / 2.0,
                DOOR_WINDOW_Z,
            )
        ),
        material=glass_dark,
        name="door_window",
    )
    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_rod",
    )
    door.visual(
        Cylinder(radius=HANDLE_BAR_RADIUS, length=0.42),
        origin=Origin(
            xyz=(0.0, -0.065, DOOR_HEIGHT - 0.105),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=stainless,
        name="handle_bar",
    )
    for sign, mount_name in ((-1.0, "handle_mount_left"), (1.0, "handle_mount_right")):
        door.visual(
            Box((HANDLE_MOUNT_WIDTH, HANDLE_MOUNT_DEPTH, HANDLE_MOUNT_HEIGHT)),
            origin=Origin(
                xyz=(
                    sign * 0.18,
                    -DOOR_THICKNESS - HANDLE_MOUNT_DEPTH / 2.0,
                    DOOR_HEIGHT - 0.105,
                )
            ),
            material=stainless,
            name=mount_name,
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, DOOR_BOTTOM + HINGE_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    for index, knob_x in enumerate(KNOB_XS, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_SKIRT_LENGTH),
            origin=Origin(xyz=(0.0, -KNOB_SKIRT_LENGTH / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=burner_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=KNOB_FACE_RADIUS, length=KNOB_FACE_LENGTH),
            origin=Origin(
                xyz=(0.0, -KNOB_SKIRT_LENGTH - KNOB_FACE_LENGTH / 2.0, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name="knob_face",
        )
        knob.visual(
            Box((0.008, 0.004, 0.014)),
            origin=Origin(
                xyz=(
                    0.0,
                    -KNOB_SKIRT_LENGTH - KNOB_FACE_LENGTH - 0.002,
                    KNOB_FACE_RADIUS - 0.007,
                )
            ),
            material=stainless,
            name="indicator",
        )
        model.articulation(
            f"knob_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, -DEPTH / 2.0, KNOB_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    body = object_model.get_part("body")
    door = object_model.get_part("oven_door")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 5)]
    door_hinge = object_model.get_articulation("door_hinge")
    knob_joints = [object_model.get_articulation(f"knob_{index}_spin") for index in range(1, 5)]

    control_panel = body.get_visual("control_panel")
    cooktop = body.get_visual("cooktop")
    burner_visuals = [body.get_visual(name) for name in (
        "burner_front_left",
        "burner_front_right",
        "burner_rear_left",
        "burner_rear_right",
    )]
    door_panel = door.get_visual("door_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.check("door_part_present", door is not None)
    for index, knob in enumerate(knobs, start=1):
        ctx.check(f"knob_{index}_present", knob is not None)

    ctx.check(
        "door_hinge_axis_is_left_right",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {door_hinge.axis}",
    )
    ctx.check(
        "door_hinge_is_revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"expected REVOLUTE, got {door_hinge.articulation_type}",
    )

    for index, knob_joint in enumerate(knob_joints, start=1):
        ctx.check(
            f"knob_{index}_axis_is_front_to_back",
            tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
            details=f"expected (0, 1, 0), got {knob_joint.axis}",
        )
        ctx.check(
            f"knob_{index}_is_continuous",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"expected CONTINUOUS, got {knob_joint.articulation_type}",
        )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            negative_elem=door_panel,
            name="door_closed_flush_to_front",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="xz",
            min_overlap=0.40,
            elem_b=door_panel,
            name="door_covers_front_opening",
        )

        for index, knob in enumerate(knobs, start=1):
            ctx.expect_gap(
                body,
                knob,
                axis="y",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=control_panel,
                name=f"knob_{index}_mounted_to_panel",
            )
            ctx.expect_overlap(
                body,
                knob,
                axes="xz",
                min_overlap=0.015,
                elem_a=control_panel,
                name=f"knob_{index}_overlaps_panel_face",
            )

        burner_centers = []
        for burner_visual in burner_visuals:
            burner_aabb = ctx.part_element_world_aabb(body, elem=burner_visual)
            burner_centers.append(_aabb_center(burner_aabb) if burner_aabb is not None else None)
            ctx.expect_gap(
                body,
                body,
                axis="z",
                positive_elem=burner_visual,
                negative_elem=cooktop,
                max_gap=0.0,
                max_penetration=0.0,
                name=f"{burner_visual.name}_seats_on_cooktop",
            )

        knob_x_positions = [ctx.part_world_position(knob)[0] for knob in knobs]
        knob_spacings = [knob_x_positions[i + 1] - knob_x_positions[i] for i in range(3)]
        ctx.check(
            "knobs_evenly_spaced",
            max(knob_spacings) - min(knob_spacings) <= 1e-6,
            details=f"knob x spacings were {knob_spacings}",
        )

        burner_ok = all(center is not None for center in burner_centers)
        if burner_ok:
            front_left, front_right, rear_left, rear_right = burner_centers
            ctx.check(
                "burners_form_two_left_right_pairs",
                abs(front_left[0] - rear_left[0]) <= 0.002
                and abs(front_right[0] - rear_right[0]) <= 0.002,
                details=f"burner x centers were {burner_centers}",
            )
            ctx.check(
                "burners_form_front_and_rear_rows",
                front_left[1] < rear_left[1] and front_right[1] < rear_right[1],
                details=f"burner y centers were {burner_centers}",
            )
        else:
            ctx.fail("burner_visuals_have_aabbs", "one or more burner visuals did not produce an AABB")

        closed_door_aabb = ctx.part_element_world_aabb(door, elem=door_panel)

    for index, (knob_joint, knob) in enumerate(zip(knob_joints, knobs), start=1):
        with ctx.pose({knob_joint: pi / 2.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"knob_{index}_quarter_turn_no_overlap")
            ctx.expect_gap(
                body,
                knob,
                axis="y",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=control_panel,
                name=f"knob_{index}_quarter_turn_stays_mounted",
            )

    limits = door_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({door_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_lower_no_floating")
        with ctx.pose({door_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_upper_no_floating")
            open_door_aabb = ctx.part_element_world_aabb(door, elem=door_panel)
            if closed_door_aabb is not None and open_door_aabb is not None:
                closed_center = _aabb_center(closed_door_aabb)
                open_center = _aabb_center(open_door_aabb)
                ctx.check(
                    "door_swings_forward_and_down",
                    open_center[1] < closed_center[1] - 0.12 and open_center[2] < closed_center[2] - 0.15,
                    details=f"closed center {closed_center}, open center {open_center}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
