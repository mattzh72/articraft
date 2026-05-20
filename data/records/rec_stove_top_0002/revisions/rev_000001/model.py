from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


COUNTER_WIDTH = 0.92
COUNTER_DEPTH = 0.64
COUNTER_THICKNESS = 0.038

COOKTOP_WIDTH = 0.562
COOKTOP_DEPTH = 0.462
COOKTOP_PANEL_THICKNESS = 0.006
COOKTOP_TRAY_WIDTH = 0.550
COOKTOP_TRAY_DEPTH = 0.450
COOKTOP_TRAY_THICKNESS = 0.026

BRIDGE_WIDTH = 0.430
BRIDGE_DEPTH = 0.065
BRIDGE_THICKNESS = 0.010
BRIDGE_CENTER_Y = -0.155

KNOB_RADIUS = 0.020
KNOB_LENGTH = 0.018
KNOB_SKIRT_RADIUS = 0.013
KNOB_SKIRT_HEIGHT = 0.010
KNOB_CAP_RADIUS = 0.017
KNOB_CAP_HEIGHT = 0.010
KNOB_CENTER_Z = (
    (COUNTER_THICKNESS * 0.5)
    + BRIDGE_THICKNESS
    + KNOB_RADIUS
)
KNOB_XS = (-0.150, -0.050, 0.050, 0.150)

LEFT_BURNER_X = -0.150
RIGHT_BURNER_X = 0.150
REAR_BURNER_Y = 0.090
FRONT_BURNER_Y = -0.020


def _add_burner(
    part,
    *,
    name: str,
    x: float,
    y: float,
    grate_material,
    burner_material,
) -> None:
    part.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(x, y, 0.021)),
        material=burner_material,
        name=f"{name}_base",
    )
    part.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(xyz=(x, y, 0.025)),
        material=burner_material,
        name=f"{name}_ring",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(x, y, 0.032)),
        material=burner_material,
        name=f"{name}_cap",
    )
    part.visual(
        Box((0.098, 0.010, 0.006)),
        origin=Origin(xyz=(x, y, 0.039)),
        material=grate_material,
        name=f"{name}_grate_x",
    )
    part.visual(
        Box((0.010, 0.098, 0.006)),
        origin=Origin(xyz=(x, y, 0.039)),
        material=grate_material,
        name=f"{name}_grate_y",
    )
    for dx, dy in ((0.028, 0.028), (0.028, -0.028), (-0.028, 0.028), (-0.028, -0.028)):
        part.visual(
            Box((0.010, 0.010, 0.008)),
            origin=Origin(xyz=(x + dx, y + dy, 0.031)),
            material=grate_material,
            name=f"{name}_foot_{'p' if dx > 0 else 'n'}x_{'p' if dy > 0 else 'n'}y",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_burner_gas_cooktop", assets=ASSETS)

    stone = model.material("stone", rgba=(0.78, 0.78, 0.75, 1.0))
    glass_black = model.material("glass_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.67, 0.68, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.24, 0.26, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.16, 0.16, 0.17, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.80, 0.82, 0.84, 1.0))

    countertop = model.part("countertop")
    countertop.visual(
        Box((COUNTER_WIDTH, (COUNTER_DEPTH - COOKTOP_DEPTH) * 0.5, COUNTER_THICKNESS)),
        origin=Origin(xyz=(0.0, -(COOKTOP_DEPTH + COUNTER_DEPTH) * 0.25, 0.0)),
        material=stone,
        name="countertop_front",
    )
    countertop.visual(
        Box((COUNTER_WIDTH, (COUNTER_DEPTH - COOKTOP_DEPTH) * 0.5, COUNTER_THICKNESS)),
        origin=Origin(xyz=(0.0, (COOKTOP_DEPTH + COUNTER_DEPTH) * 0.25, 0.0)),
        material=stone,
        name="countertop_back",
    )
    countertop.visual(
        Box(((COUNTER_WIDTH - COOKTOP_WIDTH) * 0.5, COOKTOP_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=(-(COOKTOP_WIDTH + COUNTER_WIDTH) * 0.25, 0.0, 0.0)),
        material=stone,
        name="countertop_left",
    )
    countertop.visual(
        Box(((COUNTER_WIDTH - COOKTOP_WIDTH) * 0.5, COOKTOP_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=((COOKTOP_WIDTH + COUNTER_WIDTH) * 0.25, 0.0, 0.0)),
        material=stone,
        name="countertop_right",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS)),
        mass=18.0,
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, COOKTOP_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, (COUNTER_THICKNESS * 0.5) - (COOKTOP_PANEL_THICKNESS * 0.5))
        ),
        material=glass_black,
        name="deck_panel",
    )
    cooktop.visual(
        Box((COOKTOP_TRAY_WIDTH, COOKTOP_TRAY_DEPTH, COOKTOP_TRAY_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="shallow_tray",
    )
    cooktop.visual(
        Box((BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BRIDGE_CENTER_Y,
                (COUNTER_THICKNESS * 0.5) + (BRIDGE_THICKNESS * 0.5),
            )
        ),
        material=dark_metal,
        name="control_bridge",
    )
    cooktop.visual(
        Box((0.515, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.115, 0.022)),
        material=steel,
        name="trim_bar",
    )

    _add_burner(
        cooktop,
        name="rear_left_burner",
        x=LEFT_BURNER_X,
        y=REAR_BURNER_Y,
        grate_material=cast_iron,
        burner_material=dark_metal,
    )
    _add_burner(
        cooktop,
        name="rear_right_burner",
        x=RIGHT_BURNER_X,
        y=REAR_BURNER_Y,
        grate_material=cast_iron,
        burner_material=dark_metal,
    )
    _add_burner(
        cooktop,
        name="front_left_burner",
        x=LEFT_BURNER_X,
        y=FRONT_BURNER_Y,
        grate_material=cast_iron,
        burner_material=dark_metal,
    )
    _add_burner(
        cooktop,
        name="front_right_burner",
        x=RIGHT_BURNER_X,
        y=FRONT_BURNER_Y,
        grate_material=cast_iron,
        burner_material=dark_metal,
    )
    cooktop.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, 0.060)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(),
    )

    for index, knob_x in enumerate(KNOB_XS, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=KNOB_CAP_RADIUS, length=KNOB_LENGTH * 0.86),
            origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_silver,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=KNOB_SKIRT_RADIUS, length=KNOB_SKIRT_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=dark_metal,
            name="knob_skirt",
        )
        knob.visual(
            Box((0.004, 0.002, 0.010)),
            origin=Origin(xyz=(0.0, -0.010, 0.010)),
            material=satin_silver,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
            mass=0.08,
        )
        model.articulation(
            f"cooktop_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=cooktop,
            child=knob,
            origin=Origin(xyz=(knob_x, BRIDGE_CENTER_Y, KNOB_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    knob_parts = [object_model.get_part(f"knob_{index}") for index in range(1, 5)]
    knob_joints = [object_model.get_articulation(f"cooktop_to_knob_{index}") for index in range(1, 5)]

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_knob_support")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="sampled_knob_clearance")

    ctx.expect_contact(
        cooktop,
        countertop,
        contact_tol=0.0005,
        name="cooktop_seated_in_countertop_cutout",
    )

    for index, knob in enumerate(knob_parts, start=1):
        ctx.expect_contact(
            knob,
            cooktop,
            contact_tol=0.0005,
            name=f"knob_{index}_mounted_contact_rest",
        )

    for joint in knob_joints:
        limits = joint.motion_limits
        axis_ok = tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0)
        type_ok = joint.articulation_type == ArticulationType.CONTINUOUS
        limits_ok = limits is not None and limits.lower is None and limits.upper is None
        ctx.check(
            f"{joint.name}_continuous_front_to_back_axis",
            axis_ok and type_ok and limits_ok,
            details=(
                f"type={joint.articulation_type!r} axis={joint.axis!r} "
                f"limits=({None if limits is None else limits.lower}, "
                f"{None if limits is None else limits.upper})"
            ),
        )

    deck_aabb = ctx.part_element_world_aabb(cooktop, elem="deck_panel")
    countertop_aabb = ctx.part_world_aabb(countertop)
    cooktop_aabb = ctx.part_world_aabb(cooktop)
    assert countertop_aabb is not None
    assert deck_aabb is not None
    assert cooktop_aabb is not None

    countertop_top = countertop_aabb[1][2]
    deck_top = deck_aabb[1][2]
    countertop_bottom = countertop_aabb[0][2]
    cooktop_bottom = cooktop_aabb[0][2]

    ctx.check(
        "cooktop_deck_nearly_flush_with_countertop",
        abs(deck_top - countertop_top) <= 0.001,
        details=f"deck_top={deck_top:.4f} countertop_top={countertop_top:.4f}",
    )
    ctx.check(
        "cooktop_has_no_body_below_counter_underside",
        cooktop_bottom >= countertop_bottom - 0.0005,
        details=f"cooktop_bottom={cooktop_bottom:.4f} countertop_bottom={countertop_bottom:.4f}",
    )

    knob_positions = [ctx.part_world_position(knob) for knob in knob_parts]
    assert all(position is not None for position in knob_positions)
    knob_positions = [position for position in knob_positions if position is not None]
    same_row_ok = (
        max(position[1] for position in knob_positions) - min(position[1] for position in knob_positions)
        <= 1e-6
        and max(position[2] for position in knob_positions) - min(position[2] for position in knob_positions)
        <= 1e-6
    )
    spacing = [knob_positions[index + 1][0] - knob_positions[index][0] for index in range(3)]
    spacing_ok = max(spacing) - min(spacing) <= 0.002 and min(spacing) >= 0.090
    ctx.check(
        "knobs_form_straight_even_front_row",
        same_row_ok and spacing_ok,
        details=f"positions={knob_positions!r} spacing={spacing!r}",
    )

    burner_caps = {
        "front_left": ctx.part_element_world_aabb(cooktop, elem="front_left_burner_cap"),
        "front_right": ctx.part_element_world_aabb(cooktop, elem="front_right_burner_cap"),
        "rear_left": ctx.part_element_world_aabb(cooktop, elem="rear_left_burner_cap"),
        "rear_right": ctx.part_element_world_aabb(cooktop, elem="rear_right_burner_cap"),
    }
    assert all(box is not None for box in burner_caps.values())

    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    burner_centers = {name: _aabb_center(box) for name, box in burner_caps.items() if box is not None}
    burners_ok = (
        burner_centers["front_left"][0] < 0.0
        and burner_centers["front_right"][0] > 0.0
        and burner_centers["rear_left"][1] > burner_centers["front_left"][1] + 0.08
        and burner_centers["rear_right"][1] > burner_centers["front_right"][1] + 0.08
        and abs(burner_centers["front_left"][0] + burner_centers["front_right"][0]) <= 0.005
        and abs(burner_centers["rear_left"][0] + burner_centers["rear_right"][0]) <= 0.005
    )
    ctx.check(
        "four_burners_in_two_by_two_layout",
        burners_ok,
        details=f"burner_centers={burner_centers!r}",
    )

    indicator_rest = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")
    assert indicator_rest is not None
    rest_center = _aabb_center(indicator_rest)

    for index, (knob, joint) in enumerate(zip(knob_parts, knob_joints), start=1):
        with ctx.pose({joint: math.pi / 2.0}):
            ctx.expect_contact(
                knob,
                cooktop,
                contact_tol=0.0005,
                name=f"knob_{index}_mounted_contact_quarter_turn",
            )

    with ctx.pose({knob_joints[0]: math.pi / 2.0}):
        indicator_quarter_turn = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")
        assert indicator_quarter_turn is not None
        turned_center = _aabb_center(indicator_quarter_turn)
        indicator_motion_ok = (
            abs(turned_center[0] - rest_center[0]) >= 0.007
            and abs(turned_center[1] - rest_center[1]) <= 1e-6
            and abs(turned_center[2] - rest_center[2]) >= 0.007
        )
        ctx.check(
            "knob_indicator_visibly_rotates_with_joint",
            indicator_motion_ok,
            details=f"rest_center={rest_center!r} turned_center={turned_center!r}",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
