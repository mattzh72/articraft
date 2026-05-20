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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

CABINET_WIDTH = 0.90
CABINET_DEPTH = 0.58
COUNTERTOP_WIDTH = 0.94
COUNTERTOP_DEPTH = 0.64
COUNTERTOP_THICKNESS = 0.04
COUNTERTOP_TOP_Z = 0.90
SIDE_THICKNESS = 0.018
BACK_THICKNESS = 0.012
PANEL_BOTTOM_Z = 0.10
CARCASS_TOP_Z = COUNTERTOP_TOP_Z - COUNTERTOP_THICKNESS
DOOR_THICKNESS = 0.020
DOOR_HEIGHT = 0.666
DOOR_BARREL_RADIUS = 0.012
DOOR_PANEL_WIDTH = 0.405
CUTOUT_WIDTH = 0.710
CUTOUT_DEPTH = 0.550
CUTOUT_OFFSET_Y = -0.025
COOKTOP_WIDTH = 0.760
COOKTOP_DEPTH = 0.520
COOKTOP_BODY_WIDTH = 0.700
COOKTOP_BODY_DEPTH = 0.390


def _ellipse_profile(
    radius_x: float,
    radius_y: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 24,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius_x * math.cos((2.0 * math.pi * index) / segments),
            cy + radius_y * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _make_countertop_mesh():
    outer_profile = rounded_rect_profile(
        COUNTERTOP_WIDTH,
        COUNTERTOP_DEPTH,
        radius=0.018,
        corner_segments=8,
    )
    cutout_profile = _offset_profile(
        rounded_rect_profile(
            CUTOUT_WIDTH,
            CUTOUT_DEPTH,
            radius=0.012,
            corner_segments=8,
        ),
        dy=CUTOUT_OFFSET_Y,
    )
    slab = ExtrudeWithHolesGeometry(
        outer_profile,
        [cutout_profile],
        COUNTERTOP_THICKNESS,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(slab, ASSETS.mesh_path("cooktop_countertop.obj"))


def _make_grate_mesh(
    *,
    mesh_name: str,
    width: float,
    depth: float,
    hole_specs: list[tuple[float, float, float, float]],
):
    outer_profile = rounded_rect_profile(width, depth, radius=0.016, corner_segments=8)
    hole_profiles = [
        _ellipse_profile(rx, ry, center=(cx, cy), segments=26)
        for cx, cy, rx, ry in hole_specs
    ]
    grate = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        0.012,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(grate, ASSETS.mesh_path(mesh_name))


def _build_knob_part(
    model: ArticulatedObject,
    *,
    name: str,
    radius: float,
    depth: float,
    material: str,
) -> object:
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=radius, length=depth),
        origin=Origin(xyz=(0.0, -depth / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="knob_body",
    )
    knob.visual(
        Box((radius * 0.20, depth * 0.18, radius * 0.55)),
        origin=Origin(xyz=(0.0, -depth + 0.0025, radius * 0.58)),
        material=material,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=radius, length=depth),
        mass=0.10 if radius > 0.024 else 0.06,
        origin=Origin(xyz=(0.0, -depth / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    return knob


def _build_burner_part(
    model: ArticulatedObject,
    *,
    name: str,
    base_radius: float,
    crown_radius: float,
    cap_radius: float,
    burner_material: str,
    crown_material: str,
) -> object:
    burner = model.part(name)
    burner.visual(
        Cylinder(radius=base_radius, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=burner_material,
        name="burner_base",
    )
    burner.visual(
        Cylinder(radius=crown_radius, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=crown_material,
        name="burner_crown",
    )
    burner.visual(
        Cylinder(radius=cap_radius, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=burner_material,
        name="burner_cap",
    )
    burner.inertial = Inertial.from_geometry(
        Cylinder(radius=base_radius, length=0.023),
        mass=0.25 if base_radius > 0.040 else 0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
    )
    return burner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_cooktop_cabinet", assets=ASSETS)

    cabinet_paint = model.material("cabinet_paint", rgba=(0.87, 0.87, 0.84, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.72, 0.72, 0.69, 1.0))
    counter_stone = model.material("counter_stone", rgba=(0.66, 0.68, 0.70, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.12, 0.12, 0.13, 1.0))
    cooktop_steel = model.material("cooktop_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    burner_black = model.material("burner_black", rgba=(0.16, 0.16, 0.17, 1.0))
    burner_brass = model.material("burner_brass", rgba=(0.67, 0.51, 0.23, 1.0))
    grate_iron = model.material("grate_iron", rgba=(0.20, 0.20, 0.21, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.75, 0.74, 0.70, 1.0))

    cabinet = model.part("cabinet_body")
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CARCASS_TOP_Z - PANEL_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + SIDE_THICKNESS / 2.0,
                0.0,
                (CARCASS_TOP_Z + PANEL_BOTTOM_Z) / 2.0,
            )
        ),
        material=cabinet_paint,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CARCASS_TOP_Z - PANEL_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0,
                0.0,
                (CARCASS_TOP_Z + PANEL_BOTTOM_Z) / 2.0,
            )
        ),
        material=cabinet_paint,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.488, 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                0.034,
                PANEL_BOTTOM_Z + 0.009,
            )
        ),
        material=cabinet_shadow,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, CARCASS_TOP_Z - PANEL_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0 - BACK_THICKNESS / 2.0,
                (CARCASS_TOP_Z + PANEL_BOTTOM_Z) / 2.0,
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.018, 0.10)),
        origin=Origin(xyz=(0.0, -0.205, 0.050)),
        material=cabinet_shadow,
        name="toe_kick",
    )
    cabinet.visual(
        _make_countertop_mesh(),
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_TOP_Z - COUNTERTOP_THICKNESS / 2.0)),
        material=counter_stone,
        name="countertop",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((COUNTERTOP_WIDTH, COUNTERTOP_DEPTH, COUNTERTOP_TOP_Z)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_TOP_Z / 2.0)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=cooktop_glass,
        name="top_flange",
    )
    cooktop.visual(
        Box((COOKTOP_BODY_WIDTH, COOKTOP_BODY_DEPTH, 0.130)),
        origin=Origin(xyz=(0.0, 0.025, -0.065)),
        material=cooktop_steel,
        name="underbox",
    )
    cooktop.visual(
        Box((0.690, 0.040, 0.102)),
        origin=Origin(xyz=(0.0, -0.240, -0.051)),
        material=cooktop_steel,
        name="front_fascia",
    )
    cooktop.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, 0.138)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
    )
    model.articulation(
        "cabinet_to_cooktop",
        ArticulationType.FIXED,
        parent=cabinet,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_TOP_Z)),
    )

    left_grate_mesh = _make_grate_mesh(
        mesh_name="left_burner_grate.obj",
        width=0.228,
        depth=0.255,
        hole_specs=[
            (0.0, -0.082, 0.055, 0.044),
            (0.0, 0.082, 0.055, 0.044),
        ],
    )
    center_grate_mesh = _make_grate_mesh(
        mesh_name="center_burner_grate.obj",
        width=0.205,
        depth=0.205,
        hole_specs=[(0.0, 0.0, 0.072, 0.060)],
    )
    right_grate_mesh = _make_grate_mesh(
        mesh_name="right_burner_grate.obj",
        width=0.228,
        depth=0.255,
        hole_specs=[
            (0.0, -0.082, 0.055, 0.044),
            (0.0, 0.082, 0.055, 0.044),
        ],
    )

    burner_layout = {
        "burner_front_left": (-0.225, -0.118, 0.032, 0.026, 0.021),
        "burner_rear_left": (-0.225, 0.120, 0.032, 0.026, 0.021),
        "burner_center": (0.000, 0.015, 0.046, 0.039, 0.031),
        "burner_front_right": (0.225, -0.118, 0.032, 0.026, 0.021),
        "burner_rear_right": (0.225, 0.120, 0.032, 0.026, 0.021),
    }
    for burner_name, (x_pos, y_pos, base_r, crown_r, cap_r) in burner_layout.items():
        burner = _build_burner_part(
            model,
            name=burner_name,
            base_radius=base_r,
            crown_radius=crown_r,
            cap_radius=cap_r,
            burner_material=burner_black,
            crown_material=burner_brass,
        )
        model.articulation(
            f"cooktop_to_{burner_name}",
            ArticulationType.FIXED,
            parent=cooktop,
            child=burner,
            origin=Origin(xyz=(x_pos, y_pos, 0.008)),
        )

    left_grate = model.part("left_grate")
    left_grate.visual(
        left_grate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=grate_iron,
        name="grate_frame",
    )
    model.articulation(
        "cooktop_to_left_grate",
        ArticulationType.FIXED,
        parent=cooktop,
        child=left_grate,
        origin=Origin(xyz=(-0.225, 0.001, 0.008)),
    )

    center_grate = model.part("center_grate")
    center_grate.visual(
        center_grate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=grate_iron,
        name="grate_frame",
    )
    model.articulation(
        "cooktop_to_center_grate",
        ArticulationType.FIXED,
        parent=cooktop,
        child=center_grate,
        origin=Origin(xyz=(0.0, 0.015, 0.008)),
    )

    right_grate = model.part("right_grate")
    right_grate.visual(
        right_grate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=grate_iron,
        name="grate_frame",
    )
    model.articulation(
        "cooktop_to_right_grate",
        ArticulationType.FIXED,
        parent=cooktop,
        child=right_grate,
        origin=Origin(xyz=(0.225, 0.001, 0.008)),
    )

    knob_specs = [
        ("left_knob_upper", -0.315, -0.018, 0.027, 0.028),
        ("left_knob_lower", -0.315, -0.084, 0.027, 0.028),
        ("right_knob_1", 0.070, -0.056, 0.021, 0.024),
        ("right_knob_2", 0.165, -0.056, 0.021, 0.024),
        ("right_knob_3", 0.260, -0.056, 0.021, 0.024),
    ]
    for knob_name, x_pos, z_pos, radius, depth in knob_specs:
        knob = _build_knob_part(
            model,
            name=knob_name,
            radius=radius,
            depth=depth,
            material=knob_dark,
        )
        model.articulation(
            f"cooktop_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=cooktop,
            child=knob,
            origin=Origin(xyz=(x_pos, -0.260, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=8.0),
        )

    left_door = model.part("left_door")
    left_door.visual(
        Cylinder(radius=DOOR_BARREL_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=cabinet_shadow,
        name="hinge_barrel",
    )
    left_door.visual(
        Box((0.024, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.012, 0.0, DOOR_HEIGHT / 2.0)),
        material=cabinet_shadow,
        name="hinge_stile",
    )
    left_door.visual(
        Box((DOOR_PANEL_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.012 + DOOR_PANEL_WIDTH / 2.0,
                0.0,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=cabinet_paint,
        name="door_panel",
    )
    left_door.visual(
        Box((0.014, 0.018, 0.220)),
        origin=Origin(
            xyz=(
                0.012 + DOOR_PANEL_WIDTH - 0.045,
                -0.019,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=handle_metal,
        name="pull_handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_PANEL_WIDTH + 0.024, 0.040, DOOR_HEIGHT)),
        mass=7.0,
        origin=Origin(
            xyz=(
                (DOOR_PANEL_WIDTH + 0.024) / 2.0,
                -0.006,
                DOOR_HEIGHT / 2.0,
            )
        ),
    )
    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + SIDE_THICKNESS + DOOR_BARREL_RADIUS,
                -0.290,
                PANEL_BOTTOM_Z + 0.018,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-1.75,
            upper=0.0,
        ),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Cylinder(radius=DOOR_BARREL_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=cabinet_shadow,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.024, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-0.012, 0.0, DOOR_HEIGHT / 2.0)),
        material=cabinet_shadow,
        name="hinge_stile",
    )
    right_door.visual(
        Box((DOOR_PANEL_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                -(0.012 + DOOR_PANEL_WIDTH / 2.0),
                0.0,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=cabinet_paint,
        name="door_panel",
    )
    right_door.visual(
        Box((0.014, 0.018, 0.220)),
        origin=Origin(
            xyz=(
                -(0.012 + DOOR_PANEL_WIDTH - 0.045),
                -0.019,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material=handle_metal,
        name="pull_handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_PANEL_WIDTH + 0.024, 0.040, DOOR_HEIGHT)),
        mass=7.0,
        origin=Origin(
            xyz=(
                -((DOOR_PANEL_WIDTH + 0.024) / 2.0),
                -0.006,
                DOOR_HEIGHT / 2.0,
            )
        ),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - SIDE_THICKNESS - DOOR_BARREL_RADIUS,
                -0.290,
                PANEL_BOTTOM_Z + 0.018,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet_body")
    cooktop = object_model.get_part("cooktop")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    knob_names = [
        "left_knob_upper",
        "left_knob_lower",
        "right_knob_1",
        "right_knob_2",
        "right_knob_3",
    ]
    knob_parts = [object_model.get_part(name) for name in knob_names]
    knob_joints = [object_model.get_articulation(f"cooktop_to_{name}") for name in knob_names]
    burner_names = [
        "burner_front_left",
        "burner_rear_left",
        "burner_center",
        "burner_front_right",
        "burner_rear_right",
    ]
    burners = [object_model.get_part(name) for name in burner_names]
    grate_names = ["left_grate", "center_grate", "right_grate"]
    grates = [object_model.get_part(name) for name in grate_names]

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.expect_gap(
        cooktop,
        cabinet,
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem="top_flange",
        negative_elem="countertop",
        name="cooktop_seats_flush_on_countertop",
    )
    ctx.expect_overlap(
        cooktop,
        cabinet,
        axes="xy",
        min_overlap=0.40,
        elem_a="top_flange",
        elem_b="countertop",
        name="cooktop_overlaps_countertop_footprint",
    )

    for burner, burner_name in zip(burners, burner_names):
        ctx.expect_gap(
            burner,
            cooktop,
            axis="z",
            max_gap=0.0,
            max_penetration=0.0,
            positive_elem="burner_base",
            negative_elem="top_flange",
            name=f"{burner_name}_sits_on_cooktop",
        )
        ctx.expect_overlap(
            burner,
            cooktop,
            axes="xy",
            min_overlap=0.03,
            elem_a="burner_base",
            elem_b="top_flange",
            name=f"{burner_name}_inside_cooktop_surface",
        )

    for grate, grate_name in zip(grates, grate_names):
        ctx.expect_gap(
            grate,
            cooktop,
            axis="z",
            max_gap=0.0,
            max_penetration=0.0,
            positive_elem="grate_frame",
            negative_elem="top_flange",
            name=f"{grate_name}_rests_on_cooktop",
        )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_door,
            cabinet,
            elem_a="hinge_barrel",
            elem_b="left_side_panel",
            name="left_door_hinge_contact_closed",
        )
        ctx.expect_contact(
            right_door,
            cabinet,
            elem_a="hinge_barrel",
            elem_b="right_side_panel",
            name="right_door_hinge_contact_closed",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.005,
            max_gap=0.014,
            positive_elem="door_panel",
            negative_elem="door_panel",
            name="cabinet_door_center_gap_closed",
        )
        ctx.expect_gap(
            cooktop,
            left_door,
            axis="z",
            min_gap=0.008,
            max_gap=0.020,
            positive_elem="front_fascia",
            negative_elem="door_panel",
            name="cooktop_clearance_above_closed_doors",
        )

    with ctx.pose({left_hinge: -1.60, right_hinge: 1.60}):
        ctx.fail_if_parts_overlap_in_current_pose(name="doors_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="doors_open_pose_no_floating")
        ctx.expect_contact(
            left_door,
            cabinet,
            elem_a="hinge_barrel",
            elem_b="left_side_panel",
            name="left_door_hinge_contact_open",
        )
        ctx.expect_contact(
            right_door,
            cabinet,
            elem_a="hinge_barrel",
            elem_b="right_side_panel",
            name="right_door_hinge_contact_open",
        )

    for knob_part, knob_joint, knob_name in zip(knob_parts, knob_joints, knob_names):
        limits = knob_joint.motion_limits
        axis_ok = tuple(round(value, 6) for value in knob_joint.axis) == (0.0, 1.0, 0.0)
        continuous_ok = knob_joint.articulation_type == ArticulationType.CONTINUOUS
        limits_ok = limits is not None and limits.lower is None and limits.upper is None
        ctx.check(
            f"{knob_name}_continuous_front_to_back_axis",
            axis_ok and continuous_ok and limits_ok,
            details=(
                f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )
        with ctx.pose({knob_joint: math.pi / 2.0}):
            ctx.expect_contact(
                knob_part,
                cooktop,
                elem_a="knob_body",
                elem_b="front_fascia",
                name=f"{knob_name}_remains_mounted_when_rotated",
            )

    top_left_position = ctx.part_world_position(object_model.get_part("left_knob_upper"))
    lower_left_position = ctx.part_world_position(object_model.get_part("left_knob_lower"))
    small_knob_positions = [
        ctx.part_world_position(object_model.get_part("right_knob_1")),
        ctx.part_world_position(object_model.get_part("right_knob_2")),
        ctx.part_world_position(object_model.get_part("right_knob_3")),
    ]
    burner_positions = {name: ctx.part_world_position(object_model.get_part(name)) for name in burner_names}

    if top_left_position is not None and lower_left_position is not None:
        ctx.check(
            "left_knobs_are_stacked_vertically",
            abs(top_left_position[0] - lower_left_position[0]) <= 0.002
            and 0.055 <= top_left_position[2] - lower_left_position[2] <= 0.080,
            details=f"upper={top_left_position}, lower={lower_left_position}",
        )

    if all(position is not None for position in small_knob_positions):
        xs = [position[0] for position in small_knob_positions if position is not None]
        zs = [position[2] for position in small_knob_positions if position is not None]
        ctx.check(
            "right_knobs_form_horizontal_row",
            xs[0] < xs[1] < xs[2] and min(xs) > 0.0 and max(zs) - min(zs) <= 0.002,
            details=f"positions={small_knob_positions}",
        )

    if all(position is not None for position in burner_positions.values()):
        center = burner_positions["burner_center"]
        ctx.check(
            "burner_layout_reads_as_five_burner_cooktop",
            center is not None
            and abs(center[0]) <= 0.020
            and abs(center[1]) <= 0.040
            and burner_positions["burner_front_left"][0] < -0.15
            and burner_positions["burner_front_right"][0] > 0.15
            and burner_positions["burner_rear_left"][1] > 0.08
            and burner_positions["burner_front_left"][1] < -0.08,
            details=str(burner_positions),
        )

    for hinge_name in ("cabinet_to_left_door", "cabinet_to_right_door"):
        hinge = object_model.get_articulation(hinge_name)
        limits = hinge.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({hinge: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge_name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{hinge_name}_lower_no_floating")
            with ctx.pose({hinge: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge_name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{hinge_name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
