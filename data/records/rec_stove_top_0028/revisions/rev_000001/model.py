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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

COUNTER_WIDTH = 1.20
COUNTER_DEPTH = 0.72
COUNTER_THICKNESS = 0.04

COOKTOP_WIDTH = 0.84
COOKTOP_DEPTH = 0.48
COOKTOP_HEIGHT = 0.028
COOKTOP_BOTTOM_Z = COUNTER_THICKNESS - COOKTOP_HEIGHT
COOKTOP_TOP_Z = COUNTER_THICKNESS

SMALL_KNOB_RADIUS = 0.018
SMALL_KNOB_DEPTH = 0.045
LARGE_KNOB_RADIUS = 0.024
LARGE_KNOB_DEPTH = 0.050


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_burner(
    part,
    *,
    name: str,
    center_xy: tuple[float, float],
    bowl_radius: float,
    cap_radius: float,
    burner_steel,
    cast_iron,
) -> None:
    x, y = center_xy
    bowl_top_z = COOKTOP_TOP_Z - 0.0025
    bowl_depth = 0.006
    bowl_center_z = bowl_top_z - bowl_depth * 0.5
    ring_height = 0.004
    ring_center_z = bowl_top_z + ring_height * 0.5
    cap_height = 0.008
    cap_center_z = bowl_top_z + 0.0055
    crown_height = 0.003
    crown_center_z = cap_center_z + cap_height * 0.5 + crown_height * 0.5 - 0.001
    grate_height = 0.004
    grate_z = cap_center_z + 0.001
    grate_span = bowl_radius * 1.68
    grate_bar = max(0.008, bowl_radius * 0.14)

    part.visual(
        Cylinder(radius=bowl_radius, length=bowl_depth),
        origin=Origin(xyz=(x, y, bowl_center_z)),
        material=cast_iron,
        name=f"{name}_bowl",
    )
    part.visual(
        Cylinder(radius=bowl_radius * 0.82, length=ring_height),
        origin=Origin(xyz=(x, y, ring_center_z)),
        material=burner_steel,
        name=f"{name}_ring",
    )
    part.visual(
        Cylinder(radius=cap_radius, length=cap_height),
        origin=Origin(xyz=(x, y, cap_center_z)),
        material=cast_iron,
        name=f"{name}_cap",
    )
    part.visual(
        Cylinder(radius=cap_radius * 0.62, length=crown_height),
        origin=Origin(xyz=(x, y, crown_center_z)),
        material=burner_steel,
        name=f"{name}_crown",
    )
    part.visual(
        Box((grate_span, grate_bar, grate_height)),
        origin=Origin(xyz=(x, y, grate_z)),
        material=cast_iron,
        name=f"{name}_grate_x",
    )
    part.visual(
        Box((grate_bar, grate_span, grate_height)),
        origin=Origin(xyz=(x, y, grate_z)),
        material=cast_iron,
        name=f"{name}_grate_y",
    )


def _add_knob(
    model: ArticulatedObject,
    *,
    name: str,
    x: float,
    z: float,
    radius: float,
    depth: float,
    knob_material,
    marker_material,
    effort: float,
) -> None:
    knob = model.part(name)
    knob_profile = [
        (0.0, 0.0),
        (radius * 0.76, 0.003),
        (radius * 0.98, 0.008),
        (radius, depth * 0.68),
        (radius * 0.90, depth * 0.84),
        (radius * 0.66, depth * 0.94),
        (radius * 0.22, depth),
    ]
    knob.visual(
        _save_mesh(
            f"{name}.obj",
            LatheGeometry(knob_profile, segments=40).rotate_x(pi * 0.5),
        ),
        origin=Origin(),
        material=knob_material,
        name="body",
    )
    knob.visual(
        Box((radius * 0.18, 0.003, radius * 0.74)),
        origin=Origin(xyz=(0.0, -0.0055, radius * 0.44)),
        material=marker_material,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=radius, length=depth),
        mass=0.10 if radius < 0.020 else 0.14,
        origin=Origin(xyz=(0.0, -depth * 0.5, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        f"{name}_spin",
        ArticulationType.CONTINUOUS,
        parent="cooktop",
        child=knob,
        origin=Origin(xyz=(x, -COOKTOP_DEPTH * 0.5, z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=effort, velocity=8.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_cooktop", assets=ASSETS)

    stone = model.material("stone", rgba=(0.24, 0.23, 0.21, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.65, 0.67, 0.69, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_metal = model.material("knob_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    marker = model.material("marker", rgba=(0.86, 0.86, 0.87, 1.0))

    countertop = model.part("countertop")
    cutout_profile = rounded_rect_profile(COOKTOP_WIDTH, COOKTOP_DEPTH, radius=0.022, corner_segments=10)
    side_band = (COUNTER_WIDTH - COOKTOP_WIDTH) * 0.5
    rear_band = (COUNTER_DEPTH - COOKTOP_DEPTH) * 0.5
    countertop.visual(
        Box((side_band, COUNTER_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=(-(COOKTOP_WIDTH + side_band) * 0.5, 0.0, COUNTER_THICKNESS * 0.5)),
        material=stone,
        name="left_wing",
    )
    countertop.visual(
        Box((side_band, COUNTER_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(xyz=((COOKTOP_WIDTH + side_band) * 0.5, 0.0, COUNTER_THICKNESS * 0.5)),
        material=stone,
        name="right_wing",
    )
    countertop.visual(
        Box((COOKTOP_WIDTH, rear_band, COUNTER_THICKNESS)),
        origin=Origin(xyz=(0.0, COOKTOP_DEPTH * 0.5 + rear_band * 0.5, COUNTER_THICKNESS * 0.5)),
        material=stone,
        name="rear_bridge",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_THICKNESS * 0.5)),
    )

    cooktop = model.part("cooktop")
    cooktop_body_mesh = ExtrudeGeometry(
        cutout_profile,
        COOKTOP_HEIGHT,
        center=False,
    ).translate(0.0, 0.0, COOKTOP_BOTTOM_Z)
    cooktop.visual(
        _save_mesh("cooktop_body.obj", cooktop_body_mesh),
        material=stainless,
        name="body",
    )
    cooktop.visual(
        Box((COOKTOP_WIDTH * 0.88, 0.006, 0.022)),
        origin=Origin(
            xyz=(0.0, -COOKTOP_DEPTH * 0.5 + 0.003, COOKTOP_BOTTOM_Z + 0.012),
        ),
        material=brushed_steel,
        name="front_bezel",
    )
    cooktop.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, COOKTOP_HEIGHT)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_BOTTOM_Z + COOKTOP_HEIGHT * 0.5)),
    )

    burner_specs = [
        ("left_rear_burner", (-0.24, 0.12), 0.060, 0.030),
        ("left_front_burner", (-0.24, -0.08), 0.054, 0.028),
        ("center_burner", (0.0, 0.015), 0.084, 0.040),
        ("right_rear_burner", (0.24, 0.12), 0.060, 0.030),
        ("right_front_burner", (0.24, -0.08), 0.054, 0.028),
    ]
    for burner_name, burner_xy, bowl_radius, cap_radius in burner_specs:
        _add_burner(
            cooktop,
            name=burner_name,
            center_xy=burner_xy,
            bowl_radius=bowl_radius,
            cap_radius=cap_radius,
            burner_steel=brushed_steel,
            cast_iron=cast_iron,
        )

    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(),
    )

    _add_knob(
        model,
        name="left_outer_knob",
        x=-0.30,
        z=COOKTOP_BOTTOM_Z + 0.014,
        radius=SMALL_KNOB_RADIUS,
        depth=SMALL_KNOB_DEPTH,
        knob_material=knob_metal,
        marker_material=marker,
        effort=0.7,
    )
    _add_knob(
        model,
        name="left_inner_knob",
        x=-0.13,
        z=COOKTOP_BOTTOM_Z + 0.014,
        radius=SMALL_KNOB_RADIUS,
        depth=SMALL_KNOB_DEPTH,
        knob_material=knob_metal,
        marker_material=marker,
        effort=0.7,
    )
    _add_knob(
        model,
        name="right_inner_knob",
        x=0.13,
        z=COOKTOP_BOTTOM_Z + 0.014,
        radius=SMALL_KNOB_RADIUS,
        depth=SMALL_KNOB_DEPTH,
        knob_material=knob_metal,
        marker_material=marker,
        effort=0.7,
    )
    _add_knob(
        model,
        name="right_outer_knob",
        x=0.30,
        z=COOKTOP_BOTTOM_Z + 0.014,
        radius=SMALL_KNOB_RADIUS,
        depth=SMALL_KNOB_DEPTH,
        knob_material=knob_metal,
        marker_material=marker,
        effort=0.7,
    )
    _add_knob(
        model,
        name="center_knob",
        x=0.0,
        z=COOKTOP_BOTTOM_Z + 0.004,
        radius=LARGE_KNOB_RADIUS,
        depth=LARGE_KNOB_DEPTH,
        knob_material=knob_metal,
        marker_material=marker,
        effort=0.9,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    knob_names = (
        "left_outer_knob",
        "left_inner_knob",
        "center_knob",
        "right_inner_knob",
        "right_outer_knob",
    )
    knobs = {name: object_model.get_part(name) for name in knob_names}
    joints = {name: object_model.get_articulation(f"{name}_spin") for name in knob_names}

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
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12)

    burner_caps = (
        "left_rear_burner_cap",
        "left_front_burner_cap",
        "center_burner_cap",
        "right_rear_burner_cap",
        "right_front_burner_cap",
    )
    for burner_name in burner_caps:
        ctx.check(
            f"{burner_name}_exists",
            cooktop.get_visual(burner_name) is not None,
            f"Missing cooktop visual {burner_name}.",
        )

    for part_name, part in (("countertop", countertop), ("cooktop", cooktop), *knobs.items()):
        ctx.check(f"{part_name}_present", part is not None, f"Missing part {part_name}.")

    ctx.expect_contact(cooktop, countertop, name="cooktop_contacts_countertop")
    for knob_name, knob in knobs.items():
        ctx.expect_contact(knob, cooktop, name=f"{knob_name}_touches_cooktop")

    for knob_name, joint in joints.items():
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_type",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{joint.name} should be a continuous knob joint.",
        )
        ctx.check(
            f"{joint.name}_axis_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis should be front-to-back along +Y.",
        )
        ctx.check(
            f"{joint.name}_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            f"{joint.name} should rotate continuously without lower/upper limits.",
        )

    for angle_name, angle in (("quarter_turn", pi * 0.5), ("half_turn", pi), ("three_quarter_turn", pi * 1.5)):
        with ctx.pose({joint: angle for joint in joints.values()}):
            for knob_name, knob in knobs.items():
                ctx.expect_contact(knob, cooktop, name=f"{knob_name}_{angle_name}_contact")

    def _extent(aabb, axis_index: int) -> float:
        return aabb[1][axis_index] - aabb[0][axis_index]

    small_knob_aabb = ctx.part_element_world_aabb(knobs["left_outer_knob"], elem="body")
    center_knob_aabb = ctx.part_element_world_aabb(knobs["center_knob"], elem="body")
    center_burner_aabb = ctx.part_element_world_aabb(cooktop, elem="center_burner_cap")
    small_burner_aabb = ctx.part_element_world_aabb(cooktop, elem="left_front_burner_cap")
    countertop_aabb = ctx.part_world_aabb(countertop)
    cooktop_aabb = ctx.part_world_aabb(cooktop)

    ctx.check(
        "center_knob_larger_than_small_knobs",
        small_knob_aabb is not None
        and center_knob_aabb is not None
        and _extent(center_knob_aabb, 0) > _extent(small_knob_aabb, 0) + 0.008,
        "The center knob should visibly read larger than the four small outer knobs.",
    )
    ctx.check(
        "center_burner_larger_than_side_burners",
        center_burner_aabb is not None
        and small_burner_aabb is not None
        and _extent(center_burner_aabb, 0) > _extent(small_burner_aabb, 0) + 0.018,
        "The middle gas burner should read larger than the side burners.",
    )
    ctx.check(
        "cooktop_has_no_deep_chassis_below_counter",
        countertop_aabb is not None
        and cooktop_aabb is not None
        and cooktop_aabb[0][2] >= countertop_aabb[0][2] + 0.008,
        "The installed cooktop should remain shallow within the countertop thickness.",
    )

    knob_positions = {name: ctx.part_world_position(part) for name, part in knobs.items()}
    small_positions = [knob_positions[name] for name in ("left_outer_knob", "left_inner_knob", "right_inner_knob", "right_outer_knob")]
    ctx.check(
        "small_knobs_share_front_edge_line",
        all(pos is not None for pos in small_positions)
        and max(pos[1] for pos in small_positions) - min(pos[1] for pos in small_positions) < 1e-6
        and max(pos[2] for pos in small_positions) - min(pos[2] for pos in small_positions) < 1e-6,
        "The four small knobs should line up evenly along the front edge.",
    )
    center_position = knob_positions["center_knob"]
    ctx.check(
        "center_knob_is_lower_and_centered",
        center_position is not None
        and abs(center_position[0]) < 1e-6
        and small_positions[0] is not None
        and center_position[2] < small_positions[0][2] - 0.008,
        "The larger center knob should sit slightly lower than the four small knobs.",
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
