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
    ExtrudeWithHolesGeometry,
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

COUNTERTOP_WIDTH = 1.20
COUNTERTOP_DEPTH = 0.68
COUNTERTOP_THICKNESS = 0.04
CUTOUT_WIDTH = 0.83
CUTOUT_DEPTH = 0.48
CUTOUT_NOTCH_WIDTH = 0.80
CUTOUT_NOTCH_FRONT_Y = -0.30

HOB_WIDTH = 0.86
HOB_DEPTH = 0.51
HOB_PLATE_THICKNESS = 0.008
HOB_CORNER_RADIUS = 0.022
CONTROL_APRON_WIDTH = 0.78

PLATE_BOTTOM_Z = COUNTERTOP_THICKNESS
PLATE_TOP_Z = PLATE_BOTTOM_Z + HOB_PLATE_THICKNESS

APRON_THICKNESS = 0.008
APRON_HEIGHT = 0.024
APRON_CENTER_Y = -HOB_DEPTH / 2.0 + APRON_THICKNESS / 2.0
APRON_FRONT_Y = APRON_CENTER_Y - APRON_THICKNESS / 2.0
APRON_CENTER_Z = PLATE_BOTTOM_Z - APRON_HEIGHT / 2.0

KNOB_CENTER_Z = APRON_CENTER_Z
MEDIUM_KNOB_RADIUS = 0.019
MEDIUM_KNOB_DEPTH = 0.034
LARGE_KNOB_RADIUS = 0.024
LARGE_KNOB_DEPTH = 0.040


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _countertop_cutout_profile() -> list[tuple[float, float]]:
    half_width = CUTOUT_WIDTH / 2.0
    half_depth = CUTOUT_DEPTH / 2.0
    notch_half_width = CUTOUT_NOTCH_WIDTH / 2.0
    return [
        (-half_width, -half_depth),
        (-notch_half_width, -half_depth),
        (-notch_half_width, CUTOUT_NOTCH_FRONT_Y),
        (notch_half_width, CUTOUT_NOTCH_FRONT_Y),
        (notch_half_width, -half_depth),
        (half_width, -half_depth),
        (half_width, half_depth),
        (-half_width, half_depth),
    ]


def _knob_body_mesh(name: str, *, radius: float, depth: float):
    profile = [
        (0.0, -depth / 2.0),
        (radius * 0.56, -depth / 2.0),
        (radius * 0.72, -depth * 0.42),
        (radius * 0.92, -depth * 0.10),
        (radius, depth * 0.10),
        (radius * 0.96, depth * 0.26),
        (radius * 0.80, depth * 0.40),
        (radius * 0.42, depth / 2.0),
        (0.0, depth / 2.0),
    ]
    return _save_mesh(name, LatheGeometry(profile, segments=56, closed=True))


def _burner_stack_z(cup_height: float, ring_height: float, cap_height: float, support_height: float) -> dict[str, float]:
    cup_center = PLATE_TOP_Z + cup_height / 2.0
    ring_center = PLATE_TOP_Z + cup_height + ring_height / 2.0
    cap_center = PLATE_TOP_Z + cup_height + ring_height + cap_height / 2.0
    support_center = PLATE_TOP_Z + cup_height + ring_height + cap_height + support_height / 2.0
    return {
        "cup": cup_center,
        "ring": ring_center,
        "cap": cap_center,
        "support": support_center,
    }


def _add_burner_visuals(
    part,
    *,
    prefix: str,
    x: float,
    y: float,
    cup_radius: float,
    ring_radius: float,
    cap_radius: float,
    support_span: float,
    burner_metal,
    cap_material,
    trivet_material,
) -> None:
    cup_height = 0.010
    ring_height = 0.006
    cap_height = 0.010
    support_height = 0.008
    z = _burner_stack_z(cup_height, ring_height, cap_height, support_height)

    part.visual(
        Cylinder(radius=cup_radius, length=cup_height),
        origin=Origin(xyz=(x, y, z["cup"])),
        material=burner_metal,
        name=f"{prefix}_cup",
    )
    part.visual(
        Cylinder(radius=ring_radius, length=ring_height),
        origin=Origin(xyz=(x, y, z["ring"])),
        material=burner_metal,
        name=f"{prefix}_ring",
    )
    part.visual(
        Cylinder(radius=cap_radius, length=cap_height),
        origin=Origin(xyz=(x, y, z["cap"])),
        material=cap_material,
        name=f"{prefix}_cap",
    )
    part.visual(
        Box((support_span, 0.014, support_height)),
        origin=Origin(xyz=(x, y, z["support"])),
        material=trivet_material,
        name=f"{prefix}_support_x",
    )
    part.visual(
        Box((0.014, support_span, support_height)),
        origin=Origin(xyz=(x, y, z["support"])),
        material=trivet_material,
        name=f"{prefix}_support_y",
    )


def _add_knob(
    model: ArticulatedObject,
    parent,
    *,
    part_name: str,
    joint_name: str,
    center_x: float,
    radius: float,
    depth: float,
    knob_material,
    indicator_material,
) -> None:
    knob = model.part(part_name)
    knob.visual(
        _knob_body_mesh(f"{part_name}_body.obj", radius=radius, depth=depth),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_body",
    )
    knob.visual(
        Box((radius * 0.16, 0.0022, radius * 0.82)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.0011, radius * 0.38)),
        material=indicator_material,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=radius, length=depth),
        mass=0.09 if radius > MEDIUM_KNOB_RADIUS else 0.06,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent=parent,
        child=knob,
        origin=Origin(xyz=(center_x, APRON_FRONT_Y - depth / 2.0, KNOB_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_hob", assets=ASSETS)

    countertop_stone = model.material("countertop_stone", rgba=(0.74, 0.73, 0.70, 1.0))
    hob_glass = model.material("hob_glass", rgba=(0.08, 0.08, 0.09, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.10, 0.10, 0.11, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    burner_cap = model.material("burner_cap", rgba=(0.14, 0.14, 0.15, 1.0))
    trivet_iron = model.material("trivet_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    knob_stainless = model.material("knob_stainless", rgba=(0.70, 0.71, 0.73, 1.0))
    knob_mark = model.material("knob_mark", rgba=(0.95, 0.37, 0.15, 1.0))

    countertop = model.part("countertop")
    countertop_mesh = _save_mesh(
        "countertop_frame.obj",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(COUNTERTOP_WIDTH, COUNTERTOP_DEPTH, 0.012, corner_segments=8),
            [_countertop_cutout_profile()],
            COUNTERTOP_THICKNESS,
            center=False,
        ),
    )
    countertop.visual(countertop_mesh, material=countertop_stone, name="countertop_frame")
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTERTOP_WIDTH, COUNTERTOP_DEPTH, COUNTERTOP_THICKNESS)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_THICKNESS / 2.0)),
    )

    hob_surface = model.part("hob_surface")
    hob_plate_mesh = _save_mesh(
        "hob_surface_plate.obj",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(HOB_WIDTH, HOB_DEPTH, HOB_CORNER_RADIUS, corner_segments=10),
            HOB_PLATE_THICKNESS,
        ),
    )
    hob_surface.visual(
        hob_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, PLATE_BOTTOM_Z)),
        material=hob_glass,
        name="surface_plate",
    )
    hob_surface.visual(
        Box((CONTROL_APRON_WIDTH, APRON_THICKNESS, APRON_HEIGHT)),
        origin=Origin(xyz=(0.0, APRON_CENTER_Y, APRON_CENTER_Z)),
        material=fascia_black,
        name="control_apron",
    )

    _add_burner_visuals(
        hob_surface,
        prefix="burner_back_left",
        x=-0.265,
        y=0.125,
        cup_radius=0.062,
        ring_radius=0.045,
        cap_radius=0.036,
        support_span=0.132,
        burner_metal=burner_metal,
        cap_material=burner_cap,
        trivet_material=trivet_iron,
    )
    _add_burner_visuals(
        hob_surface,
        prefix="burner_front_left",
        x=-0.285,
        y=-0.090,
        cup_radius=0.056,
        ring_radius=0.041,
        cap_radius=0.032,
        support_span=0.118,
        burner_metal=burner_metal,
        cap_material=burner_cap,
        trivet_material=trivet_iron,
    )
    _add_burner_visuals(
        hob_surface,
        prefix="burner_center",
        x=0.0,
        y=0.020,
        cup_radius=0.088,
        ring_radius=0.064,
        cap_radius=0.052,
        support_span=0.182,
        burner_metal=burner_metal,
        cap_material=burner_cap,
        trivet_material=trivet_iron,
    )
    _add_burner_visuals(
        hob_surface,
        prefix="burner_back_right",
        x=0.265,
        y=0.125,
        cup_radius=0.062,
        ring_radius=0.045,
        cap_radius=0.036,
        support_span=0.132,
        burner_metal=burner_metal,
        cap_material=burner_cap,
        trivet_material=trivet_iron,
    )
    _add_burner_visuals(
        hob_surface,
        prefix="burner_front_right",
        x=0.285,
        y=-0.115,
        cup_radius=0.067,
        ring_radius=0.048,
        cap_radius=0.040,
        support_span=0.140,
        burner_metal=burner_metal,
        cap_material=burner_cap,
        trivet_material=trivet_iron,
    )

    hob_surface.inertial = Inertial.from_geometry(
        Box((HOB_WIDTH, HOB_DEPTH, 0.082)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    model.articulation(
        "countertop_to_hob_surface",
        ArticulationType.FIXED,
        parent=countertop,
        child=hob_surface,
        origin=Origin(),
    )

    _add_knob(
        model,
        hob_surface,
        part_name="left_large_knob",
        joint_name="left_large_knob_spin",
        center_x=-0.340,
        radius=LARGE_KNOB_RADIUS,
        depth=LARGE_KNOB_DEPTH,
        knob_material=knob_stainless,
        indicator_material=knob_mark,
    )
    _add_knob(
        model,
        hob_surface,
        part_name="center_left_knob",
        joint_name="center_left_knob_spin",
        center_x=-0.120,
        radius=MEDIUM_KNOB_RADIUS,
        depth=MEDIUM_KNOB_DEPTH,
        knob_material=knob_stainless,
        indicator_material=knob_mark,
    )
    _add_knob(
        model,
        hob_surface,
        part_name="center_knob",
        joint_name="center_knob_spin",
        center_x=0.0,
        radius=MEDIUM_KNOB_RADIUS,
        depth=MEDIUM_KNOB_DEPTH,
        knob_material=knob_stainless,
        indicator_material=knob_mark,
    )
    _add_knob(
        model,
        hob_surface,
        part_name="center_right_knob",
        joint_name="center_right_knob_spin",
        center_x=0.120,
        radius=MEDIUM_KNOB_RADIUS,
        depth=MEDIUM_KNOB_DEPTH,
        knob_material=knob_stainless,
        indicator_material=knob_mark,
    )
    _add_knob(
        model,
        hob_surface,
        part_name="right_large_knob",
        joint_name="right_large_knob_spin",
        center_x=0.340,
        radius=LARGE_KNOB_RADIUS,
        depth=LARGE_KNOB_DEPTH,
        knob_material=knob_stainless,
        indicator_material=knob_mark,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    countertop = object_model.get_part("countertop")
    hob_surface = object_model.get_part("hob_surface")

    knob_specs = [
        ("left_large_knob", "left_large_knob_spin", LARGE_KNOB_RADIUS),
        ("center_left_knob", "center_left_knob_spin", MEDIUM_KNOB_RADIUS),
        ("center_knob", "center_knob_spin", MEDIUM_KNOB_RADIUS),
        ("center_right_knob", "center_right_knob_spin", MEDIUM_KNOB_RADIUS),
        ("right_large_knob", "right_large_knob_spin", LARGE_KNOB_RADIUS),
    ]
    knobs = {name: object_model.get_part(name) for name, _, _ in knob_specs}
    knob_joints = {joint_name: object_model.get_articulation(joint_name) for _, joint_name, _ in knob_specs}

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    ctx.fail_if_isolated_parts(max_pose_samples=24, name="moving_pose_no_floating")
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="knob_motion_clearance")

    ctx.expect_gap(
        hob_surface,
        countertop,
        axis="z",
        positive_elem="surface_plate",
        negative_elem="countertop_frame",
        max_gap=0.0005,
        max_penetration=0.0,
        name="hob_plate_flush_on_countertop",
    )
    ctx.expect_contact(
        hob_surface,
        countertop,
        elem_a="surface_plate",
        elem_b="countertop_frame",
        contact_tol=0.0005,
        name="hob_plate_contacts_countertop",
    )

    burner_caps = [
        "burner_back_left_cap",
        "burner_front_left_cap",
        "burner_center_cap",
        "burner_back_right_cap",
        "burner_front_right_cap",
    ]
    for burner_cap_name in burner_caps:
        try:
            hob_surface.get_visual(burner_cap_name)
            ctx.check(f"{burner_cap_name}_present", True)
        except Exception as exc:
            ctx.check(f"{burner_cap_name}_present", False, str(exc))

    center_cap_aabb = ctx.part_element_world_aabb(hob_surface, elem="burner_center_cap")
    side_cap_aabb = ctx.part_element_world_aabb(hob_surface, elem="burner_back_left_cap")
    if center_cap_aabb is not None and side_cap_aabb is not None:
        center_cap_width = center_cap_aabb[1][0] - center_cap_aabb[0][0]
        side_cap_width = side_cap_aabb[1][0] - side_cap_aabb[0][0]
        ctx.check(
            "center_burner_reads_larger",
            center_cap_width > side_cap_width + 0.02,
            f"center burner width={center_cap_width:.3f}, side burner width={side_cap_width:.3f}",
        )

    hob_aabb = ctx.part_world_aabb(hob_surface)
    countertop_aabb = ctx.part_world_aabb(countertop)
    if hob_aabb is not None and countertop_aabb is not None:
        ctx.check(
            "no_hob_body_below_countertop",
            hob_aabb[0][2] >= countertop_aabb[0][2] - 1e-6,
            f"hob minimum z={hob_aabb[0][2]:.4f} drops below countertop minimum z={countertop_aabb[0][2]:.4f}",
        )
    surface_plate_aabb = ctx.part_element_world_aabb(hob_surface, elem="surface_plate")
    if surface_plate_aabb is not None:
        plate_width = surface_plate_aabb[1][0] - surface_plate_aabb[0][0]
        plate_depth = surface_plate_aabb[1][1] - surface_plate_aabb[0][1]
        ctx.check(
            "hob_plate_realistic_size",
            abs(plate_width - HOB_WIDTH) < 0.020 and abs(plate_depth - HOB_DEPTH) < 0.020,
            f"plate size=({plate_width:.3f}, {plate_depth:.3f})",
        )

    knob_positions: list[tuple[str, tuple[float, float, float] | None]] = []
    knob_widths: dict[str, float] = {}
    for part_name, joint_name, expected_radius in knob_specs:
        knob = knobs[part_name]
        joint = knob_joints[joint_name]
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"joint type was {joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name}_front_to_back_axis",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0),
            f"axis was {joint.axis}",
        )
        ctx.check(
            f"{joint_name}_unbounded_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            f"limits were {limits}",
        )
        ctx.expect_gap(
            hob_surface,
            knob,
            axis="y",
            positive_elem="control_apron",
            negative_elem="knob_body",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"{part_name}_seated_on_front_apron",
        )
        ctx.expect_overlap(
            knob,
            hob_surface,
            axes="xz",
            min_overlap=0.018,
            elem_a="knob_body",
            elem_b="control_apron",
            name=f"{part_name}_overlaps_front_apron_footprint",
        )

        with ctx.pose({joint: pi / 2.0}):
            ctx.expect_gap(
                hob_surface,
                knob,
                axis="y",
                positive_elem="control_apron",
                negative_elem="knob_body",
                max_gap=0.0005,
                max_penetration=0.0,
                name=f"{part_name}_quarter_turn_stays_seated",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{part_name}_quarter_turn_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{part_name}_quarter_turn_no_floating")

        knob_positions.append((part_name, ctx.part_world_position(knob)))
        knob_aabb = ctx.part_element_world_aabb(knob, elem="knob_body")
        if knob_aabb is not None:
            knob_widths[part_name] = knob_aabb[1][0] - knob_aabb[0][0]

    valid_positions = [item for item in knob_positions if item[1] is not None]
    if len(valid_positions) == 5:
        xs = [position[0] for _, position in valid_positions]
        back_faces: list[float] = []
        for part_name, _, _ in knob_specs:
            knob_aabb = ctx.part_element_world_aabb(knobs[part_name], elem="knob_body")
            if knob_aabb is not None:
                back_faces.append(knob_aabb[1][1])
        ctx.check(
            "five_knobs_spread_across_front_edge",
            xs == sorted(xs) and len(back_faces) == 5 and max(back_faces) - min(back_faces) < 0.001,
            f"knob x positions={xs}, knob back faces={back_faces}",
        )
        ctx.check(
            "center_knob_in_middle",
            abs(valid_positions[2][1][0]) < 0.01,
            f"center knob x={valid_positions[2][1][0]:.3f}",
        )

    if {"left_large_knob", "center_knob", "right_large_knob"} <= knob_widths.keys():
        ctx.check(
            "end_knobs_larger_than_center_knobs",
            knob_widths["left_large_knob"] > knob_widths["center_knob"] + 0.008
            and knob_widths["right_large_knob"] > knob_widths["center_knob"] + 0.008,
            (
                "knob widths "
                f"left={knob_widths['left_large_knob']:.3f}, "
                f"center={knob_widths['center_knob']:.3f}, "
                f"right={knob_widths['right_large_knob']:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
