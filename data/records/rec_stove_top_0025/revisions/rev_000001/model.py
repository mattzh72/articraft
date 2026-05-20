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
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)

SLAB_WIDTH = 0.78
SLAB_DEPTH = 0.54
SLAB_THICKNESS = 0.024
SLAB_CORNER_RADIUS = 0.018

BUTTON_STEM_RADIUS = 0.0040
BUTTON_HOLE_RADIUS = 0.0047
BUTTON_CAP_RADIUS = 0.0078
BUTTON_CAP_HEIGHT = 0.0045
BUTTON_FLANGE_RADIUS = 0.0068
BUTTON_FLANGE_HEIGHT = 0.0040
BUTTON_TRAVEL = 0.0016


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _translated_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _trivet_loop_mesh(name: str, side: float):
    half = side * 0.5
    loop_points = [
        (-half, -half, 0.032),
        (half, -half, 0.032),
        (half, half, 0.032),
        (-half, half, 0.032),
    ]
    return _save_mesh(
        name,
        wire_from_points(
            loop_points,
            radius=0.0040,
            radial_segments=18,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=side * 0.12,
            corner_segments=10,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_cooktop", assets=ASSETS)

    slab_stone = model.material("slab_stone", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_iron = model.material("burner_iron", rgba=(0.12, 0.12, 0.13, 1.0))
    burner_alloy = model.material("burner_alloy", rgba=(0.44, 0.39, 0.30, 1.0))
    button_metal = model.material("button_metal", rgba=(0.73, 0.74, 0.76, 1.0))
    button_shaft = model.material("button_shaft", rgba=(0.30, 0.31, 0.33, 1.0))

    burner_specs = [
        {
            "name": "front_left_burner",
            "x": -0.145,
            "y": -0.050,
            "hole_radius": 0.031,
            "sleeve_radius": 0.028,
            "head_radius": 0.046,
            "cap_radius": 0.024,
            "trivet_side": 0.126,
            "mass": 0.95,
        },
        {
            "name": "front_right_burner",
            "x": 0.145,
            "y": -0.050,
            "hole_radius": 0.028,
            "sleeve_radius": 0.025,
            "head_radius": 0.040,
            "cap_radius": 0.021,
            "trivet_side": 0.106,
            "mass": 0.78,
        },
        {
            "name": "rear_left_burner",
            "x": -0.145,
            "y": 0.110,
            "hole_radius": 0.029,
            "sleeve_radius": 0.026,
            "head_radius": 0.042,
            "cap_radius": 0.022,
            "trivet_side": 0.112,
            "mass": 0.82,
        },
        {
            "name": "rear_right_burner",
            "x": 0.145,
            "y": 0.110,
            "hole_radius": 0.029,
            "sleeve_radius": 0.026,
            "head_radius": 0.042,
            "cap_radius": 0.022,
            "trivet_side": 0.112,
            "mass": 0.82,
        },
    ]
    button_specs = [
        ("button_1", -0.180, -0.210),
        ("button_2", -0.090, -0.201),
        ("button_3", 0.000, -0.192),
        ("button_4", 0.090, -0.201),
        ("button_5", 0.180, -0.210),
    ]

    slab = model.part("counter_slab")
    slab_holes = [
        _translated_profile(_circle_profile(spec["hole_radius"]), spec["x"], spec["y"])
        for spec in burner_specs
    ]
    slab_holes.extend(
        _translated_profile(_circle_profile(BUTTON_HOLE_RADIUS, segments=22), x, y)
        for _, x, y in button_specs
    )
    slab_mesh = _save_mesh(
        "counter_slab.obj",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(
                SLAB_WIDTH,
                SLAB_DEPTH,
                SLAB_CORNER_RADIUS,
                corner_segments=8,
            ),
            slab_holes,
            height=SLAB_THICKNESS,
            center=False,
        ),
    )
    slab.visual(slab_mesh, material=slab_stone, name="slab_shell")
    slab.inertial = Inertial.from_geometry(
        Box((SLAB_WIDTH, SLAB_DEPTH, SLAB_THICKNESS)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, SLAB_THICKNESS * 0.5)),
    )

    trivet_mesh_cache: dict[str, object] = {}

    def get_trivet_mesh(side: float):
        cache_key = f"{side:.3f}"
        trivet_mesh = trivet_mesh_cache.get(cache_key)
        if trivet_mesh is None:
            trivet_mesh = _trivet_loop_mesh(f"trivet_loop_{cache_key}.obj", side)
            trivet_mesh_cache[cache_key] = trivet_mesh
        return trivet_mesh

    for spec in burner_specs:
        burner = model.part(spec["name"])
        burner.visual(
            Cylinder(
                radius=spec["sleeve_radius"],
                length=SLAB_THICKNESS + 0.008,
            ),
            origin=Origin(
                xyz=(0.0, 0.0, (0.008 - SLAB_THICKNESS) * 0.5),
            ),
            material=burner_iron,
            name="burner_sleeve",
        )
        burner.visual(
            Cylinder(radius=spec["hole_radius"] + 0.010, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -SLAB_THICKNESS - 0.002)),
            material=burner_iron,
            name="burner_retainer",
        )
        burner.visual(
            Cylinder(radius=spec["head_radius"], length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=burner_alloy,
            name="burner_head",
        )
        burner.visual(
            Cylinder(radius=spec["cap_radius"], length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.023)),
            material=burner_iron,
            name="burner_cap",
        )
        burner.visual(
            Box((spec["trivet_side"], 0.010, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.032)),
            material=burner_iron,
            name="trivet_cross_x",
        )
        burner.visual(
            Box((0.010, spec["trivet_side"], 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.032)),
            material=burner_iron,
            name="trivet_cross_y",
        )
        burner.visual(
            get_trivet_mesh(spec["trivet_side"]),
            material=burner_iron,
            name="trivet_loop",
        )
        burner.inertial = Inertial.from_geometry(
            Box(
                (
                    spec["trivet_side"],
                    spec["trivet_side"],
                    SLAB_THICKNESS + 0.040,
                )
            ),
            mass=spec["mass"],
            origin=Origin(
                xyz=(0.0, 0.0, (0.040 - SLAB_THICKNESS) * 0.5),
            ),
        )
        model.articulation(
            f"{spec['name']}_mount",
            ArticulationType.FIXED,
            parent=slab,
            child=burner,
            origin=Origin(xyz=(spec["x"], spec["y"], SLAB_THICKNESS)),
        )

    for button_name, x_pos, y_pos in button_specs:
        button = model.part(button_name)
        button.visual(
            Cylinder(
                radius=BUTTON_STEM_RADIUS,
                length=SLAB_THICKNESS + BUTTON_TRAVEL,
            ),
            origin=Origin(
                xyz=(0.0, 0.0, (BUTTON_TRAVEL - SLAB_THICKNESS) * 0.5),
            ),
            material=button_shaft,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=BUTTON_FLANGE_RADIUS, length=BUTTON_FLANGE_HEIGHT),
            origin=Origin(
                xyz=(0.0, 0.0, -SLAB_THICKNESS - (BUTTON_FLANGE_HEIGHT * 0.5)),
            ),
            material=button_shaft,
            name="button_retainer",
        )
        button.visual(
            Cylinder(radius=BUTTON_CAP_RADIUS, length=BUTTON_CAP_HEIGHT),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    BUTTON_TRAVEL + (BUTTON_CAP_HEIGHT * 0.5),
                )
            ),
            material=button_metal,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box(
                (
                    BUTTON_CAP_RADIUS * 2.0,
                    BUTTON_CAP_RADIUS * 2.0,
                    SLAB_THICKNESS + BUTTON_CAP_HEIGHT + BUTTON_FLANGE_HEIGHT,
                )
            ),
            mass=0.035,
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    (BUTTON_CAP_HEIGHT - SLAB_THICKNESS - BUTTON_FLANGE_HEIGHT) * 0.5,
                )
            ),
        )
        model.articulation(
            f"{button_name}_plunger",
            ArticulationType.PRISMATIC,
            parent=slab,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, SLAB_THICKNESS)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    slab = object_model.get_part("counter_slab")
    front_left_burner = object_model.get_part("front_left_burner")
    front_right_burner = object_model.get_part("front_right_burner")
    rear_left_burner = object_model.get_part("rear_left_burner")
    rear_right_burner = object_model.get_part("rear_right_burner")
    burners = [
        front_left_burner,
        front_right_burner,
        rear_left_burner,
        rear_right_burner,
    ]
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]
    button_joints = [
        object_model.get_articulation(f"button_{index}_plunger") for index in range(1, 6)
    ]

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    for part in [slab, *burners, *buttons]:
        ctx.check(f"{part.name}_exists", part is not None, f"missing part: {part.name}")

    slab_aabb = ctx.part_world_aabb(slab)
    if slab_aabb is not None:
        slab_thickness = slab_aabb[1][2] - slab_aabb[0][2]
        ctx.check(
            "slab_thickness_realistic",
            0.020 <= slab_thickness <= 0.030,
            f"expected a thin counter slab, got thickness {slab_thickness:.4f} m",
        )

    deepest_under_panel = min(
        ctx.part_world_aabb(part)[0][2] for part in [*burners, *buttons] if ctx.part_world_aabb(part) is not None
    )
    ctx.check(
        "no_deep_base_body",
        deepest_under_panel >= -0.034,
        f"geometry extends too far below the slab: min z = {deepest_under_panel:.4f}",
    )

    for burner in burners:
        ctx.expect_contact(burner, slab, name=f"{burner.name}_mounted_to_slab")

    ctx.expect_origin_distance(
        front_left_burner,
        front_right_burner,
        axes="x",
        min_dist=0.26,
        max_dist=0.32,
        name="front_burner_spacing",
    )
    ctx.expect_origin_distance(
        rear_left_burner,
        rear_right_burner,
        axes="x",
        min_dist=0.26,
        max_dist=0.32,
        name="rear_burner_spacing",
    )
    ctx.expect_origin_distance(
        rear_left_burner,
        front_left_burner,
        axes="y",
        min_dist=0.14,
        max_dist=0.18,
        name="left_column_burner_spacing",
    )
    ctx.expect_origin_distance(
        rear_right_burner,
        front_right_burner,
        axes="y",
        min_dist=0.14,
        max_dist=0.18,
        name="right_column_burner_spacing",
    )

    button_positions = [ctx.part_world_position(button) for button in buttons]
    if all(position is not None for position in button_positions):
        xs = [position[0] for position in button_positions]
        ys = [position[1] for position in button_positions]
        ctx.check(
            "button_x_order",
            xs[0] < xs[1] < xs[2] < xs[3] < xs[4],
            f"button x positions are not ordered left-to-right: {xs}",
        )
        ctx.check(
            "button_shallow_v",
            ys[2] > ys[1] > ys[0] and ys[2] > ys[3] > ys[4],
            f"button y positions do not form a shallow V: {ys}",
        )
        ctx.check(
            "button_v_symmetry",
            math.isclose(abs(xs[0]), abs(xs[4]), abs_tol=0.003)
            and math.isclose(abs(xs[1]), abs(xs[3]), abs_tol=0.003)
            and math.isclose(ys[0], ys[4], abs_tol=0.003)
            and math.isclose(ys[1], ys[3], abs_tol=0.003),
            f"button layout is not symmetric enough: xs={xs}, ys={ys}",
        )

    for button, joint in zip(buttons, button_joints):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis_is_panel_normal",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, -1.0),
            f"expected button axis (0, 0, -1), got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_travel_is_short",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and math.isclose(limits.lower, 0.0, abs_tol=1e-9)
            and 0.0010 <= limits.upper <= 0.0025,
            f"unexpected button travel limits: {limits}",
        )
        ctx.expect_contact(button, slab, name=f"{button.name}_rest_contact")
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.expect_contact(button, slab, name=f"{button.name}_pressed_contact")
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

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
