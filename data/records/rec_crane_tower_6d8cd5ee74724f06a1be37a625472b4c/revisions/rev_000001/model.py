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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _square_loop(size: float) -> list[tuple[float, float]]:
    half = size * 0.5
    return [(-half, -half), (half, -half), (half, half), (-half, half)]


def _square_hole_loop(size: float) -> list[tuple[float, float]]:
    half = size * 0.5
    return [(-half, -half), (-half, half), (half, half), (half, -half)]


def _square_tube_mesh(outer_size: float, inner_size: float, height: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _square_loop(outer_size),
            [_square_hole_loop(inner_size)],
            height,
        ),
        name,
    )


def _add_triangular_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    bottom_z: float,
    half_width: float,
    root_top_z: float,
    tip_top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> dict[str, list[tuple[float, float, float]]]:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    span = x_end - x_start

    def top_z(x: float) -> float:
        if abs(span) < 1e-9:
            return root_top_z
        t = (x - x_start) / span
        return root_top_z + (tip_top_z - root_top_z) * t

    lower_left = [(x, -half_width, bottom_z) for x in xs]
    lower_right = [(x, half_width, bottom_z) for x in xs]
    upper = [(x, 0.0, top_z(x)) for x in xs]

    for i in range(panels):
        _add_member(part, lower_left[i], lower_left[i + 1], chord_radius, material)
        _add_member(part, lower_right[i], lower_right[i + 1], chord_radius, material)
        _add_member(part, upper[i], upper[i + 1], chord_radius, material)

    for i in range(panels + 1):
        _add_member(part, lower_left[i], lower_right[i], brace_radius, material)
        _add_member(part, lower_left[i], upper[i], brace_radius, material)
        _add_member(part, lower_right[i], upper[i], brace_radius, material)

    for i in range(panels):
        if i % 2 == 0:
            _add_member(part, lower_left[i], upper[i + 1], brace_radius, material)
            _add_member(part, lower_right[i], upper[i + 1], brace_radius, material)
        else:
            _add_member(part, upper[i], lower_left[i + 1], brace_radius, material)
            _add_member(part, upper[i], lower_right[i + 1], brace_radius, material)

    return {"lower_left": lower_left, "lower_right": lower_right, "upper": upper}


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.08, 0.0, -0.08),
            (0.10, 0.0, -0.18),
            (0.05, 0.0, -0.28),
            (-0.02, 0.0, -0.34),
            (-0.08, 0.0, -0.28),
            (-0.06, 0.0, -0.16),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "self_erecting_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_erecting_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.92, 0.77, 0.16, 1.0))
    chassis_grey = model.material("chassis_grey", rgba=(0.24, 0.25, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    ballast = model.material("ballast", rgba=(0.47, 0.47, 0.45, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.15, 1.0))
    hook_red = model.material("hook_red", rgba=(0.73, 0.13, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.60, 0.78, 0.86, 0.35))

    hook_mesh = _build_hook_mesh()

    mast_x = -0.25
    deck_top_z = 0.71
    sleeve_height = 3.40
    sleeve_top_z = deck_top_z + sleeve_height

    ground_base = model.part("ground_base")
    ground_base.visual(
        Box((3.60, 0.54, 0.20)),
        origin=Origin(xyz=(0.0, -0.67, 0.61)),
        material=chassis_grey,
        name="left_deck",
    )
    ground_base.visual(
        Box((3.60, 0.54, 0.20)),
        origin=Origin(xyz=(0.0, 0.67, 0.61)),
        material=chassis_grey,
        name="right_deck",
    )
    ground_base.visual(
        Box((3.35, 0.20, 0.24)),
        origin=Origin(xyz=(0.02, -0.74, 0.50)),
        material=chassis_grey,
        name="left_frame_rail",
    )
    ground_base.visual(
        Box((3.35, 0.20, 0.24)),
        origin=Origin(xyz=(0.02, 0.74, 0.50)),
        material=chassis_grey,
        name="right_frame_rail",
    )
    ground_base.visual(
        Box((0.95, 1.55, 0.26)),
        origin=Origin(xyz=(-1.23, 0.0, 0.84)),
        material=ballast,
        name="rear_ballast",
    )
    ground_base.visual(
        Box((0.70, 0.60, 0.42)),
        origin=Origin(xyz=(0.95, -0.45, 0.92)),
        material=chassis_grey,
        name="power_pack",
    )

    for x in (-1.15, 1.15):
        for y in (-1.15, 1.15):
            post_name = f"outrigger_post_{'f' if x > 0 else 'r'}_{'r' if y > 0 else 'l'}"
            pad_name = f"outrigger_pad_{'f' if x > 0 else 'r'}_{'r' if y > 0 else 'l'}"
            beam_name = f"outrigger_beam_{'f' if x > 0 else 'r'}_{'r' if y > 0 else 'l'}"
            ground_base.visual(
                Box((0.12, 0.12, 0.46)),
                origin=Origin(xyz=(x, y, 0.28)),
                material=steel,
                name=post_name,
            )
            ground_base.visual(
                Box((0.24, 0.24, 0.10)),
                origin=Origin(xyz=(x, y, 0.05)),
                material=steel,
                name=pad_name,
            )
            ground_base.visual(
                Box((0.22, 0.60, 0.10)),
                origin=Origin(xyz=(x, y * 0.83, 0.51)),
                material=steel,
                name=beam_name,
            )

    for axle_x in (-0.82, 0.95):
        axle_tag = "rear" if axle_x < 0.0 else "front"
        ground_base.visual(
            Box((0.10, 1.84, 0.08)),
            origin=Origin(xyz=(axle_x, 0.0, 0.30)),
            material=steel,
            name=f"{axle_tag}_axle",
        )
        for hang_y in (-0.55, 0.55):
            ground_base.visual(
                Box((0.10, 0.12, 0.22)),
                origin=Origin(xyz=(axle_x, hang_y, 0.43)),
                material=steel,
                name=f"{axle_tag}_hanger_{'r' if hang_y > 0 else 'l'}",
            )
        for wheel_y in (-1.00, 1.00):
            side_tag = "right" if wheel_y > 0.0 else "left"
            ground_base.visual(
                Cylinder(radius=0.24, length=0.16),
                origin=Origin(
                    xyz=(axle_x, wheel_y, 0.24),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=rubber,
                name=f"{axle_tag}_{side_tag}_wheel_tire",
            )
            ground_base.visual(
                Cylinder(radius=0.10, length=0.18),
                origin=Origin(
                    xyz=(axle_x, wheel_y, 0.24),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
                name=f"{axle_tag}_{side_tag}_wheel_hub",
            )

    _add_member(ground_base, (1.65, -0.42, 0.61), (2.55, 0.0, 0.42), 0.05, steel)
    _add_member(ground_base, (1.65, 0.42, 0.61), (2.55, 0.0, 0.42), 0.05, steel)
    ground_base.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(2.66, 0.0, 0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hitch_eye",
    )
    ground_base.visual(
        Box((0.28, 0.12, 0.10)),
        origin=Origin(xyz=(2.51, 0.0, 0.42)),
        material=steel,
        name="hitch_drawbar",
    )

    ground_base.visual(
        _square_tube_mesh(0.70, 0.56, sleeve_height, "mast_sleeve_mesh"),
        origin=Origin(xyz=(mast_x, 0.0, deck_top_z + sleeve_height * 0.5)),
        material=crane_yellow,
        name="mast_sleeve",
    )
    ground_base.visual(
        _square_tube_mesh(0.80, 0.58, 0.16, "mast_bottom_collar_mesh"),
        origin=Origin(xyz=(mast_x, 0.0, deck_top_z + 0.08)),
        material=chassis_grey,
        name="mast_bottom_collar",
    )
    ground_base.visual(
        _square_tube_mesh(0.80, 0.58, 0.18, "mast_top_collar_mesh"),
        origin=Origin(xyz=(mast_x, 0.0, sleeve_top_z - 0.09)),
        material=chassis_grey,
        name="mast_top_collar",
    )
    ground_base.visual(
        _square_tube_mesh(1.26, 0.68, 0.10, "mast_base_ring_mesh"),
        origin=Origin(xyz=(mast_x, 0.0, deck_top_z + 0.05)),
        material=steel,
        name="mast_base_ring",
    )

    ground_base.inertial = Inertial.from_geometry(
        Box((5.40, 2.50, 4.20)),
        mass=6400.0,
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        _square_tube_mesh(0.50, 0.40, 5.80, "inner_mast_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.70)),
        material=crane_yellow,
        name="inner_mast_tube",
    )
    inner_mast.visual(
        Box((0.62, 0.62, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 2.27)),
        material=chassis_grey,
        name="mast_head_box",
    )
    inner_mast.visual(
        Cylinder(radius=0.34, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.39)),
        material=steel,
        name="mast_head_bearing",
    )
    inner_mast.inertial = Inertial.from_geometry(
        Box((0.68, 0.68, 5.90)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, -0.50)),
    )

    rotating_jib = model.part("rotating_jib")
    rotating_jib.visual(
        Cylinder(radius=0.34, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=steel,
        name="slew_bearing_lower",
    )
    rotating_jib.visual(
        Box((0.42, 0.42, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=chassis_grey,
        name="slew_pedestal",
    )
    rotating_jib.visual(
        Box((1.30, 0.72, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, 0.20)),
        material=chassis_grey,
        name="machinery_platform",
    )
    rotating_jib.visual(
        Box((0.45, 0.30, 0.30)),
        origin=Origin(xyz=(0.58, -0.34, 0.32)),
        material=chassis_grey,
        name="operator_cab_shell",
    )
    rotating_jib.visual(
        Box((0.40, 0.24, 0.24)),
        origin=Origin(xyz=(0.60, -0.35, 0.33)),
        material=glass,
        name="operator_cab_glazing",
    )

    front_jib = _add_triangular_truss(
        rotating_jib,
        x_start=0.70,
        x_end=6.25,
        bottom_z=0.24,
        half_width=0.16,
        root_top_z=1.05,
        tip_top_z=0.60,
        panels=8,
        chord_radius=0.022,
        brace_radius=0.012,
        material=crane_yellow,
    )
    rear_jib = _add_triangular_truss(
        rotating_jib,
        x_start=-0.55,
        x_end=-1.95,
        bottom_z=0.24,
        half_width=0.14,
        root_top_z=0.95,
        tip_top_z=0.48,
        panels=4,
        chord_radius=0.020,
        brace_radius=0.011,
        material=crane_yellow,
    )

    for idx, tie_x in enumerate((0.95, 1.75, 2.55, 3.35, 4.15, 4.95, 5.75)):
        rotating_jib.visual(
            Box((0.10, 0.36, 0.044)),
            origin=Origin(xyz=(tie_x, 0.0, 0.196)),
            material=steel,
            name=f"rail_tie_{idx}",
        )
    rotating_jib.visual(
        Box((5.00, 0.18, 0.012)),
        origin=Origin(xyz=(3.30, 0.0, 0.224)),
        material=steel,
        name="front_walkway",
    )
    rotating_jib.visual(
        Box((5.00, 0.05, 0.05)),
        origin=Origin(xyz=(3.30, -0.14, 0.243)),
        material=steel,
        name="lower_rail_left",
    )
    rotating_jib.visual(
        Box((5.00, 0.05, 0.05)),
        origin=Origin(xyz=(3.30, 0.14, 0.243)),
        material=steel,
        name="lower_rail_right",
    )
    rotating_jib.visual(
        Box((0.12, 0.34, 0.38)),
        origin=Origin(xyz=(6.25, 0.0, 0.43)),
        material=crane_yellow,
        name="front_tip_cap",
    )
    rotating_jib.visual(
        Box((0.70, 0.62, 0.26)),
        origin=Origin(xyz=(-1.48, 0.0, 0.34)),
        material=ballast,
        name="counterweight_pack",
    )
    rotating_jib.visual(
        Box((0.42, 0.28, 0.26)),
        origin=Origin(xyz=(-0.62, 0.0, 0.34)),
        material=chassis_grey,
        name="winch_house",
    )

    apex = (0.20, 0.0, 1.50)
    _add_member(rotating_jib, (0.08, -0.20, 0.25), apex, 0.022, crane_yellow)
    _add_member(rotating_jib, (0.08, 0.20, 0.25), apex, 0.022, crane_yellow)
    rotating_jib.visual(
        Box((0.14, 0.12, 0.12)),
        origin=Origin(xyz=apex),
        material=crane_yellow,
        name="apex_head",
    )
    _add_member(rotating_jib, apex, front_jib["upper"][5], 0.010, cable)
    _add_member(rotating_jib, apex, front_jib["upper"][-1], 0.010, cable)
    _add_member(rotating_jib, apex, rear_jib["upper"][-1], 0.010, cable)

    rotating_jib.inertial = Inertial.from_geometry(
        Box((8.60, 0.90, 1.70)),
        mass=2100.0,
        origin=Origin(xyz=(2.10, 0.0, 0.70)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.36, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=chassis_grey,
        name="carriage_frame",
    )
    trolley.visual(
        Box((0.32, 0.12, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=steel,
        name="top_spreader",
    )
    trolley.visual(
        Box((0.32, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.125, -0.005)),
        material=steel,
        name="left_wheel_axle_bar",
    )
    trolley.visual(
        Box((0.32, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.125, -0.005)),
        material=steel,
        name="right_wheel_axle_bar",
    )
    trolley.visual(
        Box((0.26, 0.06, 0.18)),
        origin=Origin(xyz=(0.0, -0.095, -0.09)),
        material=steel,
        name="left_hanger",
    )
    trolley.visual(
        Box((0.26, 0.06, 0.18)),
        origin=Origin(xyz=(0.0, 0.095, -0.09)),
        material=steel,
        name="right_hanger",
    )
    trolley.visual(
        Box((0.18, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=crane_yellow,
        name="motor_house",
    )
    for wheel_x in (-0.12, 0.12):
        for wheel_y in (-0.145, 0.145):
            trolley.visual(
                Cylinder(radius=0.035, length=0.05),
                origin=Origin(
                    xyz=(wheel_x, wheel_y, 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
                name=f"wheel_{'front' if wheel_x > 0.0 else 'rear'}_{'r' if wheel_y > 0.0 else 'l'}",
            )

    trolley.visual(
        Box((0.10, 0.08, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
        material=steel,
        name="hoist_stem",
    )
    trolley.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.35)),
        material=crane_yellow,
        name="hoist_head",
    )
    trolley.visual(
        Cylinder(radius=0.006, length=0.70),
        origin=Origin(xyz=(0.0, -0.05, -0.73)),
        material=cable,
        name="left_hoist_cable",
    )
    trolley.visual(
        Cylinder(radius=0.006, length=0.70),
        origin=Origin(xyz=(0.0, 0.05, -0.73)),
        material=cable,
        name="right_hoist_cable",
    )
    trolley.visual(
        Cylinder(radius=0.015, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -1.08), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="equalizer_bar",
    )
    trolley.visual(
        Box((0.18, 0.12, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -1.18)),
        material=crane_yellow,
        name="hook_block_upper",
    )
    trolley.visual(
        Box((0.14, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -1.37)),
        material=crane_yellow,
        name="hook_block_lower",
    )
    trolley.visual(
        Cylinder(radius=0.025, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -1.53)),
        material=steel,
        name="hook_shank",
    )
    trolley.visual(
        hook_mesh,
        origin=Origin(xyz=(0.0, 0.0, -1.60)),
        material=hook_red,
        name="hook",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.52, 0.50, 2.10)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, -0.84)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=ground_base,
        child=inner_mast,
        origin=Origin(xyz=(mast_x, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120000.0,
            velocity=0.18,
            lower=0.0,
            upper=2.80,
        ),
    )
    model.articulation(
        "jib_slew",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=rotating_jib,
        origin=Origin(xyz=(0.0, 0.0, 2.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90000.0,
            velocity=0.35,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=rotating_jib,
        child=trolley,
        origin=Origin(xyz=(1.35, 0.0, 0.183)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15000.0,
            velocity=0.70,
            lower=0.0,
            upper=4.60,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_base = object_model.get_part("ground_base")
    inner_mast = object_model.get_part("inner_mast")
    rotating_jib = object_model.get_part("rotating_jib")
    trolley = object_model.get_part("trolley")
    mast_extension = object_model.get_articulation("mast_extension")
    jib_slew = object_model.get_articulation("jib_slew")
    trolley_travel = object_model.get_articulation("trolley_travel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        inner_mast,
        reason="The telescoping mast is intentionally modeled with running clearance inside the square sleeve, and support is proven by retained insertion checks.",
    )
    ctx.allow_isolated_part(
        rotating_jib,
        reason="The slewing upperworks are carried by the clearanced telescoping mast assembly rather than rest-pose solid contact.",
    )
    ctx.allow_isolated_part(
        trolley,
        reason="The trolley is supported by the rail-guided jib assembly, which is modeled with running clearances and exact rail contact checks.",
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

    mast_upper = mast_extension.motion_limits.upper or 0.0
    slew_test_angle = math.pi / 2.0
    trolley_upper = trolley_travel.motion_limits.upper or 0.0

    ctx.expect_within(
        inner_mast,
        ground_base,
        axes="xy",
        inner_elem="inner_mast_tube",
        outer_elem="mast_sleeve",
        margin=0.0,
        name="inner mast stays centered in the base sleeve at rest",
    )
    ctx.expect_overlap(
        inner_mast,
        ground_base,
        axes="z",
        elem_a="inner_mast_tube",
        elem_b="mast_sleeve",
        min_overlap=3.0,
        name="inner mast has deep retained insertion at rest",
    )

    rest_mast_pos = ctx.part_world_position(inner_mast)
    with ctx.pose({mast_extension: mast_upper}):
        extended_mast_pos = ctx.part_world_position(inner_mast)
        ctx.expect_within(
            inner_mast,
            ground_base,
            axes="xy",
            inner_elem="inner_mast_tube",
            outer_elem="mast_sleeve",
            margin=0.0,
            name="extended mast stays centered in the base sleeve",
        )
        ctx.expect_overlap(
            inner_mast,
            ground_base,
            axes="z",
            elem_a="inner_mast_tube",
            elem_b="mast_sleeve",
            min_overlap=0.75,
            name="extended mast retains insertion in the base sleeve",
        )
    ctx.check(
        "mast extends upward from the wheeled base",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 2.5,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    mast_axis_pos = ctx.part_world_position(inner_mast)
    rest_tip = _aabb_center(ctx.part_element_world_aabb(rotating_jib, elem="front_tip_cap"))
    with ctx.pose({jib_slew: slew_test_angle}):
        swung_tip = _aabb_center(ctx.part_element_world_aabb(rotating_jib, elem="front_tip_cap"))
    ctx.check(
        "jib slews about the mast-top vertical axis",
        mast_axis_pos is not None
        and rest_tip is not None
        and swung_tip is not None
        and abs(rest_tip[1] - mast_axis_pos[1]) < 0.2
        and swung_tip[1] > mast_axis_pos[1] + 4.5
        and abs(swung_tip[0] - mast_axis_pos[0]) < 1.0,
        details=f"mast={mast_axis_pos}, rest_tip={rest_tip}, swung_tip={swung_tip}",
    )

    with ctx.pose({trolley_travel: 0.0, jib_slew: 0.0}):
        rest_trolley_pos = ctx.part_world_position(trolley)
        ctx.expect_gap(
            rotating_jib,
            trolley,
            axis="z",
            positive_elem="lower_rail_left",
            negative_elem="wheel_rear_l",
            min_gap=0.0,
            max_gap=0.002,
            name="left trolley wheels sit on the left jib rail",
        )
        ctx.expect_gap(
            rotating_jib,
            trolley,
            axis="z",
            positive_elem="lower_rail_right",
            negative_elem="wheel_rear_r",
            min_gap=0.0,
            max_gap=0.002,
            name="right trolley wheels sit on the right jib rail",
        )
    with ctx.pose({trolley_travel: trolley_upper, jib_slew: 0.0}):
        outboard_trolley_pos = ctx.part_world_position(trolley)
        ctx.expect_origin_gap(
            trolley,
            inner_mast,
            axis="x",
            min_gap=5.0,
            name="trolley can reach far out along the jib",
        )
    ctx.check(
        "trolley travels outward along the jib",
        rest_trolley_pos is not None
        and outboard_trolley_pos is not None
        and outboard_trolley_pos[0] > rest_trolley_pos[0] + 4.2
        and abs(outboard_trolley_pos[1] - rest_trolley_pos[1]) < 0.05,
        details=f"rest={rest_trolley_pos}, outboard={outboard_trolley_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
