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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_speed_urban_crankset")

    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    arm_black = model.material("arm_black", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    pedal_silver = model.material("pedal_silver", rgba=(0.63, 0.65, 0.67, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos((2.0 * math.pi * i) / segments),
                radius * math.sin((2.0 * math.pi * i) / segments),
            )
            for i in range(segments)
        ]

    def _chainring_profile(
        tooth_count: int,
        *,
        tip_radius: float,
        root_radius: float,
    ) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        tooth_angle = (2.0 * math.pi) / tooth_count
        mid_radius = root_radius + (tip_radius - root_radius) * 0.58
        for index in range(tooth_count):
            angle = index * tooth_angle
            for radius, offset in (
                (root_radius, -0.34),
                (mid_radius, -0.14),
                (tip_radius, 0.0),
                (mid_radius, 0.14),
                (root_radius, 0.34),
            ):
                sample_angle = angle + offset * tooth_angle
                points.append((radius * math.cos(sample_angle), radius * math.sin(sample_angle)))
        return points

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

    def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
        part.visual(
            Cylinder(radius=radius, length=_distance(a, b)),
            origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    def _arm_section(
        x: float,
        *,
        y_center: float,
        width: float,
        thickness: float,
        corner: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y_center + y, z)
            for y, z in rounded_rect_profile(width, thickness, radius=corner, corner_segments=6)
        ]

    bb_shell = model.part("bottom_bracket_shell")
    shell_outer = [
        (0.0205, -0.034),
        (0.0215, -0.031),
        (0.0215, -0.026),
        (0.0200, -0.022),
        (0.0200, 0.022),
        (0.0215, 0.026),
        (0.0215, 0.031),
        (0.0205, 0.034),
    ]
    shell_inner = [
        (0.0115, -0.034),
        (0.0115, 0.034),
    ]
    shell_mesh = _save_mesh(
        "bb_shell",
        LatheGeometry.from_shell_profiles(
            shell_outer,
            shell_inner,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    bb_shell.visual(
        shell_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_black,
        name="shell_tube",
    )
    bb_shell.visual(
        Cylinder(radius=0.0235, length=0.003),
        origin=Origin(xyz=(0.0, -0.0335, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_cup",
    )
    bb_shell.visual(
        Cylinder(radius=0.0235, length=0.003),
        origin=Origin(xyz=(0.0, 0.0335, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_cup",
    )
    bb_shell.inertial = Inertial.from_geometry(
        Box((0.050, 0.080, 0.050)),
        mass=0.7,
        origin=Origin(),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.009, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="spindle_axle",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0095, length=0.078),
        mass=0.35,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_crank = model.part("right_crank")
    right_arm_geom = section_loft(
        [
            _arm_section(0.020, y_center=-0.048, width=0.022, thickness=0.014, corner=0.0040),
            _arm_section(0.060, y_center=-0.049, width=0.020, thickness=0.013, corner=0.0035),
            _arm_section(0.120, y_center=-0.053, width=0.021, thickness=0.012, corner=0.0030),
            _arm_section(0.170, y_center=-0.056, width=0.030, thickness=0.014, corner=0.0045),
        ]
    )
    right_crank.visual(
        _save_mesh("right_crank_arm", right_arm_geom),
        material=arm_black,
        name="right_arm_blade",
    )
    right_crank.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_black,
        name="right_hub",
    )
    right_crank.visual(
        Box((0.014, 0.014, 0.012)),
        origin=Origin(xyz=(0.013, -0.048, 0.0)),
        material=arm_black,
        name="right_hub_bridge",
    )
    right_crank.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.170, -0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_black,
        name="right_pedal_boss",
    )
    chainring_mesh = _save_mesh(
        "single_chainring",
        ExtrudeWithHolesGeometry(
            _chainring_profile(
                46,
                tip_radius=0.097,
                root_radius=0.091,
            ),
            [_circle_profile(0.074, segments=84)],
            height=0.004,
            center=True,
        ).rotate_x(-math.pi / 2.0),
    )
    right_crank.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.0, -0.031, 0.0)),
        material=steel,
        name="chainring_ring",
    )
    right_crank.visual(
        Cylinder(radius=0.029, length=0.004),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="chainring_carrier",
    )
    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0 + math.radians(10.0)
        start = (0.029 * math.cos(angle), -0.034, 0.029 * math.sin(angle))
        end = (0.075 * math.cos(angle), -0.031, 0.075 * math.sin(angle))
        _add_member(right_crank, start, end, 0.0042, steel)
    right_crank.inertial = Inertial.from_geometry(
        Box((0.190, 0.090, 0.200)),
        mass=0.75,
        origin=Origin(xyz=(0.085, -0.043, 0.0)),
    )

    left_crank = model.part("left_crank")
    left_arm_geom = section_loft(
        [
            _arm_section(-0.020, y_center=0.048, width=0.022, thickness=0.014, corner=0.0040),
            _arm_section(-0.060, y_center=0.049, width=0.020, thickness=0.013, corner=0.0035),
            _arm_section(-0.120, y_center=0.053, width=0.021, thickness=0.012, corner=0.0030),
            _arm_section(-0.170, y_center=0.056, width=0.030, thickness=0.014, corner=0.0045),
        ]
    )
    left_crank.visual(
        _save_mesh("left_crank_arm", left_arm_geom),
        material=arm_black,
        name="left_arm_blade",
    )
    left_crank.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_black,
        name="left_hub",
    )
    left_crank.visual(
        Box((0.014, 0.014, 0.012)),
        origin=Origin(xyz=(-0.013, 0.048, 0.0)),
        material=arm_black,
        name="left_hub_bridge",
    )
    left_crank.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(-0.170, 0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_black,
        name="left_pedal_boss",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.190, 0.090, 0.050)),
        mass=0.55,
        origin=Origin(xyz=(-0.085, 0.043, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pedal_axle",
    )
    right_pedal.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pedal_inner_boss",
    )
    right_pedal.visual(
        Box((0.092, 0.064, 0.014)),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=pedal_silver,
        name="pedal_platform",
    )
    right_pedal.visual(
        Box((0.088, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.050, 0.009)),
        material=dark_steel,
        name="top_traction_bar",
    )
    right_pedal.visual(
        Box((0.010, 0.060, 0.014)),
        origin=Origin(xyz=(0.041, -0.050, 0.0)),
        material=dark_steel,
        name="front_cage_bar",
    )
    right_pedal.visual(
        Box((0.010, 0.060, 0.014)),
        origin=Origin(xyz=(-0.041, -0.050, 0.0)),
        material=dark_steel,
        name="rear_cage_bar",
    )
    right_pedal.visual(
        Box((0.018, 0.018, 0.004)),
        origin=Origin(xyz=(0.024, -0.072, 0.009)),
        material=steel,
        name="pedal_marker",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.096, 0.086, 0.026)),
        mass=0.20,
        origin=Origin(xyz=(0.0, -0.043, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pedal_axle",
    )
    left_pedal.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pedal_inner_boss",
    )
    left_pedal.visual(
        Box((0.092, 0.064, 0.014)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=pedal_silver,
        name="pedal_platform",
    )
    left_pedal.visual(
        Box((0.088, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.050, 0.009)),
        material=dark_steel,
        name="top_traction_bar",
    )
    left_pedal.visual(
        Box((0.010, 0.060, 0.014)),
        origin=Origin(xyz=(0.041, 0.050, 0.0)),
        material=dark_steel,
        name="front_cage_bar",
    )
    left_pedal.visual(
        Box((0.010, 0.060, 0.014)),
        origin=Origin(xyz=(-0.041, 0.050, 0.0)),
        material=dark_steel,
        name="rear_cage_bar",
    )
    left_pedal.visual(
        Box((0.018, 0.018, 0.004)),
        origin=Origin(xyz=(0.024, 0.072, 0.009)),
        material=steel,
        name="pedal_marker",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.096, 0.086, 0.026)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
    )

    model.articulation(
        "shell_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.170, -0.067, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.170, 0.067, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bottom_bracket_shell")
    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    spindle_spin = object_model.get_articulation("shell_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return (
            (low[0] + high[0]) * 0.5,
            (low[1] + high[1]) * 0.5,
            (low[2] + high[2]) * 0.5,
        )

    ctx.check(
        "spindle articulation is continuous",
        spindle_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spindle_spin.articulation_type}",
    )
    ctx.check(
        "right pedal articulation is continuous",
        right_pedal_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={right_pedal_spin.articulation_type}",
    )
    ctx.check(
        "left pedal articulation is continuous",
        left_pedal_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={left_pedal_spin.articulation_type}",
    )

    with ctx.pose({spindle_spin: 0.0, right_pedal_spin: 0.0, left_pedal_spin: 0.0}):
        ctx.expect_within(
            spindle,
            shell,
            axes="xz",
            margin=0.0,
            inner_elem="spindle_axle",
            outer_elem="shell_tube",
            name="spindle stays centered within shell projection",
        )
        ctx.expect_overlap(
            spindle,
            shell,
            axes="y",
            min_overlap=0.060,
            elem_a="spindle_axle",
            elem_b="shell_tube",
            name="spindle spans the shell width",
        )
        ctx.expect_contact(
            right_crank,
            spindle,
            elem_a="right_hub",
            elem_b="spindle_axle",
            name="right crank hub seats on spindle",
        )
        ctx.expect_contact(
            left_crank,
            spindle,
            elem_a="left_hub",
            elem_b="spindle_axle",
            name="left crank hub seats on spindle",
        )
        ctx.expect_contact(
            right_pedal,
            right_crank,
            elem_a="pedal_axle",
            elem_b="right_pedal_boss",
            name="right pedal axle threads into crank boss",
        )
        ctx.expect_contact(
            left_pedal,
            left_crank,
            elem_a="pedal_axle",
            elem_b="left_pedal_boss",
            name="left pedal axle threads into crank boss",
        )

    rest_right_pos = ctx.part_world_position(right_pedal)
    rest_left_pos = ctx.part_world_position(left_pedal)
    with ctx.pose({spindle_spin: math.pi / 2.0, right_pedal_spin: 0.0, left_pedal_spin: 0.0}):
        turned_right_pos = ctx.part_world_position(right_pedal)
        turned_left_pos = ctx.part_world_position(left_pedal)

    ctx.check(
        "crank rotation raises right pedal",
        rest_right_pos is not None
        and turned_right_pos is not None
        and turned_right_pos[2] > rest_right_pos[2] + 0.12,
        details=f"rest={rest_right_pos}, turned={turned_right_pos}",
    )
    ctx.check(
        "crank rotation lowers left pedal",
        rest_left_pos is not None
        and turned_left_pos is not None
        and turned_left_pos[2] < rest_left_pos[2] - 0.12,
        details=f"rest={rest_left_pos}, turned={turned_left_pos}",
    )

    right_marker_rest = _aabb_center(ctx.part_element_world_aabb(right_pedal, elem="pedal_marker"))
    with ctx.pose({right_pedal_spin: math.pi / 2.0}):
        right_marker_spun = _aabb_center(ctx.part_element_world_aabb(right_pedal, elem="pedal_marker"))
    left_marker_rest = _aabb_center(ctx.part_element_world_aabb(left_pedal, elem="pedal_marker"))
    with ctx.pose({left_pedal_spin: math.pi / 2.0}):
        left_marker_spun = _aabb_center(ctx.part_element_world_aabb(left_pedal, elem="pedal_marker"))

    ctx.check(
        "right pedal spins about axle",
        right_marker_rest is not None
        and right_marker_spun is not None
        and math.dist(right_marker_rest, right_marker_spun) > 0.020,
        details=f"rest={right_marker_rest}, spun={right_marker_spun}",
    )
    ctx.check(
        "left pedal spins about axle",
        left_marker_rest is not None
        and left_marker_spun is not None
        and math.dist(left_marker_rest, left_marker_spun) > 0.020,
        details=f"rest={left_marker_rest}, spun={left_marker_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
