from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, height: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=48),
            [_circle_profile(inner_radius, segments=40)],
            height=height,
            center=True,
        ),
        name,
    )


def _rect_tube_mesh(
    name: str,
    *,
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
):
    outer_half_x = outer_x * 0.5
    outer_half_y = outer_y * 0.5
    inner_half_x = inner_x * 0.5
    inner_half_y = inner_y * 0.5
    wall_x = outer_half_x - inner_half_x
    wall_y = outer_half_y - inner_half_y

    left = BoxGeometry((wall_x, outer_y, height)).translate(
        -(outer_half_x + inner_half_x) * 0.5, 0.0, 0.0
    )
    right = BoxGeometry((wall_x, outer_y, height)).translate(
        (outer_half_x + inner_half_x) * 0.5, 0.0, 0.0
    )
    top = BoxGeometry((outer_x, wall_y, height)).translate(
        0.0, (outer_half_y + inner_half_y) * 0.5, 0.0
    )
    bottom = BoxGeometry((outer_x, wall_y, height)).translate(
        0.0, -(outer_half_y + inner_half_y) * 0.5, 0.0
    )

    return mesh_from_geometry(left.merge(right).merge(top).merge(bottom), name)


def _xz_section(
    width: float,
    height: float,
    corner_radius: float,
    y: float,
    *,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_offset) for x, z in rounded_rect_profile(width, height, corner_radius)]


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


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_pillar_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.25, 0.29, 1.0))
    dark_machine = model.material("dark_machine", rgba=(0.15, 0.17, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    column_y = -0.09
    spindle_y = 0.07
    head_z = 0.69

    frame = model.part("frame")
    frame.visual(
        Box((0.42, 0.32, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.05),
        origin=Origin(xyz=(0.0, column_y, 0.055)),
        material=dark_machine,
        name="column_foot",
    )
    frame.visual(
        Cylinder(radius=0.03, length=0.80),
        origin=Origin(xyz=(0.0, column_y, 0.43)),
        material=polished_steel,
        name="column",
    )
    frame.visual(
        Box((0.10, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, column_y, 0.055)),
        material=dark_machine,
        name="column_pedestal",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.32, 0.83)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
    )

    head = model.part("head")
    head_collar = _annulus_mesh(
        "drill_press_head_collar",
        outer_radius=0.056,
        inner_radius=0.034,
        height=0.12,
    )
    spindle_nose = _rect_tube_mesh(
        "drill_press_spindle_nose",
        outer_x=0.092,
        outer_y=0.092,
        inner_x=0.060,
        inner_y=0.060,
        height=0.16,
    )
    head.visual(head_collar, material=cast_iron, name="head_collar")
    head.visual(
        Box((0.080, 0.18, 0.30)),
        origin=Origin(xyz=(0.075, 0.09, 0.06)),
        material=cast_iron,
        name="head_side_right",
    )
    head.visual(
        Box((0.080, 0.18, 0.30)),
        origin=Origin(xyz=(-0.075, 0.09, 0.06)),
        material=cast_iron,
        name="head_side_left",
    )
    head.visual(
        Box((0.020, 0.10, 0.10)),
        origin=Origin(xyz=(0.045, 0.05, 0.0)),
        material=cast_iron,
        name="head_bridge_right",
    )
    head.visual(
        Box((0.020, 0.10, 0.10)),
        origin=Origin(xyz=(-0.045, 0.05, 0.0)),
        material=cast_iron,
        name="head_bridge_left",
    )
    head.visual(
        Box((0.12, 0.05, 0.14)),
        origin=Origin(xyz=(0.0, 0.055, 0.07)),
        material=cast_iron,
        name="head_rear_bridge",
    )
    head.visual(
        Box((0.18, 0.20, 0.07)),
        origin=Origin(xyz=(0.0, 0.10, 0.205)),
        material=cast_iron,
        name="belt_cover",
    )
    head.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.13, 0.165)),
        material=dark_machine,
        name="motor_housing",
    )
    head.visual(
        Box((0.020, 0.08, 0.12)),
        origin=Origin(xyz=(0.045, spindle_y - column_y, 0.00)),
        material=cast_iron,
        name="front_cheek_right",
    )
    head.visual(
        Box((0.020, 0.08, 0.12)),
        origin=Origin(xyz=(-0.045, spindle_y - column_y, 0.00)),
        material=cast_iron,
        name="front_cheek_left",
    )
    head.visual(
        Box((0.10, 0.08, 0.035)),
        origin=Origin(xyz=(0.0, spindle_y - column_y, 0.0625)),
        material=cast_iron,
        name="front_cap",
    )
    head.visual(
        spindle_nose,
        origin=Origin(xyz=(0.0, spindle_y - column_y, -0.03)),
        material=dark_machine,
        name="spindle_nose",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(xyz=(0.11, 0.11, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="feed_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        start = (
            0.128,
            0.11 + 0.014 * math.cos(angle),
            -0.005 + 0.014 * math.sin(angle),
        )
        end = (
            0.145,
            0.11 + 0.095 * math.cos(angle),
            -0.005 + 0.095 * math.sin(angle),
        )
        _add_member(
            head,
            start,
            end,
            radius=0.005,
            material=steel,
            name=f"feed_handle_{index}",
        )
        head.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=end),
            material=handle_black,
            name=f"feed_knob_{index}",
        )
    head.inertial = Inertial.from_geometry(
        Box((0.22, 0.25, 0.33)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.08, 0.07)),
    )

    table = model.part("table")
    table_collar = _annulus_mesh(
        "drill_press_table_collar",
        outer_radius=0.055,
        inner_radius=0.034,
        height=0.12,
    )
    table.visual(table_collar, material=cast_iron, name="table_collar")
    table.visual(
        Box((0.06, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.105, -0.01)),
        material=cast_iron,
        name="table_support",
    )
    table.visual(
        Cylinder(radius=0.040, length=0.03),
        origin=Origin(xyz=(0.0, 0.135, 0.005)),
        material=dark_machine,
        name="table_boss",
    )
    table.visual(
        Cylinder(radius=0.12, length=0.02),
        origin=Origin(xyz=(0.0, spindle_y - column_y, 0.01)),
        material=cast_iron,
        name="table_disk",
    )
    table.visual(
        Box((0.04, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.12, -0.03)),
        material=dark_machine,
        name="table_web",
    )
    table.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.072, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="table_clamp_stem",
    )
    table.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.042, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_machine,
        name="table_clamp_pad",
    )
    table.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.098, 0.0, 0.005)),
        material=handle_black,
        name="table_clamp_knob",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.26, 0.30, 0.16)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.030, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=polished_steel,
        name="quill_body",
    )
    quill.visual(
        Cylinder(radius=0.022, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.1775)),
        material=polished_steel,
        name="spindle_stub",
    )
    quill.visual(
        Cylinder(radius=0.024, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=dark_machine,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.005, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.295)),
        material=steel,
        name="drill_bit",
    )
    quill.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.36),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
    )

    model.articulation(
        "frame_to_head",
        ArticulationType.FIXED,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, column_y, head_z)),
    )
    model.articulation(
        "table_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table,
        origin=Origin(xyz=(0.0, column_y, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.12),
    )
    model.articulation(
        "quill_feed",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, spindle_y - column_y, -0.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.20, lower=0.0, upper=0.09),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")
    table_slide = object_model.get_articulation("table_slide")
    quill_feed = object_model.get_articulation("quill_feed")

    ctx.expect_overlap(
        table,
        frame,
        axes="z",
        elem_a="table_collar",
        elem_b="column",
        min_overlap=0.11,
        name="table collar spans the column at rest",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="xy",
        elem_a="quill_body",
        elem_b="spindle_nose",
        min_overlap=0.055,
        name="quill stays centered in the spindle nose",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="z",
        elem_a="quill_body",
        elem_b="spindle_nose",
        min_overlap=0.055,
        name="quill retains insertion in the spindle nose at rest",
    )

    rest_table = ctx.part_world_position(table)
    with ctx.pose({table_slide: 0.12}):
        ctx.expect_overlap(
            table,
            frame,
            axes="z",
            elem_a="table_collar",
            elem_b="column",
            min_overlap=0.11,
            name="table collar remains engaged at maximum height",
        )
        raised_table = ctx.part_world_position(table)
    ctx.check(
        "table rises straight up the column",
        rest_table is not None
        and raised_table is not None
        and raised_table[2] > rest_table[2] + 0.10
        and abs(raised_table[0] - rest_table[0]) < 1e-6
        and abs(raised_table[1] - rest_table[1]) < 1e-6,
        details=f"rest={rest_table}, raised={raised_table}",
    )

    rest_quill = ctx.part_world_position(quill)
    with ctx.pose({quill_feed: 0.09}):
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            elem_a="quill_body",
            elem_b="spindle_nose",
            min_overlap=0.055,
            name="quill retains insertion at maximum feed",
        )
        fed_quill = ctx.part_world_position(quill)
    ctx.check(
        "quill drops downward under positive feed",
        rest_quill is not None
        and fed_quill is not None
        and fed_quill[2] < rest_quill[2] - 0.08
        and abs(fed_quill[0] - rest_quill[0]) < 1e-6
        and abs(fed_quill[1] - rest_quill[1]) < 1e-6,
        details=f"rest={rest_quill}, fed={fed_quill}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
