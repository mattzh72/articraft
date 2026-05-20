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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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
    model = ArticulatedObject(name="sensitive_bench_drill_press")

    casting = model.material("casting_green", rgba=(0.24, 0.37, 0.31, 1.0))
    table_metal = model.material("table_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.22, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=casting,
        name="base_plate",
    )
    base.visual(
        Box((0.16, 0.14, 0.020)),
        origin=Origin(xyz=(0.090, 0.0, 0.034)),
        material=casting,
        name="front_pad",
    )
    base.visual(
        Box((0.095, 0.110, 0.028)),
        origin=Origin(xyz=(-0.020, 0.0, 0.026)),
        material=casting,
        name="base_web",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.068),
        origin=Origin(xyz=(-0.070, 0.0, 0.058)),
        material=casting,
        name="column_socket",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.080)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.021, length=0.575),
        origin=Origin(xyz=(0.0, 0.0, 0.2875)),
        material=machined_steel,
        name="column_shaft",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.575),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.2875)),
    )

    head = model.part("head")
    head.visual(
        Box((0.050, 0.018, 0.122)),
        origin=Origin(xyz=(0.000, 0.032, 0.061)),
        material=casting,
        name="head_left_clamp",
    )
    head.visual(
        Box((0.050, 0.018, 0.122)),
        origin=Origin(xyz=(0.000, -0.032, 0.061)),
        material=casting,
        name="head_right_clamp",
    )
    head.visual(
        Box((0.024, 0.086, 0.122)),
        origin=Origin(xyz=(-0.033, 0.0, 0.061)),
        material=casting,
        name="head_back_bridge",
    )
    head.visual(
        Box((0.090, 0.094, 0.026)),
        origin=Origin(xyz=(0.025, 0.0, 0.122)),
        material=casting,
        name="head_top_cap",
    )
    head.visual(
        Box((0.120, 0.094, 0.100)),
        origin=Origin(xyz=(0.084, 0.0, 0.068)),
        material=casting,
        name="head_body",
    )
    head.visual(
        Box((0.060, 0.090, 0.040)),
        origin=Origin(xyz=(0.020, 0.0, 0.155)),
        material=casting,
        name="belt_cover",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.052),
        origin=Origin(xyz=(0.118, 0.0, 0.026)),
        material=machined_steel,
        name="quill_sleeve",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(
            xyz=(0.065, -0.058, 0.072),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=machined_steel,
        name="feed_pivot",
    )
    _add_member(
        head,
        (0.078, -0.069, 0.072),
        (0.106, -0.153, 0.020),
        radius=0.0042,
        material=machined_steel,
        name="fine_feed_lever",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(
            xyz=(0.106, -0.165, 0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_handle,
        name="lever_grip",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.180, 0.110, 0.190)),
        mass=5.5,
        origin=Origin(xyz=(0.050, 0.0, 0.085)),
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.014, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=machined_steel,
        name="quill_body",
    )
    quill.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        material=machined_steel,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=machined_steel,
        name="drill_bit",
    )
    quill.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.190)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.042, 0.016, 0.060)),
        origin=Origin(xyz=(0.000, 0.032, 0.0)),
        material=casting,
        name="carriage_left_clamp",
    )
    table_carriage.visual(
        Box((0.042, 0.016, 0.060)),
        origin=Origin(xyz=(0.000, -0.032, 0.0)),
        material=casting,
        name="carriage_right_clamp",
    )
    table_carriage.visual(
        Box((0.024, 0.080, 0.060)),
        origin=Origin(xyz=(-0.033, 0.0, 0.0)),
        material=casting,
        name="carriage_back_bridge",
    )
    table_carriage.visual(
        Box((0.070, 0.028, 0.018)),
        origin=Origin(xyz=(0.060, 0.0, -0.010)),
        material=casting,
        name="table_arm",
    )
    table_carriage.visual(
        Box((0.074, 0.010, 0.040)),
        origin=Origin(xyz=(0.061, 0.013, -0.010)),
        material=casting,
        name="left_gusset",
    )
    table_carriage.visual(
        Box((0.074, 0.010, 0.040)),
        origin=Origin(xyz=(0.061, -0.013, -0.010)),
        material=casting,
        name="right_gusset",
    )
    table_carriage.visual(
        Box((0.010, 0.016, 0.040)),
        origin=Origin(xyz=(0.026, 0.023, -0.010)),
        material=casting,
        name="left_clamp_bridge",
    )
    table_carriage.visual(
        Box((0.010, 0.016, 0.040)),
        origin=Origin(xyz=(0.026, -0.023, -0.010)),
        material=casting,
        name="right_clamp_bridge",
    )
    table_carriage.visual(
        Box((0.020, 0.010, 0.032)),
        origin=Origin(xyz=(0.107, 0.018, -0.006)),
        material=casting,
        name="left_fork",
    )
    table_carriage.visual(
        Box((0.020, 0.010, 0.032)),
        origin=Origin(xyz=(0.107, -0.018, -0.006)),
        material=casting,
        name="right_fork",
    )
    table_carriage.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(
            xyz=(0.105, 0.018, -0.006),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=machined_steel,
        name="left_trunnion",
    )
    table_carriage.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(
            xyz=(0.105, -0.018, -0.006),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=machined_steel,
        name="right_trunnion",
    )
    table_carriage.inertial = Inertial.from_geometry(
        Box((0.160, 0.100, 0.080)),
        mass=1.4,
        origin=Origin(xyz=(0.040, 0.0, -0.004)),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.0095, length=0.022),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=machined_steel,
        name="table_barrel",
    )
    table.visual(
        Box((0.064, 0.022, 0.026)),
        origin=Origin(xyz=(0.034, 0.0, 0.013)),
        material=casting,
        name="table_support_rib",
    )
    table.visual(
        Cylinder(radius=0.082, length=0.014),
        origin=Origin(xyz=(0.062, 0.0, 0.032)),
        material=table_metal,
        name="table_surface",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.170, 0.170, 0.060)),
        mass=1.2,
        origin=Origin(xyz=(0.050, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(-0.070, 0.0, 0.092)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
    )
    model.articulation(
        "column_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.15,
            lower=0.0,
            upper=0.180,
        ),
    )
    model.articulation(
        "table_carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.105, 0.0, -0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.8,
            lower=math.radians(-35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
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

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")

    table_lift = object_model.get_articulation("column_to_table_carriage")
    table_tilt = object_model.get_articulation("table_carriage_to_table")
    quill_feed = object_model.get_articulation("head_to_quill")

    ctx.expect_gap(
        column,
        base,
        axis="z",
        positive_elem="column_shaft",
        negative_elem="column_socket",
        max_gap=0.0015,
        max_penetration=1e-6,
        name="column seats on the base socket",
    )
    ctx.expect_origin_gap(
        head,
        column,
        axis="z",
        min_gap=0.574,
        max_gap=0.576,
        name="head is mounted at the top of the slim column",
    )
    ctx.expect_overlap(
        table,
        quill,
        axes="xy",
        elem_a="table_surface",
        elem_b="quill_body",
        min_overlap=0.010,
        name="table sits under the spindle line",
    )

    rest_table_pos = ctx.part_world_position(table_carriage)
    raised_table_pos = None
    with ctx.pose({table_lift: table_lift.motion_limits.upper}):
        raised_table_pos = ctx.part_world_position(table_carriage)
    ctx.check(
        "table slides upward on the column",
        rest_table_pos is not None
        and raised_table_pos is not None
        and raised_table_pos[2] > rest_table_pos[2] + 0.15,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_surface")
    tilted_table_aabb = None
    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_surface")
    ctx.check(
        "table tilts at the mount bracket",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_aabb[1][2] > rest_table_aabb[1][2] + 0.025,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_quill_pos = ctx.part_world_position(quill)
    extended_quill_pos = None
    with ctx.pose({quill_feed: quill_feed.motion_limits.upper}):
        extended_quill_pos = ctx.part_world_position(quill)
    ctx.check(
        "quill feeds downward from the head",
        rest_quill_pos is not None
        and extended_quill_pos is not None
        and extended_quill_pos[2] < rest_quill_pos[2] - 0.045,
        details=f"rest={rest_quill_pos}, extended={extended_quill_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
