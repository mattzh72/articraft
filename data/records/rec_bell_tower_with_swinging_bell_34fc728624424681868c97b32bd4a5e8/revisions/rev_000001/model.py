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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    thickness: float,
    material,
    name: str | None = None,
    pad: float = 0.03,
) -> None:
    part.visual(
        Box((thickness, thickness, _distance(a, b) + pad)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def _build_bell_shell():
    outer_profile = [
        (0.080, -0.240),
        (0.120, -0.275),
        (0.185, -0.395),
        (0.285, -0.650),
        (0.405, -0.965),
        (0.515, -1.240),
        (0.565, -1.410),
        (0.580, -1.500),
    ]
    inner_profile = [
        (0.030, -0.220),
        (0.075, -0.265),
        (0.135, -0.385),
        (0.225, -0.625),
        (0.330, -0.930),
        (0.430, -1.200),
        (0.490, -1.405),
        (0.505, -1.475),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="round",
            end_cap="flat",
            lip_samples=10,
        ),
        "campanile_bell_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_campanile_bell_tower")

    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.65, 1.0))
    steel = model.material("steel", rgba=(0.33, 0.36, 0.40, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    cap_steel = model.material("cap_steel", rgba=(0.44, 0.46, 0.50, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.45, 0.29, 0.15, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((3.60, 3.60, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=concrete,
        name="base_pad",
    )
    tower.visual(
        Box((2.25, 2.25, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=concrete,
        name="plinth",
    )
    tower.visual(
        Box((1.12, 1.12, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=steel_dark,
        name="base_table",
    )

    tower_half = 1.45
    column_bottom = 0.80
    column_top = 9.45
    lower_levels = [0.80, 2.00, 3.20, 4.40, 5.60, 6.80, 7.75]
    upper_levels = [7.75, 8.45, 9.10]
    corners = [
        (-tower_half, -tower_half),
        (tower_half, -tower_half),
        (tower_half, tower_half),
        (-tower_half, tower_half),
    ]

    for x, y in corners:
        _add_box_member(
            tower,
            (x, y, column_bottom),
            (x, y, column_top),
            thickness=0.10,
            material=steel,
        )

    base_transition = 0.52
    for x, y in (
        (-base_transition, -base_transition),
        (base_transition, -base_transition),
        (base_transition, base_transition),
        (-base_transition, base_transition),
    ):
        _add_box_member(
            tower,
            (x, y, 0.78),
            (1.45 if x > 0.0 else -1.45, 1.45 if y > 0.0 else -1.45, 1.35),
            thickness=0.060,
            material=steel_dark,
        )

    for z in lower_levels + upper_levels[1:] + [column_top]:
        for index in range(4):
            x0, y0 = corners[index]
            x1, y1 = corners[(index + 1) % 4]
            _add_box_member(
                tower,
                (x0, y0, z),
                (x1, y1, z),
                thickness=0.060,
                material=steel,
            )

    face_corner_pairs = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
    )
    for level_index in range(len(lower_levels) - 1):
        z0 = lower_levels[level_index]
        z1 = lower_levels[level_index + 1]
        for left_index, right_index in face_corner_pairs:
            x0, y0 = corners[left_index]
            x1, y1 = corners[right_index]
            _add_box_member(
                tower,
                (x0, y0, z0),
                (x1, y1, z1),
                thickness=0.045,
                material=steel,
            )
            _add_box_member(
                tower,
                (x1, y1, z0),
                (x0, y0, z1),
                thickness=0.045,
                material=steel,
            )

    inner_column_x = 0.82
    inner_column_bottom = 7.75
    inner_column_top = 9.10
    for x in (-inner_column_x, inner_column_x):
        _add_box_member(
            tower,
            (x, 0.0, inner_column_bottom),
            (x, 0.0, inner_column_top),
            thickness=0.10,
            material=steel_dark,
        )

    for z in (7.75, 8.45, 9.10):
        _add_box_member(
            tower,
            (-inner_column_x, 0.0, z),
            (-tower_half, -tower_half, z),
            thickness=0.040,
            material=steel,
        )
        _add_box_member(
            tower,
            (-inner_column_x, 0.0, z),
            (-tower_half, tower_half, z),
            thickness=0.040,
            material=steel,
        )
        _add_box_member(
            tower,
            (inner_column_x, 0.0, z),
            (tower_half, -tower_half, z),
            thickness=0.040,
            material=steel,
        )
        _add_box_member(
            tower,
            (inner_column_x, 0.0, z),
            (tower_half, tower_half, z),
            thickness=0.040,
            material=steel,
        )

    support_beam_z = 8.86
    tower.visual(
        Box((1.78, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, support_beam_z)),
        material=steel_dark,
        name="support_beam",
    )

    bearing_center_x = 0.71
    bearing_z = 8.56
    tower.visual(
        Box((0.12, 0.18, 0.18)),
        origin=Origin(xyz=(-bearing_center_x, 0.0, bearing_z)),
        material=bearing_steel,
        name="left_bearing",
    )
    tower.visual(
        Box((0.12, 0.18, 0.18)),
        origin=Origin(xyz=(bearing_center_x, 0.0, bearing_z)),
        material=bearing_steel,
        name="right_bearing",
    )

    cap_post_top = 9.80
    for x, y in corners:
        _add_box_member(
            tower,
            (x, y, column_top),
            (x, y, cap_post_top),
            thickness=0.075,
            material=cap_steel,
        )
    tower.visual(
        Box((2.70, 2.70, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 9.74)),
        material=cap_steel,
        name="cap_subframe",
    )
    tower.visual(
        Box((3.00, 3.00, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 9.86)),
        material=cap_steel,
        name="flat_cap",
    )
    tower.inertial = Inertial.from_geometry(
        Box((3.60, 3.60, 10.00)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 5.00)),
    )

    bell = model.part("bell")
    bell.visual(_build_bell_shell(), material=bell_bronze, name="bell_shell")
    bell.visual(
        Box((1.22, 0.20, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=steel_dark,
        name="headstock",
    )
    bell.visual(
        Cylinder(radius=0.036, length=0.05),
        origin=Origin(xyz=(-0.625, 0.0, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="left_journal",
    )
    bell.visual(
        Cylinder(radius=0.036, length=0.05),
        origin=Origin(xyz=(0.625, 0.0, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="right_journal",
    )
    bell.visual(
        Box((0.20, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
        material=steel_dark,
        name="crown_block",
    )
    bell.visual(
        Box((0.08, 0.05, 0.26)),
        origin=Origin(xyz=(-0.14, 0.0, -0.14)),
        material=steel_dark,
        name="left_yoke",
    )
    bell.visual(
        Box((0.08, 0.05, 0.26)),
        origin=Origin(xyz=(0.14, 0.0, -0.14)),
        material=steel_dark,
        name="right_yoke",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.58, length=1.65),
        mass=650.0,
        origin=Origin(xyz=(0.0, 0.0, -0.62)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, bearing_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.8,
            lower=-0.55,
            upper=0.55,
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

    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    swing = object_model.get_articulation("bell_swing")

    ctx.expect_contact(
        bell,
        tower,
        elem_a="left_journal",
        elem_b="left_bearing",
        contact_tol=0.002,
        name="left bell journal seats in the left bearing",
    )
    ctx.expect_contact(
        bell,
        tower,
        elem_a="right_journal",
        elem_b="right_bearing",
        contact_tol=0.002,
        name="right bell journal seats in the right bearing",
    )
    ctx.expect_gap(
        tower,
        bell,
        axis="z",
        positive_elem="support_beam",
        negative_elem="headstock",
        min_gap=0.10,
        max_gap=0.24,
        name="headstock hangs below the belfry support beam",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="x",
        elem_a="headstock",
        elem_b="support_beam",
        min_overlap=1.10,
        name="headstock spans beneath the support beam",
    )

    rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    with ctx.pose({swing: 0.40}):
        swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")

    if rest_aabb is None or swung_aabb is None:
        ctx.fail(
            "bell shell aabbs available for swing check",
            details=f"rest={rest_aabb}, swung={swung_aabb}",
        )
    else:
        rest_center_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
        swung_center_y = 0.5 * (swung_aabb[0][1] + swung_aabb[1][1])
        ctx.check(
            "positive bell swing moves the bell toward +y",
            swung_center_y > rest_center_y + 0.18,
            details=f"rest_center_y={rest_center_y:.4f}, swung_center_y={swung_center_y:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
