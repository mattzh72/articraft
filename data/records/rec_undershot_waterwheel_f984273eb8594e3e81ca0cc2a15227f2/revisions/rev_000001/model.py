from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    timber = model.material("timber", rgba=(0.52, 0.36, 0.20, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.36, 0.23, 0.12, 1.0))
    iron = model.material("iron", rgba=(0.33, 0.35, 0.37, 1.0))
    stone = model.material("stone", rgba=(0.58, 0.58, 0.56, 1.0))

    axle_z = 0.80
    wheel_half_width = 0.41
    rim_center_radius = 0.47
    paddle_center_radius = 0.46
    wheel_outer_radius = 0.56
    base_side_x = 0.82

    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=rim_center_radius, tube=0.020, radial_segments=18, tubular_segments=64).rotate_y(pi / 2.0),
        "waterwheel_rim",
    )

    base = model.part("base")
    base.visual(
        Box((0.34, 1.64, 0.10)),
        origin=Origin(xyz=(base_side_x, 0.0, 0.05)),
        material=dark_timber,
        name="left_ground_sill",
    )
    base.visual(
        Box((0.34, 1.64, 0.10)),
        origin=Origin(xyz=(-base_side_x, 0.0, 0.05)),
        material=dark_timber,
        name="right_ground_sill",
    )
    base.visual(
        Box((1.40, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.54, 0.14)),
        material=timber,
        name="front_tie_beam",
    )
    base.visual(
        Box((1.40, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, -0.54, 0.14)),
        material=timber,
        name="rear_tie_beam",
    )
    base.visual(
        Box((1.32, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=timber,
        name="center_tie_beam",
    )
    base.visual(
        Box((0.24, 0.30, 0.16)),
        origin=Origin(xyz=(base_side_x, 0.0, 0.08)),
        material=stone,
        name="left_plinth",
    )
    base.visual(
        Box((0.24, 0.30, 0.16)),
        origin=Origin(xyz=(-base_side_x, 0.0, 0.08)),
        material=stone,
        name="right_plinth",
    )
    base.inertial = Inertial.from_geometry(
        Box((1.98, 1.72, 0.28)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.035, length=1.48),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.11, length=0.84),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_timber,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.16, length=0.05),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="hub_flange_left",
    )
    wheel.visual(
        Cylinder(radius=0.16, length=0.05),
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="hub_flange_right",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(wheel_half_width, 0.0, 0.0)),
        material=iron,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(-wheel_half_width, 0.0, 0.0)),
        material=iron,
        name="right_rim",
    )

    spoke_count = 10
    for index in range(spoke_count):
        angle = 2.0 * pi * index / spoke_count
        spoke_radius = 0.10 + 0.5 * (rim_center_radius - 0.10)
        paddle_span = wheel_outer_radius - 0.34
        wheel.visual(
            Box((0.82, 0.040, rim_center_radius - 0.10)),
            origin=Origin(
                xyz=(0.0, -spoke_radius * sin(angle), spoke_radius * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_timber,
            name=f"spoke_{index:02d}",
        )
        wheel.visual(
            Box((0.84, 0.075, paddle_span)),
            origin=Origin(
                xyz=(
                    0.0,
                    -paddle_center_radius * sin(angle),
                    paddle_center_radius * cos(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=timber,
            name=f"paddle_{index:02d}",
        )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_outer_radius, length=1.48),
        mass=95.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    def add_bearing_module(name: str) -> None:
        module = model.part(name)
        module.visual(
            Box((0.18, 0.58, 0.12)),
            origin=Origin(xyz=(0.0, 0.0, 0.06)),
            material=timber,
            name="sole_block",
        )
        module.visual(
            Box((0.12, 0.16, 0.88)),
            origin=Origin(xyz=(0.0, 0.19, 0.48)),
            material=timber,
            name="front_post",
        )
        module.visual(
            Box((0.12, 0.16, 0.88)),
            origin=Origin(xyz=(0.0, -0.19, 0.48)),
            material=timber,
            name="rear_post",
        )
        module.visual(
            Box((0.16, 0.58, 0.12)),
            origin=Origin(xyz=(0.0, 0.0, 0.90)),
            material=dark_timber,
            name="cap_beam",
        )
        module.visual(
            Box((0.16, 0.22, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, 0.70)),
            material=timber,
            name="lower_bearing_saddle",
        )
        module.visual(
            Box((0.16, 0.08, 0.18)),
            origin=Origin(xyz=(0.0, 0.10, axle_z - 0.16)),
            material=iron,
            name="front_bearing_cheek",
        )
        module.visual(
            Box((0.16, 0.08, 0.18)),
            origin=Origin(xyz=(0.0, -0.10, axle_z - 0.16)),
            material=iron,
            name="rear_bearing_cheek",
        )
        module.visual(
            Box((0.16, 0.22, 0.05)),
            origin=Origin(xyz=(0.0, 0.0, axle_z - 0.01)),
            material=iron,
            name="upper_bearing_cap",
        )
        module.visual(
            Cylinder(radius=0.055, length=0.020),
            origin=Origin(xyz=(-0.07, 0.0, axle_z - 0.16), rpy=(0.0, pi / 2.0, 0.0)),
            material=iron,
            name="journal_face_a",
        )
        module.visual(
            Cylinder(radius=0.055, length=0.020),
            origin=Origin(xyz=(0.07, 0.0, axle_z - 0.16), rpy=(0.0, pi / 2.0, 0.0)),
            material=iron,
            name="journal_face_b",
        )
        module.inertial = Inertial.from_geometry(
            Box((0.22, 0.62, 1.16)),
            mass=38.0,
            origin=Origin(xyz=(0.0, 0.0, 0.58)),
        )

    add_bearing_module("left_bearing_module")
    add_bearing_module("right_bearing_module")

    def add_head_module(name: str, side_sign: float) -> None:
        module = model.part(name)
        module.visual(
            Box((0.12, 0.16, 0.22)),
            origin=Origin(xyz=(0.0, 0.18, 0.11)),
            material=timber,
            name="front_head_riser",
        )
        module.visual(
            Box((0.12, 0.16, 0.22)),
            origin=Origin(xyz=(0.0, -0.18, 0.11)),
            material=timber,
            name="rear_head_riser",
        )
        module.visual(
            Box((0.18, 0.54, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, 0.27)),
            material=dark_timber,
            name="head_cap",
        )
        module.visual(
            Box((0.18, 0.18, 0.14)),
            origin=Origin(xyz=(0.0, 0.22, 0.39)),
            material=timber,
            name="trough_cheek",
        )
        module.visual(
            Box((0.82, 0.12, 0.10)),
            origin=Origin(xyz=(-0.41 * side_sign, 0.37, 0.37)),
            material=timber,
            name="trough_half_edge",
        )
        module.inertial = Inertial.from_geometry(
            Box((0.22, 0.58, 0.44)),
            mass=16.0,
            origin=Origin(xyz=(0.0, 0.0, 0.22)),
        )

    add_head_module("left_head_module", 1.0)
    add_head_module("right_head_module", -1.0)

    model.articulation(
        "left_bearing_mount",
        ArticulationType.FIXED,
        parent=base,
        child="left_bearing_module",
        origin=Origin(xyz=(base_side_x, 0.0, 0.16)),
    )
    model.articulation(
        "right_bearing_mount",
        ArticulationType.FIXED,
        parent=base,
        child="right_bearing_module",
        origin=Origin(xyz=(-base_side_x, 0.0, 0.16)),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.4),
    )
    model.articulation(
        "left_head_mount",
        ArticulationType.FIXED,
        parent=base,
        child="left_head_module",
        origin=Origin(xyz=(base_side_x, 0.0, 1.12)),
    )
    model.articulation(
        "right_head_mount",
        ArticulationType.FIXED,
        parent=base,
        child="right_head_module",
        origin=Origin(xyz=(-base_side_x, 0.0, 1.12)),
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
    wheel = object_model.get_part("wheel")
    left_bearing = object_model.get_part("left_bearing_module")
    right_bearing = object_model.get_part("right_bearing_module")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_contact(
        wheel,
        left_bearing,
        elem_a="axle",
        elem_b="journal_face_a",
        contact_tol=0.001,
        name="left bearing supports the axle",
    )
    ctx.expect_contact(
        wheel,
        right_bearing,
        elem_a="axle",
        elem_b="journal_face_b",
        contact_tol=0.001,
        name="right bearing supports the axle",
    )

    paddle_rest = ctx.part_element_world_aabb(wheel, elem="paddle_00")
    with ctx.pose({spin: pi / 2.0}):
        paddle_turned = ctx.part_element_world_aabb(wheel, elem="paddle_00")
    rest_center_y = None if paddle_rest is None else 0.5 * (paddle_rest[0][1] + paddle_rest[1][1])
    turned_center_y = None if paddle_turned is None else 0.5 * (paddle_turned[0][1] + paddle_turned[1][1])
    ctx.check(
        "wheel articulation rotates a named paddle around the axle",
        rest_center_y is not None and turned_center_y is not None and turned_center_y < rest_center_y - 0.20,
        details=f"rest_center_y={rest_center_y}, turned_center_y={turned_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
