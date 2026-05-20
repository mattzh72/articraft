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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="router_with_kickstand_and_antennas")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    led_blue = model.material("led_blue", rgba=(0.33, 0.73, 0.96, 1.0))

    body_w = 0.086
    body_d = 0.018
    body_h = 0.132
    shell_t = 0.0018
    overlap = 0.0012

    antenna_mount_w = 0.012
    antenna_mount_d = 0.010
    antenna_mount_h = 0.004
    antenna_x = body_w * 0.5 - 0.011
    antenna_y = -0.0035
    antenna_pivot_r = 0.004
    antenna_pivot_len = 0.014
    antenna_pivot_z = body_h + antenna_mount_h + antenna_pivot_r
    antenna_stem_r = 0.0035
    antenna_stem_len = 0.052

    stand_w = 0.056
    stand_t = 0.0026
    stand_h = 0.084
    stand_barrel_r = 0.0038
    stand_barrel_len = 0.058
    stand_hinge_z = 0.016

    body = model.part("router_body")
    body.visual(
        Box((body_w, shell_t, body_h)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - shell_t * 0.5, body_h * 0.5)),
        material=shell_white,
        name="front_cover",
    )
    body.visual(
        Box((body_w, shell_t, body_h)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + shell_t * 0.5, body_h * 0.5)),
        material=shell_white,
        name="rear_cover",
    )
    body.visual(
        Box((shell_t + 0.0004, body_d - overlap, body_h - overlap)),
        origin=Origin(xyz=(body_w * 0.5 - (shell_t + 0.0004) * 0.5, 0.0, body_h * 0.5)),
        material=shell_white,
        name="right_wall",
    )
    body.visual(
        Box((shell_t + 0.0004, body_d - overlap, body_h - overlap)),
        origin=Origin(xyz=(-body_w * 0.5 + (shell_t + 0.0004) * 0.5, 0.0, body_h * 0.5)),
        material=shell_white,
        name="left_wall",
    )
    body.visual(
        Box((body_w - overlap, body_d - overlap, shell_t + 0.0004)),
        origin=Origin(xyz=(0.0, 0.0, body_h - (shell_t + 0.0004) * 0.5)),
        material=shell_white,
        name="top_shell",
    )
    body.visual(
        Box((body_w - overlap, body_d - overlap, shell_t + 0.0004)),
        origin=Origin(xyz=(0.0, 0.0, (shell_t + 0.0004) * 0.5)),
        material=shell_white,
        name="bottom_shell",
    )
    body.visual(
        Box((0.068, 0.0010, 0.098)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - 0.0005, 0.072)),
        material=bezel_black,
        name="front_bezel",
    )
    body.visual(
        Box((0.026, 0.0008, 0.006)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - 0.0004, 0.020)),
        material=led_blue,
        name="status_window",
    )
    body.visual(
        Box((antenna_mount_w, antenna_mount_d, antenna_mount_h)),
        origin=Origin(xyz=(-antenna_x, antenna_y, body_h + antenna_mount_h * 0.5)),
        material=bezel_black,
        name="left_antenna_mount",
    )
    body.visual(
        Box((antenna_mount_w, antenna_mount_d, antenna_mount_h)),
        origin=Origin(xyz=(antenna_x, antenna_y, body_h + antenna_mount_h * 0.5)),
        material=bezel_black,
        name="right_antenna_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h + antenna_mount_h)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, (body_h + antenna_mount_h) * 0.5)),
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=stand_barrel_r, length=stand_barrel_len),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=rubber_black,
        name="stand_hinge_barrel",
    )
    kickstand.visual(
        Box((stand_w, stand_t, stand_h)),
        origin=Origin(
            xyz=(0.0, stand_barrel_r - stand_t * 0.5, stand_h * 0.5),
        ),
        material=bezel_black,
        name="stand_plate",
    )
    kickstand.visual(
        Box((stand_w * 0.78, stand_t * 1.15, 0.010)),
        origin=Origin(
            xyz=(0.0, stand_barrel_r - stand_t * 0.5, stand_h - 0.005),
        ),
        material=bezel_black,
        name="stand_top_rib",
    )
    kickstand.inertial = Inertial.from_geometry(
        Box((stand_w, 0.010, stand_h)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0015, stand_h * 0.5)),
    )

    def add_stub_antenna(part_name: str) -> None:
        antenna = model.part(part_name)
        antenna.visual(
            Cylinder(radius=antenna_pivot_r, length=antenna_pivot_len),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=rubber_black,
            name="antenna_barrel",
        )
        antenna.visual(
            Box((0.010, 0.006, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=rubber_black,
            name="antenna_base_block",
        )
        antenna.visual(
            Cylinder(radius=antenna_stem_r, length=antenna_stem_len),
            origin=Origin(xyz=(0.0, 0.0, 0.034)),
            material=rubber_black,
            name="stub_radiator",
        )
        antenna.inertial = Inertial.from_geometry(
            Box((0.016, 0.012, 0.060)),
            mass=0.015,
            origin=Origin(xyz=(0.0, 0.0, 0.030)),
        )

    add_stub_antenna("left_antenna")
    add_stub_antenna("right_antenna")

    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, -body_d * 0.5 - stand_barrel_r, stand_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=1.12,
        ),
    )
    model.articulation(
        "left_antenna_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child="left_antenna",
        origin=Origin(xyz=(-antenna_x, antenna_y, antenna_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=-0.30,
            upper=1.35,
        ),
    )
    model.articulation(
        "right_antenna_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child="right_antenna",
        origin=Origin(xyz=(antenna_x, antenna_y, antenna_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=-0.30,
            upper=1.35,
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

    body = object_model.get_part("router_body")
    kickstand = object_model.get_part("kickstand")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")

    kickstand_hinge = object_model.get_articulation("kickstand_hinge")
    left_antenna_pivot = object_model.get_articulation("left_antenna_pivot")
    right_antenna_pivot = object_model.get_articulation("right_antenna_pivot")

    for part_name, part in (
        ("router_body", body),
        ("kickstand", kickstand),
        ("left_antenna", left_antenna),
        ("right_antenna", right_antenna),
    ):
        ctx.check(f"{part_name} exists", part is not None, details=f"missing part {part_name}")

    ctx.expect_contact(
        left_antenna,
        body,
        elem_a="antenna_barrel",
        elem_b="left_antenna_mount",
        contact_tol=0.0002,
        name="left antenna barrel seats on left mount",
    )
    ctx.expect_contact(
        right_antenna,
        body,
        elem_a="antenna_barrel",
        elem_b="right_antenna_mount",
        contact_tol=0.0002,
        name="right antenna barrel seats on right mount",
    )
    ctx.expect_gap(
        body,
        kickstand,
        axis="y",
        positive_elem="rear_cover",
        negative_elem="stand_plate",
        max_gap=0.0002,
        max_penetration=0.00001,
        name="kickstand closes flush against rear cover",
    )
    ctx.expect_overlap(
        body,
        kickstand,
        axes="xz",
        elem_a="rear_cover",
        elem_b="stand_plate",
        min_overlap=0.040,
        name="kickstand spans a broad rear support area when closed",
    )
    ctx.expect_gap(
        left_antenna,
        body,
        axis="z",
        positive_elem="stub_radiator",
        negative_elem="top_shell",
        min_gap=0.006,
        name="left antenna rises above the top shell",
    )
    ctx.expect_gap(
        right_antenna,
        body,
        axis="z",
        positive_elem="stub_radiator",
        negative_elem="top_shell",
        min_gap=0.006,
        name="right antenna rises above the top shell",
    )
    ctx.expect_origin_distance(
        left_antenna,
        right_antenna,
        axes="x",
        min_dist=0.055,
        max_dist=0.075,
        name="antenna pivots sit at opposite top corners",
    )

    with ctx.pose({kickstand_hinge: 0.95}):
        ctx.expect_gap(
            body,
            kickstand,
            axis="y",
            positive_elem="rear_cover",
            negative_elem="stand_top_rib",
            min_gap=0.050,
            name="opened kickstand lifts its upper support rib behind the router body",
        )

    closed_stand_aabb = ctx.part_element_world_aabb(kickstand, elem="stand_plate")
    left_rest_aabb = ctx.part_element_world_aabb(left_antenna, elem="stub_radiator")
    right_rest_aabb = ctx.part_element_world_aabb(right_antenna, elem="stub_radiator")
    with ctx.pose(
        {
            kickstand_hinge: 0.95,
            left_antenna_pivot: 1.05,
            right_antenna_pivot: 1.05,
        }
    ):
        opened_stand_aabb = ctx.part_element_world_aabb(kickstand, elem="stand_plate")
        left_folded_aabb = ctx.part_element_world_aabb(left_antenna, elem="stub_radiator")
        right_folded_aabb = ctx.part_element_world_aabb(right_antenna, elem="stub_radiator")

    ctx.check(
        "kickstand opens rearward",
        closed_stand_aabb is not None
        and opened_stand_aabb is not None
        and opened_stand_aabb[0][1] < closed_stand_aabb[0][1] - 0.025,
        details=f"closed={closed_stand_aabb}, opened={opened_stand_aabb}",
    )
    ctx.check(
        "left antenna folds toward the rear",
        left_rest_aabb is not None
        and left_folded_aabb is not None
        and left_folded_aabb[0][1] < left_rest_aabb[0][1] - 0.020,
        details=f"rest={left_rest_aabb}, folded={left_folded_aabb}",
    )
    ctx.check(
        "right antenna folds toward the rear",
        right_rest_aabb is not None
        and right_folded_aabb is not None
        and right_folded_aabb[0][1] < right_rest_aabb[0][1] - 0.020,
        details=f"rest={right_rest_aabb}, folded={right_folded_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
