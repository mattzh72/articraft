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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    body_dark = model.material("body_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.72, 0.75, 0.78, 1.0))
    accent_red = model.material("accent_red", rgba=(0.78, 0.12, 0.10, 1.0))
    head_black = model.material("head_black", rgba=(0.12, 0.13, 0.14, 1.0))

    def yz_section(
        x: float,
        *,
        width: float,
        height: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z_center + z)
            for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
        ]

    body = model.part("motor_body")
    body_shell_geom = section_loft(
        [
            yz_section(-0.165, width=0.070, height=0.105, radius=0.020, z_center=0.110),
            yz_section(-0.080, width=0.108, height=0.145, radius=0.030, z_center=0.112),
            yz_section(0.015, width=0.100, height=0.132, radius=0.028, z_center=0.104),
            yz_section(0.102, width=0.072, height=0.094, radius=0.018, z_center=0.090),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell_geom, "vacuum_body_shell"),
        material=body_dark,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.047, length=0.160),
        origin=Origin(xyz=(-0.005, 0.0, 0.082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_red,
        name="dust_bin",
    )
    body.visual(
        Box((0.038, 0.042, 0.118)),
        origin=Origin(xyz=(-0.096, 0.0, 0.138)),
        material=body_dark,
        name="grip_spine",
    )
    body.visual(
        Box((0.124, 0.032, 0.030)),
        origin=Origin(xyz=(-0.038, 0.0, 0.185)),
        material=body_dark,
        name="handle_bridge",
    )
    body.visual(
        Box((0.050, 0.050, 0.030)),
        origin=Origin(xyz=(0.083, 0.0, 0.060)),
        material=body_dark,
        name="front_collar",
    )
    body.visual(
        Box((0.036, 0.006, 0.050)),
        origin=Origin(xyz=(0.112, -0.022, 0.055)),
        material=body_dark,
        name="left_cheek",
    )
    body.visual(
        Box((0.036, 0.006, 0.050)),
        origin=Origin(xyz=(0.112, 0.022, 0.055)),
        material=body_dark,
        name="right_cheek",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.305, 0.120, 0.210)),
        mass=2.8,
        origin=Origin(xyz=(-0.020, 0.0, 0.105)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.015, length=0.038),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wand_metal,
        name="hinge_barrel",
    )
    wand.visual(
        Box((0.050, 0.028, 0.028)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=body_dark,
        name="hinge_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.016, length=0.650),
        origin=Origin(xyz=(0.345, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wand_metal,
        name="wand_tube",
    )
    wand.visual(
        Box((0.040, 0.046, 0.012)),
        origin=Origin(xyz=(0.660, 0.0, 0.002)),
        material=body_dark,
        name="front_bridge",
    )
    wand.visual(
        Box((0.030, 0.005, 0.046)),
        origin=Origin(xyz=(0.691, -0.0205, -0.017)),
        material=body_dark,
        name="front_cheek_left",
    )
    wand.visual(
        Box((0.030, 0.005, 0.046)),
        origin=Origin(xyz=(0.691, 0.0205, -0.017)),
        material=body_dark,
        name="front_cheek_right",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.740, 0.055, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.370, 0.0, -0.010)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="pivot_barrel",
    )
    floor_head.visual(
        Box((0.042, 0.034, 0.032)),
        origin=Origin(xyz=(0.020, 0.0, -0.018)),
        material=body_dark,
        name="pivot_neck",
    )
    head_shell_geom = ExtrudeGeometry(
        rounded_rect_profile(0.272, 0.108, 0.026, corner_segments=10),
        0.036,
        center=True,
    )
    floor_head.visual(
        mesh_from_geometry(head_shell_geom, "vacuum_floor_head_shell"),
        origin=Origin(xyz=(0.148, 0.0, -0.030)),
        material=head_black,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.220, 0.010, 0.010)),
        origin=Origin(xyz=(0.160, 0.0, -0.014)),
        material=accent_red,
        name="front_bumper",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.280, 0.110, 0.055)),
        mass=1.0,
        origin=Origin(xyz=(0.125, 0.0, -0.028)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.130, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.695, 0.0, -0.008)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=math.radians(-32.0),
            upper=math.radians(26.0),
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
    body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("body_to_wand_fold")
    pitch = object_model.get_articulation("wand_to_floor_head_pitch")

    ctx.expect_gap(
        wand,
        body,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="front_collar",
        min_gap=0.0,
        max_gap=0.01,
        name="fold hinge packs tightly against body collar",
    )
    ctx.expect_gap(
        floor_head,
        wand,
        axis="x",
        positive_elem="pivot_barrel",
        negative_elem="front_bridge",
        min_gap=0.0,
        max_gap=0.01,
        name="floor head pivot nests tightly under wand bridge",
    )

    def center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_wand_front = ctx.part_element_world_aabb(wand, elem="front_bridge")
    with ctx.pose({fold: fold.motion_limits.upper}):
        folded_wand_front = ctx.part_element_world_aabb(wand, elem="front_bridge")
    ctx.check(
        "fold joint swings the wand downward",
        rest_wand_front is not None
        and folded_wand_front is not None
        and center_z(folded_wand_front) < center_z(rest_wand_front) - 0.20,
        details=f"rest={rest_wand_front}, folded={folded_wand_front}",
    )

    rest_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    with ctx.pose({pitch: pitch.motion_limits.upper}):
        pitched_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    ctx.check(
        "floor head pitches its nose upward",
        rest_bumper is not None
        and pitched_bumper is not None
        and center_z(pitched_bumper) > center_z(rest_bumper) + 0.02,
        details=f"rest={rest_bumper}, pitched={pitched_bumper}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
