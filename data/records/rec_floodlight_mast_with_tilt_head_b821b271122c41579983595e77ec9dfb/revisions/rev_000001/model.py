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
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_loop(radius: float, z: float, segments: int = 40) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments), z)
        for i in range(segments)
    )


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = cos(angle)
    s = sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _pitched_origin(xyz: tuple[float, float, float], pitch: float) -> Origin:
    return Origin(xyz=_rotate_y(xyz, pitch), rpy=(0.0, pitch, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roadway_single_arm_floodlight")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.64, 1.0))
    galvanized = model.material("galvanized", rgba=(0.60, 0.62, 0.65, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    powder_black = model.material("powder_black", rgba=(0.14, 0.15, 0.16, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.84, 0.90, 0.55))

    base = model.part("concrete_base")
    base.visual(
        Box((0.90, 0.90, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="base_body",
    )
    base.visual(
        Box((0.38, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.2575)),
        material=concrete,
        name="base_plinth",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.90, 0.90, 0.275)),
        mass=1750.0,
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
    )

    pole = model.part("pole")
    pole_frustum = section_loft(
        [
            _circle_loop(0.105, 0.02, segments=48),
            _circle_loop(0.055, 8.18, segments=48),
        ]
    )
    pole.visual(
        _save_mesh("pole_shaft", pole_frustum),
        material=galvanized,
        name="pole_shaft",
    )
    pole.visual(
        Box((0.28, 0.28, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=dark_metal,
        name="pole_flange",
    )
    pole.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 8.19)),
        material=galvanized,
        name="pole_top_cap",
    )
    pole.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 8.20)),
        mass=190.0,
        origin=Origin(xyz=(0.0, 0.0, 4.10)),
    )

    arm = model.part("outreach_arm")
    arm.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="arm_mount_cap",
    )
    arm.visual(
        Cylinder(radius=0.050, length=0.085),
        origin=Origin(xyz=(0.03, 0.0, 0.052), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="arm_stub",
    )
    arm_tube = tube_from_spline_points(
        [
            (0.02, 0.0, 0.052),
            (0.16, 0.0, 0.090),
            (0.60, 0.0, 0.150),
            (1.08, 0.0, 0.185),
            (1.23, 0.0, 0.182),
        ],
        radius=0.032,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    arm.visual(
        _save_mesh("outreach_arm_tube", arm_tube),
        material=galvanized,
        name="arm_tube",
    )
    arm.visual(
        Box((0.10, 0.16, 0.035)),
        origin=Origin(xyz=(1.255, 0.0, 0.152)),
        material=dark_metal,
        name="yoke_receiver",
    )
    arm.visual(
        Box((0.070, 0.012, 0.130)),
        origin=Origin(xyz=(1.325, 0.118, 0.115)),
        material=dark_metal,
        name="yoke_left_plate",
    )
    arm.visual(
        Box((0.070, 0.012, 0.130)),
        origin=Origin(xyz=(1.325, -0.118, 0.115)),
        material=dark_metal,
        name="yoke_right_plate",
    )
    arm.visual(
        Box((0.090, 0.236, 0.014)),
        origin=Origin(xyz=(1.325, 0.0, 0.175)),
        material=dark_metal,
        name="yoke_bridge",
    )
    arm.inertial = Inertial.from_geometry(
        Box((1.42, 0.28, 0.24)),
        mass=22.0,
        origin=Origin(xyz=(0.71, 0.0, 0.11)),
    )

    light = model.part("floodlight_head")
    aim_pitch = 0.42
    light.visual(
        Box((0.34, 0.20, 0.10)),
        origin=_pitched_origin((0.185, 0.0, -0.050), aim_pitch),
        material=powder_black,
        name="housing_body",
    )
    light.visual(
        Box((0.028, 0.22, 0.132)),
        origin=_pitched_origin((0.369, 0.0, -0.050), aim_pitch),
        material=powder_black,
        name="front_bezel",
    )
    light.visual(
        Box((0.006, 0.184, 0.094)),
        origin=_pitched_origin((0.386, 0.0, -0.050), aim_pitch),
        material=lens_glass,
        name="lens",
    )
    light.visual(
        Box((0.11, 0.18, 0.030)),
        origin=_pitched_origin((0.085, 0.0, 0.004), aim_pitch),
        material=powder_black,
        name="driver_box",
    )
    light.visual(
        Box((0.060, 0.22, 0.018)),
        origin=_pitched_origin((0.330, 0.0, 0.009), aim_pitch),
        material=powder_black,
        name="visor",
    )
    for index, x_center in enumerate((0.12, 0.18, 0.24)):
        light.visual(
            Box((0.024, 0.17, 0.016)),
            origin=_pitched_origin((x_center, 0.0, 0.008), aim_pitch),
            material=dark_metal,
            name=f"cooling_fin_{index}",
        )
    light.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.104, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    light.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.104, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    light.inertial = Inertial.from_geometry(
        Box((0.40, 0.24, 0.16)),
        mass=14.0,
        origin=Origin(xyz=(0.20, 0.0, -0.03)),
    )

    model.articulation(
        "base_to_pole",
        ArticulationType.FIXED,
        parent=base,
        child=pole,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
    )
    model.articulation(
        "pole_to_arm",
        ArticulationType.FIXED,
        parent=pole,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 8.20)),
    )
    model.articulation(
        "arm_to_floodlight",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=light,
        origin=Origin(xyz=(1.325, 0.0, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.70,
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

    base = object_model.get_part("concrete_base")
    pole = object_model.get_part("pole")
    arm = object_model.get_part("outreach_arm")
    light = object_model.get_part("floodlight_head")
    tilt = object_model.get_articulation("arm_to_floodlight")

    ctx.check(
        "named parts resolve",
        all(part is not None for part in (base, pole, arm, light)),
        details="One or more expected parts could not be resolved.",
    )

    with ctx.pose({tilt: 0.0}):
        ctx.expect_contact(
            pole,
            base,
            elem_a="pole_flange",
            elem_b="base_plinth",
            name="pole flange seats on concrete plinth",
        )
        ctx.expect_contact(
            arm,
            pole,
            elem_a="arm_mount_cap",
            elem_b="pole_top_cap",
            name="outreach arm cap seats on pole top",
        )
        ctx.expect_gap(
            arm,
            light,
            axis="z",
            positive_elem="yoke_bridge",
            negative_elem="housing_body",
            min_gap=0.015,
            max_gap=0.090,
            name="floodlight housing hangs below the yoke bridge",
        )
        ctx.expect_gap(
            arm,
            light,
            axis="y",
            positive_elem="yoke_left_plate",
            negative_elem="left_trunnion",
            min_gap=0.0,
            max_gap=0.010,
            name="left trunnion clears the left yoke plate",
        )
        ctx.expect_gap(
            light,
            arm,
            axis="y",
            positive_elem="right_trunnion",
            negative_elem="yoke_right_plate",
            min_gap=0.0,
            max_gap=0.010,
            name="right trunnion clears the right yoke plate",
        )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        ctx.expect_gap(
            arm,
            light,
            axis="z",
            positive_elem="yoke_bridge",
            negative_elem="housing_body",
            min_gap=0.005,
            max_gap=0.110,
            name="upward aim still clears the bridge",
        )
        lens_up = ctx.part_element_world_aabb(light, elem="lens")

    with ctx.pose({tilt: tilt.motion_limits.upper}):
        lens_down = ctx.part_element_world_aabb(light, elem="lens")

    def _z_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return 0.5 * (lower[2] + upper[2])

    z_up = _z_center(lens_up)
    z_down = _z_center(lens_down)
    ctx.check(
        "positive tilt drives the floodlight downward",
        z_up is not None and z_down is not None and z_down < z_up - 0.10,
        details=f"upward-aim lens z={z_up}, downward-aim lens z={z_down}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
