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


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_juicer")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.82, 0.90, 0.96, 0.34))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))

    body = model.part("body")
    body_shell = section_loft(
        [
            _xy_section(0.220, 0.180, 0.030, 0.000),
            _xy_section(0.236, 0.194, 0.032, 0.024),
            _xy_section(0.186, 0.152, 0.036, 0.150),
            _xy_section(0.142, 0.120, 0.026, 0.240),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.231)),
        material=trim_dark,
        name="top_collar",
    )
    body.visual(
        Box((0.032, 0.028, 0.120)),
        origin=Origin(xyz=(-0.094, 0.000, 0.206)),
        material=trim_dark,
        name="hinge_tower",
    )
    button_bezel = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.017, -0.010), (0.017, 0.010)],
        inner_profile=[(0.0105, -0.010), (0.0105, 0.010)],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(button_bezel, "button_bezel"),
        origin=Origin(xyz=(0.028, 0.089, 0.128), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_bezel",
    )
    body.visual(
        Box((0.008, 0.044, 0.032)),
        origin=Origin(xyz=(0.009, 0.072, 0.178)),
        material=steel,
        name="pivot_strut_0",
    )
    body.visual(
        Box((0.008, 0.044, 0.032)),
        origin=Origin(xyz=(0.031, 0.072, 0.178)),
        material=steel,
        name="pivot_strut_1",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.009, 0.096, 0.194), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_ear_0",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.031, 0.096, 0.194), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_ear_1",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.236, 0.194, 0.318)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.159)),
    )

    chamber = model.part("chamber")
    chamber_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.062, 0.000),
            (0.074, 0.012),
            (0.076, 0.086),
            (0.071, 0.098),
        ],
        inner_profile=[
            (0.056, 0.004),
            (0.068, 0.014),
            (0.070, 0.086),
            (0.065, 0.092),
        ],
        segments=56,
    )
    chamber.visual(
        mesh_from_geometry(chamber_shell, "chamber_shell"),
        material=clear_smoke,
        name="chamber_shell",
    )
    chamber.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile=[(0.078, 0.000), (0.078, 0.010)],
                inner_profile=[(0.067, 0.000), (0.067, 0.010)],
                segments=48,
            ),
            "chamber_base_ring",
        ),
        material=trim_dark,
        name="chamber_base_ring",
    )
    chamber.inertial = Inertial.from_geometry(
        Cylinder(radius=0.078, length=0.100),
        mass=0.5,
        origin=Origin(xyz=(0.000, 0.000, 0.050)),
    )
    model.articulation(
        "body_to_chamber",
        ArticulationType.FIXED,
        parent=body,
        child=chamber,
        origin=Origin(xyz=(0.000, 0.000, 0.240)),
    )

    lid = model.part("lid")
    lid_plate = ExtrudeWithHolesGeometry(
        _circle_profile(0.084, segments=48),
        [list(reversed(_circle_profile(0.033, segments=32)))],
        0.008,
        center=False,
    )
    lid.visual(
        mesh_from_geometry(lid_plate, "lid_plate"),
        origin=Origin(xyz=(0.076, 0.000, 0.000)),
        material=clear_smoke,
        name="lid_plate",
    )
    lid_chute = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.034, 0.000),
            (0.034, 0.090),
            (0.038, 0.120),
        ],
        inner_profile=[
            (0.028, 0.000),
            (0.028, 0.090),
            (0.032, 0.118),
        ],
        segments=48,
    )
    lid.visual(
        mesh_from_geometry(lid_chute, "lid_chute"),
        origin=Origin(xyz=(0.076, 0.000, 0.008)),
        material=clear_smoke,
        name="lid_chute",
    )
    lid.visual(
        Box((0.022, 0.160, 0.016)),
        origin=Origin(xyz=(0.005, 0.000, 0.008)),
        material=trim_dark,
        name="hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(-0.006, -0.050, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel_0",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(-0.006, 0.050, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel_1",
    )
    lid.visual(
        Box((0.022, 0.020, 0.018)),
        origin=Origin(xyz=(0.112, 0.083, 0.014)),
        material=trim_dark,
        name="latch_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.180, 0.168, 0.168)),
        mass=0.7,
        origin=Origin(xyz=(0.080, 0.000, 0.040)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.078, 0.000, 0.338)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.18,
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.026, length=0.140),
        origin=Origin(xyz=(0.000, 0.000, -0.050)),
        material=trim_dark,
        name="pusher_shaft",
    )
    pusher.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.000, 0.000, 0.013)),
        material=trim_dark,
        name="pusher_cap",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.166),
        mass=0.25,
        origin=Origin(xyz=(0.000, 0.000, -0.030)),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.076, 0.000, 0.128)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.22,
            lower=0.0,
            upper=0.090,
        ),
    )

    basket = model.part("basket")
    basket_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.000, 0.000),
            (0.048, 0.004),
            (0.056, 0.012),
            (0.057, 0.066),
            (0.060, 0.076),
        ],
        inner_profile=[
            (0.000, 0.012),
            (0.040, 0.016),
            (0.049, 0.020),
            (0.050, 0.066),
            (0.053, 0.072),
        ],
        segments=56,
    )
    basket.visual(
        mesh_from_geometry(basket_shell, "basket_shell"),
        material=steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.010, length=0.092),
        origin=Origin(xyz=(0.000, 0.000, 0.046)),
        material=trim_dark,
        name="basket_shaft",
    )
    basket.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=steel,
        name="cutter_disc",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.092),
        mass=0.45,
        origin=Origin(xyz=(0.000, 0.000, 0.046)),
    )
    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.000, 0.000, 0.248)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=40.0,
        ),
    )

    lock_arm = model.part("lock_arm")
    lock_arm.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="arm_sleeve",
    )
    lock_arm.visual(
        Box((0.014, 0.012, 0.156)),
        origin=Origin(xyz=(0.000, 0.004, 0.078)),
        material=steel,
        name="arm_spine",
    )
    lock_arm.visual(
        Box((0.020, 0.008, 0.022)),
        origin=Origin(xyz=(0.006, 0.002, 0.149)),
        material=steel,
        name="arm_head",
    )
    lock_arm.inertial = Inertial.from_geometry(
        Box((0.036, 0.030, 0.176)),
        mass=0.3,
        origin=Origin(xyz=(0.006, 0.010, 0.088)),
    )
    model.articulation(
        "body_to_lock_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_arm,
        origin=Origin(xyz=(0.020, 0.096, 0.194)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=0.0,
            upper=1.15,
        ),
    )

    button = model.part("button")
    button.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.000, 0.005, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.000, -0.002, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_stem",
    )
    button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.018),
        mass=0.05,
        origin=Origin(xyz=(0.000, 0.004, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.028, 0.099, 0.128)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
