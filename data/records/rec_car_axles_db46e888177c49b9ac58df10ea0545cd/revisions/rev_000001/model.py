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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lathe_x(name: str, profile):
    return _mesh(name, LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0))


def _shell_lathe_x(name: str, outer_profile, inner_profile):
    return _mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=60,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="irs_rear_half_shaft")

    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    inboard_coupling = model.part("inboard_coupling")
    inboard_coupling.visual(
        _lathe_x(
            "inboard_cup_body",
            [
                (0.0, -0.128),
                (0.055, -0.128),
                (0.055, -0.114),
                (0.031, -0.114),
                (0.031, -0.102),
                (0.050, -0.090),
                (0.050, -0.036),
                (0.042, -0.028),
                (0.036, -0.022),
                (0.0, -0.022),
            ],
        ),
        material=dark_steel,
        name="cup_body",
    )
    inboard_coupling.visual(
        Box((0.034, 0.014, 0.044)),
        origin=Origin(xyz=(-0.003, 0.027, 0.0)),
        material=dark_steel,
        name="fork_left_ear",
    )
    inboard_coupling.visual(
        Box((0.034, 0.014, 0.044)),
        origin=Origin(xyz=(-0.003, -0.027, 0.0)),
        material=dark_steel,
        name="fork_right_ear",
    )
    inboard_coupling.visual(
        Box((0.022, 0.018, 0.026)),
        origin=Origin(xyz=(-0.017, 0.024, 0.0)),
        material=dark_steel,
        name="left_fork_brace",
    )
    inboard_coupling.visual(
        Box((0.022, 0.018, 0.026)),
        origin=Origin(xyz=(-0.017, -0.024, 0.0)),
        material=dark_steel,
        name="right_fork_brace",
    )
    inboard_coupling.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(-0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="pilot_spigot",
    )
    inboard_coupling.inertial = Inertial.from_geometry(
        Box((0.165, 0.090, 0.090)),
        mass=4.8,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
    )

    inner_spider = model.part("inner_spider")
    inner_spider.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pivot_barrel",
    )
    inner_spider.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="spider_shoulder",
    )
    inner_spider.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="spider_nose",
    )
    inner_spider.visual(
        Box((0.030, 0.020, 0.020)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=machined_steel,
        name="spider_bridge",
    )
    inner_spider.inertial = Inertial.from_geometry(
        Box((0.075, 0.050, 0.050)),
        mass=1.1,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.026, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="input_sleeve",
    )
    shaft.visual(
        _shell_lathe_x(
            "inboard_boot",
            [
                (0.026, 0.000),
                (0.038, 0.012),
                (0.031, 0.022),
                (0.043, 0.036),
                (0.034, 0.049),
                (0.045, 0.064),
                (0.034, 0.078),
                (0.024, 0.092),
            ],
            [
                (0.021, 0.000),
                (0.032, 0.012),
                (0.025, 0.022),
                (0.037, 0.036),
                (0.028, 0.049),
                (0.039, 0.064),
                (0.028, 0.078),
                (0.019, 0.092),
            ],
        ),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=rubber,
        name="inboard_boot",
    )
    shaft.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="inboard_boot_inner_clamp",
    )
    shaft.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="inboard_boot_outer_clamp",
    )
    shaft.visual(
        Cylinder(radius=0.017, length=0.270),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_bar",
    )
    shaft.visual(
        Cylinder(radius=0.027, length=0.030),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="damper_band",
    )
    shaft.visual(
        Box((0.050, 0.020, 0.024)),
        origin=Origin(xyz=(0.205, 0.0, 0.029)),
        material=dark_steel,
        name="damper_weight",
    )
    shaft.visual(
        Cylinder(radius=0.023, length=0.042),
        origin=Origin(xyz=(0.379, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="outboard_collar",
    )
    shaft.visual(
        _shell_lathe_x(
            "outboard_boot",
            [
                (0.023, 0.000),
                (0.033, 0.011),
                (0.028, 0.020),
                (0.041, 0.034),
                (0.032, 0.047),
                (0.045, 0.061),
                (0.034, 0.078),
            ],
            [
                (0.018, 0.000),
                (0.028, 0.011),
                (0.023, 0.020),
                (0.036, 0.034),
                (0.027, 0.047),
                (0.039, 0.061),
                (0.028, 0.078),
            ],
        ),
        origin=Origin(xyz=(0.334, 0.0, 0.0)),
        material=rubber,
        name="outboard_boot",
    )
    shaft.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.344, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="outboard_boot_inner_clamp",
    )
    shaft.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.407, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="outboard_boot_outer_clamp",
    )
    shaft.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.406, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="outboard_stub",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.420),
        mass=6.2,
        origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    wheel_hub = model.part("wheel_hub")
    wheel_hub.visual(
        _lathe_x(
            "outer_cv_bell",
            [
                (0.0, 0.000),
                (0.032, 0.000),
                (0.041, 0.010),
                (0.048, 0.026),
                (0.046, 0.040),
                (0.040, 0.055),
                (0.0, 0.055),
            ],
        ),
        material=dark_steel,
        name="outer_cv_bell",
    )
    wheel_hub.visual(
        Cylinder(radius=0.040, length=0.082),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="bearing_barrel",
    )
    wheel_hub.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.133, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub_flange",
    )
    wheel_hub.visual(
        Cylinder(radius=0.022, length=0.052),
        origin=Origin(xyz=(0.161, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="stub_axle",
    )
    wheel_hub.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_nut",
    )
    for stud_index in range(5):
        angle = stud_index * (2.0 * math.pi / 5.0)
        wheel_hub.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(
                xyz=(0.145, 0.044 * math.cos(angle), 0.044 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"lug_stud_{stud_index}",
        )
    wheel_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.190),
        mass=5.0,
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "inboard_pitch",
        ArticulationType.REVOLUTE,
        parent=inboard_coupling,
        child=inner_spider,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=2.5,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=inner_spider,
        child=shaft,
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=40.0),
    )
    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=wheel_hub,
        origin=Origin(xyz=(0.412, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=40.0),
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

    inboard_coupling = object_model.get_part("inboard_coupling")
    inner_spider = object_model.get_part("inner_spider")
    shaft = object_model.get_part("shaft")
    wheel_hub = object_model.get_part("wheel_hub")

    inboard_pitch = object_model.get_articulation("inboard_pitch")
    shaft_spin = object_model.get_articulation("shaft_spin")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "prompt parts exist",
        all(part is not None for part in (inboard_coupling, inner_spider, shaft, wheel_hub)),
        details="Expected inboard coupling, inner spider, shaft, and wheel hub parts.",
    )

    pitch_limits = inboard_pitch.motion_limits
    ctx.check(
        "inboard coupling is a revolute pitch joint",
        inboard_pitch.articulation_type == ArticulationType.REVOLUTE
        and inboard_pitch.axis == (0.0, -1.0, 0.0)
        and pitch_limits is not None
        and pitch_limits.lower == -0.45
        and pitch_limits.upper == 0.45,
        details=(
            f"type={inboard_pitch.articulation_type}, axis={inboard_pitch.axis}, "
            f"limits={pitch_limits}"
        ),
    )
    ctx.check(
        "shaft rotates continuously about its own axis",
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and shaft_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={shaft_spin.articulation_type}, axis={shaft_spin.axis}",
    )
    ctx.check(
        "wheel hub rotates continuously at the outboard end",
        hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and hub_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={hub_spin.articulation_type}, axis={hub_spin.axis}",
    )

    ctx.expect_contact(
        inner_spider,
        inboard_coupling,
        elem_a="pivot_barrel",
        elem_b="fork_left_ear",
        contact_tol=5e-4,
        name="left fork ear supports the pivot barrel",
    )
    ctx.expect_contact(
        inner_spider,
        inboard_coupling,
        elem_a="pivot_barrel",
        elem_b="fork_right_ear",
        contact_tol=5e-4,
        name="right fork ear supports the pivot barrel",
    )
    ctx.expect_contact(
        shaft,
        inner_spider,
        elem_a="input_sleeve",
        elem_b="spider_nose",
        contact_tol=5e-4,
        name="shaft seats on the inner spider nose",
    )
    ctx.expect_contact(
        wheel_hub,
        shaft,
        elem_a="outer_cv_bell",
        elem_b="outboard_stub",
        contact_tol=5e-4,
        name="outboard CV bell meets the shaft stub",
    )
    ctx.expect_overlap(
        wheel_hub,
        shaft,
        axes="yz",
        elem_a="outer_cv_bell",
        elem_b="outboard_stub",
        min_overlap=0.035,
        name="outboard CV stays coaxial with the shaft",
    )

    hub_rest = ctx.part_world_position(wheel_hub)
    with ctx.pose({inboard_pitch: 0.30}):
        hub_pitched = ctx.part_world_position(wheel_hub)

    ctx.check(
        "positive inboard pitch raises the outboard end",
        hub_rest is not None
        and hub_pitched is not None
        and hub_pitched[2] > hub_rest[2] + 0.10,
        details=f"rest={hub_rest}, pitched={hub_pitched}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
