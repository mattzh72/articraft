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
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capsule_candy_vending_machine")

    body_red = model.material("body_red", rgba=(0.72, 0.10, 0.10, 1.0))
    trim_red = model.material("trim_red", rgba=(0.60, 0.08, 0.08, 1.0))
    cream = model.material("cream", rgba=(0.93, 0.90, 0.82, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.74, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.15, 0.18, 0.22, 0.45))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.80, 0.90, 0.96, 0.28))

    body = model.part("machine_body")
    body.visual(
        Box((0.28, 0.26, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_gray,
        name="base_plinth",
    )
    body.visual(
        Box((0.22, 0.20, 0.012)),
        origin=Origin(xyz=(0.0, -0.094, 0.195)),
        material=cream,
        name="rear_wall",
    )
    body.visual(
        Box((0.012, 0.20, 0.33)),
        origin=Origin(xyz=(-0.114, 0.0, 0.195)),
        material=cream,
        name="left_side_wall",
    )
    body.visual(
        Box((0.012, 0.20, 0.33)),
        origin=Origin(xyz=(0.114, 0.0, 0.195)),
        material=cream,
        name="right_side_wall",
    )
    body.visual(
        Box((0.216, 0.188, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=cream,
        name="cabinet_floor",
    )
    body.visual(
        Box((0.216, 0.188, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.354)),
        material=cream,
        name="cabinet_roof",
    )
    body.visual(
        Box((0.216, 0.012, 0.170)),
        origin=Origin(xyz=(0.0, 0.094, 0.275)),
        material=cream,
        name="front_fascia",
    )
    body.visual(
        Box((0.024, 0.012, 0.145)),
        origin=Origin(xyz=(-0.096, 0.094, 0.1125)),
        material=cream,
        name="left_jamb",
    )
    body.visual(
        Box((0.024, 0.012, 0.145)),
        origin=Origin(xyz=(0.096, 0.094, 0.1125)),
        material=cream,
        name="right_jamb",
    )
    body.visual(
        Box((0.216, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.094, 0.045)),
        material=cream,
        name="front_sill",
    )
    body.visual(
        Box((0.26, 0.22, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.3525)),
        material=trim_red,
        name="top_cap",
    )
    body.visual(
        Box((0.13, 0.13, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.374)),
        material=trim_red,
        name="hopper_collar",
    )
    body.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.400), rpy=(0.0, 0.0, 0.0)),
        material=trim_red,
        name="hopper_neck",
    )
    hopper_shell = LatheGeometry.from_shell_profiles(
        [
            (0.064, 0.000),
            (0.096, 0.030),
            (0.123, 0.085),
            (0.136, 0.150),
            (0.132, 0.205),
            (0.108, 0.245),
            (0.072, 0.270),
            (0.046, 0.282),
        ],
        [
            (0.056, 0.004),
            (0.088, 0.034),
            (0.115, 0.087),
            (0.128, 0.150),
            (0.124, 0.202),
            (0.102, 0.241),
            (0.069, 0.264),
            (0.040, 0.274),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        _mesh("hopper_shell", hopper_shell),
        origin=Origin(xyz=(0.0, 0.0, 0.402)),
        material=clear_acrylic,
        name="hopper_shell",
    )
    body.visual(
        Cylinder(radius=0.075, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.684)),
        material=trim_red,
        name="hopper_lid",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.704)),
        material=steel,
        name="lid_knob",
    )
    body.visual(
        Box((0.11, 0.03, 0.025)),
        origin=Origin(xyz=(0.0, 0.115, 0.245)),
        material=trim_red,
        name="wheel_backplate",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(xyz=(0.0, 0.122, 0.245), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_red,
        name="wheel_mount_boss",
    )
    body.visual(
        Box((0.092, 0.038, 0.022)),
        origin=Origin(xyz=(0.0, 0.112, 0.207)),
        material=steel,
        name="chute_bezel",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.28, 0.26, 0.70)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    wheel = model.part("dispensing_wheel")
    wheel.visual(_mesh("wheel_rim", TorusGeometry(radius=0.041, tube=0.007).rotate_x(math.pi / 2.0)), material=steel, name="wheel_rim")
    wheel.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="wheel_hub",
    )
    wheel.visual(
        Box((0.046, 0.008, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=dark_gray,
        name="wheel_spoke_0",
    )
    wheel.visual(
        Box((0.046, 0.008, 0.010)),
        origin=Origin(xyz=(0.020 * math.cos(2.0 * math.pi / 3.0), 0.0, -0.020 * math.sin(2.0 * math.pi / 3.0)), rpy=(0.0, 2.0 * math.pi / 3.0, 0.0)),
        material=dark_gray,
        name="wheel_spoke_1",
    )
    wheel.visual(
        Box((0.046, 0.008, 0.010)),
        origin=Origin(xyz=(0.020 * math.cos(4.0 * math.pi / 3.0), 0.0, -0.020 * math.sin(4.0 * math.pi / 3.0)), rpy=(0.0, 4.0 * math.pi / 3.0, 0.0)),
        material=dark_gray,
        name="wheel_spoke_2",
    )
    wheel.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.037, 0.010, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wheel_knob_stem",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.037, 0.022, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_red,
        name="wheel_knob",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.022),
        mass=0.6,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.18, 0.012, 0.16)),
        origin=Origin(xyz=(-0.09, 0.006, -0.08)),
        material=body_red,
        name="panel_door",
    )
    service_panel.visual(
        Box((0.14, 0.004, 0.12)),
        origin=Origin(xyz=(-0.09, 0.010, -0.07)),
        material=cream,
        name="panel_inset",
    )
    service_panel.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(-0.155, 0.018, -0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="panel_lock",
    )
    service_panel.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, 0.006, -0.020)),
        material=steel,
        name="hinge_barrel_top",
    )
    service_panel.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=(0.0, 0.006, -0.080)),
        material=steel,
        name="hinge_barrel_mid",
    )
    service_panel.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, 0.006, -0.140)),
        material=steel,
        name="hinge_barrel_bottom",
    )
    service_panel.inertial = Inertial.from_geometry(
        Box((0.18, 0.02, 0.16)),
        mass=1.2,
        origin=Origin(xyz=(-0.09, 0.01, -0.08)),
    )

    flap = model.part("retrieval_flap")
    flap.visual(
        Box((0.10, 0.008, 0.055)),
        origin=Origin(xyz=(0.0, 0.008, -0.0275)),
        material=smoked_clear,
        name="flap_door",
    )
    flap.visual(
        Cylinder(radius=0.004, length=0.108),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.070, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.011, -0.051)),
        material=dark_gray,
        name="flap_pull_lip",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.10, 0.012, 0.055)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.006, -0.0275)),
    )

    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.138, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )
    model.articulation(
        "body_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_panel,
        origin=Origin(xyz=(0.09, 0.100, 0.190)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.8),
    )
    model.articulation(
        "panel_to_flap",
        ArticulationType.REVOLUTE,
        parent=service_panel,
        child=flap,
        origin=Origin(xyz=(-0.09, 0.012, -0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.2),
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
    body = object_model.get_part("machine_body")
    wheel = object_model.get_part("dispensing_wheel")
    service_panel = object_model.get_part("service_panel")
    flap = object_model.get_part("retrieval_flap")
    wheel_joint = object_model.get_articulation("body_to_wheel")
    panel_joint = object_model.get_articulation("body_to_service_panel")
    flap_joint = object_model.get_articulation("panel_to_flap")

    ctx.check("body exists", body is not None)
    ctx.check("wheel exists", wheel is not None)
    ctx.check("service panel exists", service_panel is not None)
    ctx.check("retrieval flap exists", flap is not None)

    ctx.check(
        "wheel is continuous on horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "service panel uses vertical hinge",
        panel_joint.articulation_type == ArticulationType.REVOLUTE and tuple(panel_joint.axis) == (0.0, 0.0, -1.0),
        details=f"type={panel_joint.articulation_type}, axis={panel_joint.axis}",
    )
    ctx.check(
        "flap uses horizontal hinge",
        flap_joint.articulation_type == ArticulationType.REVOLUTE and tuple(flap_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={flap_joint.articulation_type}, axis={flap_joint.axis}",
    )

    with ctx.pose({panel_joint: 0.0, flap_joint: 0.0}):
        ctx.expect_contact(
            wheel,
            body,
            elem_a="wheel_hub",
            elem_b="wheel_mount_boss",
            name="wheel hub seats on mount boss",
        )
        ctx.expect_contact(
            service_panel,
            body,
            elem_a="panel_door",
            elem_b="front_fascia",
            name="service panel closes under fascia",
        )
        ctx.expect_gap(
            wheel,
            service_panel,
            axis="z",
            min_gap=0.003,
            name="wheel stays above service panel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
