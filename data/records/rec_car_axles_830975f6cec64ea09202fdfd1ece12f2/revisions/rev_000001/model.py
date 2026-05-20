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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_axle_hub_reduction_unit")

    cast_iron = model.material("cast_iron", rgba=(0.42, 0.45, 0.47, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.69, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    blackened = model.material("blackened", rgba=(0.10, 0.11, 0.12, 1.0))

    drop = 0.19

    portal_housing = model.part("portal_housing")

    body_outline = sample_catmull_rom_spline_2d(
        [
            (-0.018, 0.072),
            (-0.046, 0.046),
            (-0.050, -0.010),
            (-0.048, -0.090),
            (-0.043, -0.168),
            (-0.028, -0.248),
            (0.012, -0.282),
            (0.032, -0.252),
            (0.040, -0.188),
            (0.046, -0.110),
            (0.050, -0.022),
            (0.056, 0.046),
            (0.024, 0.078),
        ],
        samples_per_segment=10,
        closed=True,
    )
    body_mesh = mesh_from_geometry(
        ExtrudeGeometry(body_outline, 0.105, center=True).rotate_x(math.pi / 2.0),
        "portal_gearbox_body",
    )
    portal_housing.visual(body_mesh, material=cast_iron, name="gearbox_body")
    portal_housing.visual(
        Cylinder(radius=0.055, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="upper_bearing_boss",
    )
    portal_housing.visual(
        Cylinder(radius=0.075, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -drop), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="lower_output_boss",
    )
    axle_stub_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.040, -0.220), (0.040, -0.100), (0.046, -0.060)],
            [(0.028, -0.220), (0.030, -0.100), (0.034, -0.060)],
            segments=48,
        ).rotate_y(math.pi / 2.0),
        "axle_stub_tube_shell",
    )
    portal_housing.visual(
        axle_stub_mesh,
        material=dark_steel,
        name="axle_stub_tube",
    )
    portal_housing.visual(
        Cylinder(radius=0.048, length=0.032),
        origin=Origin(xyz=(-0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="axle_mount_flange",
    )
    portal_housing.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.012, 0.0, -drop), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="brake_backing_plate",
    )
    portal_housing.visual(
        Cylinder(radius=0.039, length=0.022),
        origin=Origin(xyz=(0.061, 0.0, -drop), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="spindle_nose",
    )
    portal_housing.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.012, 0.057, -0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_service_cover",
    )
    portal_housing.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.012, -0.057, -0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_service_cover",
    )
    portal_housing.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(-0.006, 0.0, 0.072)),
        material=blackened,
        name="fill_plug",
    )
    portal_housing.inertial = Inertial.from_geometry(
        Box((0.360, 0.140, 0.330)),
        mass=46.0,
        origin=Origin(xyz=(-0.050, 0.0, -0.100)),
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(-0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="input_shaft",
    )
    input_shaft.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(-0.067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="input_flange",
    )
    input_shaft.visual(
        Box((0.042, 0.010, 0.010)),
        origin=Origin(xyz=(-0.142, 0.014, 0.0)),
        material=dark_steel,
        name="input_key",
    )
    input_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.170),
        mass=4.5,
        origin=Origin(xyz=(-0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    wheel_hub = model.part("wheel_hub")
    hub_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.050, 0.055),
                (0.054, 0.070),
                (0.060, 0.086),
                (0.071, 0.098),
                (0.082, 0.108),
                (0.082, 0.114),
            ],
            [
                (0.043, 0.055),
                (0.043, 0.100),
                (0.050, 0.108),
                (0.050, 0.114),
            ],
            segments=56,
        ).rotate_y(math.pi / 2.0),
        "wheel_hub_shell",
    )
    wheel_hub.visual(hub_shell_mesh, material=dark_steel, name="hub_shell")
    wheel_hub.visual(
        Cylinder(radius=0.112, length=0.018),
        origin=Origin(xyz=(0.123, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="mounting_flange",
    )
    wheel_hub.visual(
        Cylinder(radius=0.041, length=0.024),
        origin=Origin(xyz=(0.144, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened,
        name="hub_cap",
    )
    bolt_circle_radius = 0.066
    for index in range(6):
        angle = (2.0 * math.pi * index) / 6.0
        wheel_hub.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(
                xyz=(
                    0.146,
                    bolt_circle_radius * math.cos(angle),
                    bolt_circle_radius * math.sin(angle),
                ),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name=f"wheel_stud_{index}",
        )
    wheel_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.160),
        mass=14.0,
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "input_spin",
        ArticulationType.CONTINUOUS,
        parent=portal_housing,
        child=input_shaft,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=24.0),
    )
    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=portal_housing,
        child=wheel_hub,
        origin=Origin(xyz=(0.0, 0.0, -drop)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=520.0, velocity=18.0),
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

    portal_housing = object_model.get_part("portal_housing")
    input_shaft = object_model.get_part("input_shaft")
    wheel_hub = object_model.get_part("wheel_hub")
    input_spin = object_model.get_articulation("input_spin")
    hub_spin = object_model.get_articulation("hub_spin")

    upper_boss = portal_housing.get_visual("upper_bearing_boss")
    lower_boss = portal_housing.get_visual("lower_output_boss")
    input_flange = input_shaft.get_visual("input_flange")
    hub_shell = wheel_hub.get_visual("hub_shell")

    ctx.check(
        "input shaft uses continuous axle-axis rotation",
        input_spin.articulation_type == ArticulationType.CONTINUOUS and input_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={input_spin.articulation_type}, axis={input_spin.axis}",
    )
    ctx.check(
        "wheel hub uses continuous axle-axis rotation",
        hub_spin.articulation_type == ArticulationType.CONTINUOUS and hub_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={hub_spin.articulation_type}, axis={hub_spin.axis}",
    )
    ctx.check(
        "continuous joints stay unbounded",
        input_spin.motion_limits is not None
        and hub_spin.motion_limits is not None
        and input_spin.motion_limits.lower is None
        and input_spin.motion_limits.upper is None
        and hub_spin.motion_limits.lower is None
        and hub_spin.motion_limits.upper is None,
        details=(
            f"input_limits={input_spin.motion_limits}, "
            f"hub_limits={hub_spin.motion_limits}"
        ),
    )

    ctx.expect_origin_gap(
        input_shaft,
        wheel_hub,
        axis="z",
        min_gap=0.17,
        max_gap=0.21,
        name="wheel hub is offset below the axle center",
    )
    ctx.expect_origin_distance(
        input_shaft,
        wheel_hub,
        axes="y",
        max_dist=0.001,
        name="input and output stay on the same fore-aft plane",
    )
    ctx.expect_gap(
        portal_housing,
        input_shaft,
        axis="x",
        positive_elem=upper_boss,
        negative_elem=input_flange,
        min_gap=0.0,
        max_gap=0.002,
        name="input flange seats against the upper bearing boss",
    )
    ctx.expect_gap(
        wheel_hub,
        portal_housing,
        axis="x",
        positive_elem=hub_shell,
        negative_elem=lower_boss,
        min_gap=0.0,
        max_gap=0.002,
        name="wheel hub starts just outboard of the lower output boss",
    )
    ctx.expect_overlap(
        wheel_hub,
        portal_housing,
        axes="yz",
        elem_a=hub_shell,
        elem_b=lower_boss,
        min_overlap=0.080,
        name="wheel hub stays coaxial with the lower reduction stage",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
