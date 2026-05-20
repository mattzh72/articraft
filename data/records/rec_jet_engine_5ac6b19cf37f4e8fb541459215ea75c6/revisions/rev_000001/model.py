from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _pylon_stub_geometry() -> MeshGeometry:
    """A short tapered pylon stub mounted on the top of the nacelle."""
    geom = MeshGeometry()

    x_center = 0.05
    z_bottom = 0.55
    z_top = 0.95
    x_len_bottom = 0.78
    x_len_top = 0.52
    y_width_bottom = 0.26
    y_width_top = 0.16

    xb0 = x_center - x_len_bottom / 2.0
    xb1 = x_center + x_len_bottom / 2.0
    xt0 = x_center - x_len_top / 2.0
    xt1 = x_center + x_len_top / 2.0
    yb = y_width_bottom / 2.0
    yt = y_width_top / 2.0

    vertices = [
        (xb0, -yb, z_bottom),
        (xb1, -yb, z_bottom),
        (xb1, yb, z_bottom),
        (xb0, yb, z_bottom),
        (xt0, -yt, z_top),
        (xt1, -yt, z_top),
        (xt1, yt, z_top),
        (xt0, yt, z_top),
    ]
    for vertex in vertices:
        geom.add_vertex(*vertex)

    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    for face in faces:
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_bypass_turbofan")

    nacelle_mat = model.material("warm_white_composite", rgba=(0.82, 0.84, 0.82, 1.0))
    dark_mat = model.material("dark_duct_liner", rgba=(0.015, 0.016, 0.018, 1.0))
    steel_mat = model.material("brushed_titanium", rgba=(0.56, 0.58, 0.60, 1.0))
    spinner_mat = model.material("polished_spinner", rgba=(0.78, 0.80, 0.82, 1.0))

    nacelle = model.part("nacelle")

    shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.61, -0.85),
            (0.67, -0.58),
            (0.64, -0.22),
            (0.55, 0.30),
            (0.43, 0.85),
        ],
        inner_profile=[
            (0.53, -0.85),
            (0.50, -0.58),
            (0.46, -0.22),
            (0.37, 0.30),
            (0.30, 0.85),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    nacelle.visual(
        mesh_from_geometry(shell, "nacelle_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_mat,
        name="nacelle_shell",
    )

    inlet_liner = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.505, -0.52),
            (0.472, -0.30),
        ],
        inner_profile=[
            (0.481, -0.52),
            (0.448, -0.30),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=3,
    )
    nacelle.visual(
        mesh_from_geometry(inlet_liner, "inlet_liner"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="inlet_liner",
    )

    rear_plug = LatheGeometry(
        [
            (0.0, -0.43),
            (0.16, -0.43),
            (0.23, -0.18),
            (0.20, 0.20),
            (0.13, 0.52),
            (0.035, 0.80),
            (0.0, 0.82),
        ],
        segments=72,
        closed=True,
    )
    nacelle.visual(
        mesh_from_geometry(rear_plug, "rear_plug"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="rear_plug",
    )
    nacelle.visual(
        Cylinder(radius=0.075, length=0.18),
        origin=Origin(xyz=(-0.485, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="bearing_socket",
    )

    nacelle.visual(
        mesh_from_geometry(_pylon_stub_geometry(), "pylon_stub"),
        material=nacelle_mat,
        name="pylon_stub",
    )
    nacelle.visual(
        Box((0.54, 0.34, 0.08)),
        origin=Origin(xyz=(0.05, 0.0, 0.98)),
        material=nacelle_mat,
        name="pylon_pad",
    )

    stator_inner_radius = 0.17
    stator_outer_radius = 0.56
    stator_center_radius = (stator_inner_radius + stator_outer_radius) / 2.0
    stator_length = stator_outer_radius - stator_inner_radius
    for index in range(8):
        angle = index * math.tau / 8.0
        nacelle.visual(
            Box((0.08, stator_length, 0.035)),
            origin=Origin(
                xyz=(
                    -0.30,
                    stator_center_radius * math.cos(angle),
                    stator_center_radius * math.sin(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=steel_mat,
            name=f"stator_{index}",
        )

    fan = model.part("fan")
    fan_rotor = FanRotorGeometry(
        outer_radius=0.470,
        hub_radius=0.125,
        blade_count=18,
        thickness=0.085,
        blade_pitch_deg=34.0,
        blade_sweep_deg=32.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=18.0, camber=0.12, tip_clearance=0.0),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.035, rear_collar_radius=0.115),
    )
    fan.visual(
        mesh_from_geometry(fan_rotor, "fan_disk"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="fan_disk",
    )

    spinner = LatheGeometry(
        [
            (0.0, -0.230),
            (0.045, -0.205),
            (0.105, -0.120),
            (0.138, -0.015),
            (0.135, 0.020),
            (0.0, 0.020),
        ],
        segments=72,
        closed=True,
    )
    fan.visual(
        mesh_from_geometry(spinner, "spinner"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spinner_mat,
        name="spinner",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan,
        origin=Origin(xyz=(-0.61, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("fan")
    spin = object_model.get_articulation("fan_spin")

    ctx.check(
        "front fan uses continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "spin axis follows engine centerline",
        tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.allow_overlap(
        fan,
        nacelle,
        elem_a="fan_disk",
        elem_b="bearing_socket",
        reason="The fan hub is intentionally captured on the fixed center bearing socket that supports the rotor.",
    )
    ctx.expect_within(
        nacelle,
        fan,
        axes="yz",
        inner_elem="bearing_socket",
        outer_elem="fan_disk",
        margin=0.0,
        name="bearing socket is centered inside the fan hub envelope",
    )
    ctx.expect_overlap(
        fan,
        nacelle,
        axes="x",
        elem_a="fan_disk",
        elem_b="bearing_socket",
        min_overlap=0.02,
        name="fan hub remains captured on bearing socket",
    )
    ctx.expect_origin_distance(
        fan,
        nacelle,
        axes="yz",
        max_dist=0.001,
        name="fan hub remains on nacelle centerline",
    )
    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="fan_disk",
        outer_elem="nacelle_shell",
        margin=0.0,
        name="wide fan disk is contained by nacelle outside diameter",
    )

    rest = ctx.part_world_position(fan)
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            fan,
            nacelle,
            axes="yz",
            max_dist=0.001,
            name="spinning fan stays centered in nacelle",
        )
        spun = ctx.part_world_position(fan)

    ctx.check(
        "continuous spin does not translate fan hub",
        rest is not None
        and spun is not None
        and max(abs(rest[i] - spun[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest}, spun={spun}",
    )

    return ctx.report()


object_model = build_object_model()
