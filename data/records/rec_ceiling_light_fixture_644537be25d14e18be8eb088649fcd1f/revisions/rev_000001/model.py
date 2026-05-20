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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 64,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
        ),
        name,
    )


def _build_pull_chain_geometry():
    pulley = CylinderGeometry(radius=0.0075, height=0.005, radial_segments=28).rotate_y(math.pi / 2.0)
    pulley.merge(
        CylinderGeometry(radius=0.0090, height=0.0012, radial_segments=28)
        .rotate_y(math.pi / 2.0)
        .translate(-0.0019, 0.0, 0.0)
    )
    pulley.merge(
        CylinderGeometry(radius=0.0090, height=0.0012, radial_segments=28)
        .rotate_y(math.pi / 2.0)
        .translate(0.0019, 0.0, 0.0)
    )
    pulley.merge(CylinderGeometry(radius=0.0020, height=0.014, radial_segments=20).rotate_y(math.pi / 2.0))

    chain_loop = tube_from_spline_points(
        [
            (0.0, -0.0045, 0.0068),
            (0.0, 0.0000, 0.0086),
            (0.0, 0.0045, 0.0068),
            (0.0, 0.0095, 0.0010),
            (0.0, 0.0100, -0.0200),
            (0.0, 0.0100, -0.0550),
            (0.0, 0.0070, -0.0950),
            (0.0, 0.0000, -0.1180),
            (0.0, -0.0070, -0.0950),
            (0.0, -0.0100, -0.0550),
            (0.0, -0.0100, -0.0200),
            (0.0, -0.0095, 0.0010),
        ],
        radius=0.00105,
        closed_spline=True,
        samples_per_segment=18,
        radial_segments=12,
        cap_ends=False,
        up_hint=(1.0, 0.0, 0.0),
    )

    pulley.merge(chain_loop)
    return pulley


def _build_chain_pull_geometry():
    pull = CylinderGeometry(radius=0.0014, height=0.010, radial_segments=12).translate(0.0, 0.0, -0.123)
    pull.merge(SphereGeometry(radius=0.0046).translate(0.0, 0.0, -0.129))
    return pull


def _build_filament_assembly_geometry():
    filament = CylinderGeometry(radius=0.0036, height=0.042, radial_segments=18).translate(0.0, 0.0, -0.050)
    filament.merge(
        CylinderGeometry(radius=0.0010, height=0.012, radial_segments=12)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, -0.071)
    )
    filament.merge(CylinderGeometry(radius=0.0008, height=0.040, radial_segments=10).translate(-0.0052, 0.0, -0.091))
    filament.merge(CylinderGeometry(radius=0.0008, height=0.040, radial_segments=10).translate(0.0052, 0.0, -0.091))
    filament.merge(
        tube_from_spline_points(
            [
                (-0.0052, 0.0, -0.111),
                (-0.0026, 0.0, -0.106),
                (0.0, 0.0, -0.1125),
                (0.0026, 0.0, -0.106),
                (0.0052, 0.0, -0.111),
            ],
            radius=0.0006,
            samples_per_segment=16,
            radial_segments=10,
            cap_ends=True,
        )
    )
    return filament


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_edison_pendant")

    brass = model.material("aged_brass", rgba=(0.63, 0.50, 0.27, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.43, 0.34, 0.18, 1.0))
    cloth_cord = model.material("cloth_cord", rgba=(0.16, 0.12, 0.09, 1.0))
    chain_finish = model.material("chain_finish", rgba=(0.49, 0.40, 0.23, 1.0))
    glass = model.material("glass_clear", rgba=(0.90, 0.90, 0.86, 0.22))
    tungsten = model.material("tungsten_glow", rgba=(0.95, 0.72, 0.33, 1.0))
    ceramic = model.material("ceramic", rgba=(0.88, 0.84, 0.76, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        _shell_mesh(
            "canopy_shell",
            [
                (0.010, 0.000),
                (0.058, -0.002),
                (0.060, -0.008),
                (0.053, -0.020),
                (0.033, -0.031),
                (0.016, -0.031),
            ],
            [
                (0.0070, -0.0008),
                (0.0545, -0.0032),
                (0.0565, -0.0082),
                (0.0490, -0.0192),
                (0.0300, -0.0286),
                (0.0118, -0.0286),
            ],
            segments=72,
        ),
        material=brass,
        name="canopy_shell",
    )
    canopy.visual(
        _shell_mesh(
            "canopy_guide",
            [(0.0138, -0.018), (0.0138, -0.040)],
            [(0.0102, -0.018), (0.0102, -0.040)],
            segments=48,
        ),
        material=dark_brass,
        name="canopy_guide",
    )
    canopy.visual(
        _shell_mesh(
            "canopy_neck",
            [(0.0160, -0.0310), (0.0110, -0.0180)],
            [(0.0118, -0.0286), (0.0066, -0.0180)],
            segments=48,
        ),
        material=dark_brass,
        name="canopy_neck",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.125, 0.125, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    cord_assembly = model.part("cord_assembly")
    cord_assembly.visual(
        Cylinder(radius=0.0036, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, -0.0120)),
        material=cloth_cord,
        name="cord_stem",
    )
    cord_assembly.visual(
        Cylinder(radius=0.0100, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.0540)),
        material=dark_brass,
        name="cord_grip_barrel",
    )
    cord_assembly.visual(
        Cylinder(radius=0.0118, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.0420)),
        material=dark_brass,
        name="cord_stop_flange",
    )
    cord_assembly.visual(
        Cylinder(radius=0.0039, length=0.478),
        origin=Origin(xyz=(0.0, 0.0, -0.3090)),
        material=cloth_cord,
        name="hanging_cord",
    )
    cord_assembly.visual(
        Cylinder(radius=0.0060, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.5570)),
        material=dark_brass,
        name="cord_ferrule",
    )
    cord_assembly.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.590)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
    )

    model.articulation(
        "canopy_to_cord_assembly",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=cord_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.06,
        ),
    )

    socket_housing = model.part("socket_housing")
    socket_housing.visual(
        _shell_mesh(
            "socket_housing_shell",
            [
                (0.006, -0.001),
                (0.012, -0.003),
                (0.018, -0.012),
                (0.022, -0.035),
                (0.025, -0.064),
                (0.025, -0.086),
                (0.021, -0.092),
            ],
            [
                (0.004, -0.001),
                (0.010, -0.004),
                (0.015, -0.012),
                (0.019, -0.035),
                (0.021, -0.064),
                (0.021, -0.087),
            ],
            segments=72,
        ),
        material=brass,
        name="housing_shell",
    )
    socket_housing.visual(
        Cylinder(radius=0.0135, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=dark_brass,
        name="top_cap",
    )
    socket_housing.visual(
        _shell_mesh(
            "switch_boss",
            [(0.0120, -0.0180), (0.0120, 0.0180)],
            [(0.0088, -0.0180), (0.0088, 0.0180)],
            segments=48,
        ),
        origin=Origin(xyz=(0.036, 0.0, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="switch_boss",
    )
    socket_housing.inertial = Inertial.from_geometry(
        Box((0.080, 0.060, 0.100)),
        mass=0.6,
        origin=Origin(xyz=(0.015, 0.0, -0.046)),
    )

    model.articulation(
        "cord_to_socket_housing",
        ArticulationType.FIXED,
        parent=cord_assembly,
        child=socket_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.565)),
    )

    edison_bulb = model.part("edison_bulb")
    edison_bulb.visual(
        Cylinder(radius=0.0135, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_brass,
        name="bulb_base",
    )
    edison_bulb.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=ceramic,
        name="bulb_contact",
    )
    edison_bulb.visual(
        _shell_mesh(
            "edison_bulb_glass",
            [
                (0.0140, -0.026),
                (0.0220, -0.040),
                (0.0410, -0.070),
                (0.0460, -0.108),
                (0.0330, -0.145),
                (0.0110, -0.170),
                (0.0045, -0.178),
            ],
            [
                (0.0122, -0.028),
                (0.0202, -0.042),
                (0.0392, -0.071),
                (0.0442, -0.108),
                (0.0312, -0.144),
                (0.0092, -0.169),
                (0.0026, -0.176),
            ],
            segments=88,
        ),
        material=glass,
        name="bulb_glass",
    )
    edison_bulb.visual(
        mesh_from_geometry(_build_filament_assembly_geometry(), "filament_assembly"),
        material=tungsten,
        name="filament_assembly",
    )
    edison_bulb.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.185)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    model.articulation(
        "housing_to_edison_bulb",
        ArticulationType.FIXED,
        parent=socket_housing,
        child=edison_bulb,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    pull_chain = model.part("pull_chain")
    pull_chain.visual(
        mesh_from_geometry(_build_pull_chain_geometry(), "pull_chain_assembly"),
        material=chain_finish,
        name="chain_assembly",
    )
    pull_chain.visual(
        mesh_from_geometry(_build_chain_pull_geometry(), "pull_chain_pull"),
        material=chain_finish,
        name="chain_pull",
    )
    pull_chain.inertial = Inertial.from_geometry(
        Box((0.025, 0.026, 0.145)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    model.articulation(
        "housing_to_pull_chain",
        ArticulationType.CONTINUOUS,
        parent=socket_housing,
        child=pull_chain,
        origin=Origin(xyz=(0.047, 0.0, -0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    cord_assembly = object_model.get_part("cord_assembly")
    socket_housing = object_model.get_part("socket_housing")
    edison_bulb = object_model.get_part("edison_bulb")
    pull_chain = object_model.get_part("pull_chain")

    cord_slide = object_model.get_articulation("canopy_to_cord_assembly")
    chain_spin = object_model.get_articulation("housing_to_pull_chain")

    ctx.check(
        "cord adjuster uses downward prismatic travel",
        cord_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 3) for v in cord_slide.axis) == (0.0, 0.0, -1.0)
        and cord_slide.motion_limits is not None
        and cord_slide.motion_limits.upper is not None
        and cord_slide.motion_limits.upper >= 0.05,
        details=f"type={cord_slide.articulation_type}, axis={cord_slide.axis}, limits={cord_slide.motion_limits}",
    )
    ctx.check(
        "pull chain uses a continuous pulley axle",
        chain_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in chain_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={chain_spin.articulation_type}, axis={chain_spin.axis}",
    )

    with ctx.pose({cord_slide: 0.0}):
        ctx.expect_gap(
            canopy,
            socket_housing,
            axis="z",
            min_gap=0.48,
            name="socket hangs clearly below the canopy at rest",
        )
        ctx.expect_overlap(
            edison_bulb,
            socket_housing,
            axes="xy",
            elem_a="bulb_base",
            elem_b="housing_shell",
            min_overlap=0.020,
            name="bulb base stays centered in the socket housing",
        )
        ctx.expect_contact(
            cord_assembly,
            canopy,
            elem_a="cord_stop_flange",
            elem_b="canopy_guide",
            name="grip flange seats against the canopy guide",
        )
        ctx.expect_contact(
            cord_assembly,
            socket_housing,
            elem_a="cord_ferrule",
            elem_b="top_cap",
            name="cord ferrule seats on the socket cap",
        )
        ctx.expect_gap(
            socket_housing,
            pull_chain,
            axis="z",
            positive_elem="switch_boss",
            negative_elem="chain_pull",
            min_gap=0.020,
            name="pull chain drops below the switch boss",
        )

    rest_pos = ctx.part_world_position(socket_housing)
    with ctx.pose({cord_slide: 0.06}):
        ctx.expect_overlap(
            cord_assembly,
            canopy,
            axes="z",
            elem_a="cord_stem",
            elem_b="canopy_guide",
            min_overlap=0.018,
            name="cord stem remains retained inside the canopy guide",
        )
        extended_pos = ctx.part_world_position(socket_housing)

    ctx.check(
        "prismatic grip lengthens the hanging drop",
        rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.055,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
