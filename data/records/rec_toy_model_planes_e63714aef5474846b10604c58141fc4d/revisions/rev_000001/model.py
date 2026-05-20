from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
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
            radius * math.cos((2.0 * math.pi * idx) / segments),
            radius * math.sin((2.0 * math.pi * idx) / segments),
        )
        for idx in range(segments)
    ]


def _yz_section(x_pos: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y_val, z_val) for y_val, z_val in rounded_rect_profile(width, height, radius)]


def _annular_ring(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    segments: int = 40,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        thickness,
        center=True,
    ).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_propeller_model_airplane")

    airframe_red = model.material("airframe_red", rgba=(0.78, 0.18, 0.15, 1.0))
    cream = model.material("cream", rgba=(0.95, 0.93, 0.85, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.22, 1.0))
    silver = model.material("silver", rgba=(0.78, 0.80, 0.84, 1.0))
    smoke = model.material("smoke", rgba=(0.20, 0.24, 0.28, 0.92))

    airframe = model.part("airframe")

    fuselage_geom = section_loft(
        [
            _yz_section(-0.260, 0.014, 0.020, 0.004),
            _yz_section(-0.195, 0.034, 0.048, 0.010),
            _yz_section(-0.055, 0.072, 0.092, 0.020),
            _yz_section(0.100, 0.064, 0.084, 0.018),
            _yz_section(0.195, 0.042, 0.058, 0.012),
            _yz_section(0.226, 0.028, 0.040, 0.008),
        ]
    )
    airframe.visual(
        mesh_from_geometry(fuselage_geom, "fuselage_shell"),
        material=airframe_red,
        name="fuselage_shell",
    )

    wing_geom = ExtrudeGeometry(
        [
            (-0.035, -0.010),
            (-0.108, -0.405),
            (0.020, -0.405),
            (0.142, 0.000),
            (0.020, 0.405),
            (-0.108, 0.405),
        ],
        0.012,
        center=True,
    )
    airframe.visual(
        mesh_from_geometry(wing_geom, "wing_panel"),
        origin=Origin(xyz=(-0.010, 0.0, -0.002)),
        material=cream,
        name="wing_panel",
    )

    stabilizer_geom = ExtrudeGeometry(
        [
            (-0.018, -0.010),
            (-0.070, -0.165),
            (0.012, -0.165),
            (0.060, 0.000),
            (0.012, 0.165),
            (-0.070, 0.165),
        ],
        0.008,
        center=True,
    )
    airframe.visual(
        mesh_from_geometry(stabilizer_geom, "horizontal_stabilizer"),
        origin=Origin(xyz=(-0.205, 0.0, 0.016)),
        material=cream,
        name="horizontal_stabilizer",
    )

    fin_geom = ExtrudeGeometry(
        [
            (-0.038, 0.000),
            (0.008, 0.000),
            (0.050, 0.086),
            (0.012, 0.118),
            (-0.020, 0.076),
        ],
        0.010,
        center=True,
    ).rotate_x(math.pi / 2.0)
    airframe.visual(
        mesh_from_geometry(fin_geom, "vertical_fin"),
        origin=Origin(xyz=(-0.212, 0.0, 0.014)),
        material=airframe_red,
        name="vertical_fin",
    )

    airframe.visual(
        Box((0.085, 0.034, 0.028)),
        origin=Origin(xyz=(-0.020, 0.0, 0.038)),
        material=smoke,
        name="canopy",
    )

    rear_ring_geom = _annular_ring(0.0245, 0.0205, 0.003)
    front_ring_geom = _annular_ring(0.0245, 0.0205, 0.003)
    airframe.visual(
        mesh_from_geometry(rear_ring_geom, "rear_bearing_ring"),
        origin=Origin(xyz=(0.219, 0.0, 0.0)),
        material=dark_trim,
        name="rear_bearing_ring",
    )
    airframe.visual(
        mesh_from_geometry(front_ring_geom, "front_bearing_ring"),
        origin=Origin(xyz=(0.223, 0.0, 0.0)),
        material=dark_trim,
        name="front_bearing_ring",
    )
    airframe.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(0.221, 0.0, 0.020)),
        material=dark_trim,
        name="upper_prop_cheek",
    )
    airframe.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(0.221, 0.0, -0.020)),
        material=dark_trim,
        name="lower_prop_cheek",
    )
    airframe.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(0.227, 0.0, 0.0)),
        material=dark_trim,
        name="prop_spindle",
    )

    airframe.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(xyz=(-0.015, 0.0, -0.110)),
        material=dark_trim,
        name="display_peg",
    )
    airframe.visual(
        Box((0.095, 0.032, 0.015)),
        origin=Origin(xyz=(-0.015, 0.0, -0.190)),
        material=dark_trim,
        name="display_base",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.540, 0.820, 0.240)),
        mass=0.85,
        origin=Origin(xyz=(-0.010, 0.0, -0.005)),
    )

    propeller = model.part("propeller")
    hub_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.018, segments=40),
        [[(-0.005, -0.005), (0.005, -0.005), (0.005, 0.005), (-0.005, 0.005)]],
        0.012,
        center=True,
    ).rotate_y(math.pi / 2.0)
    propeller.visual(
        mesh_from_geometry(hub_geom, "hub_shell"),
        material=dark_trim,
        name="hub_shell",
    )

    spinner_geom = ConeGeometry(radius=0.024, height=0.060, radial_segments=36).rotate_y(math.pi / 2.0)
    propeller.visual(
        mesh_from_geometry(spinner_geom, "spinner"),
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        material=silver,
        name="spinner",
    )

    propeller.visual(
        Box((0.024, 0.044, 0.010)),
        origin=Origin(xyz=(0.014, 0.028, 0.0), rpy=(0.0, 0.10, 0.0)),
        material=silver,
        name="blade_root_upper",
    )
    propeller.visual(
        Box((0.024, 0.044, 0.010)),
        origin=Origin(xyz=(0.014, -0.028, 0.0), rpy=(0.0, 0.10, 0.0)),
        material=silver,
        name="blade_root_lower",
    )
    propeller.visual(
        Box((0.030, 0.150, 0.008)),
        origin=Origin(xyz=(0.026, 0.100, 0.0), rpy=(0.0, 0.12, 0.0)),
        material=silver,
        name="blade_upper",
    )
    propeller.visual(
        Box((0.030, 0.150, 0.008)),
        origin=Origin(xyz=(0.026, -0.100, 0.0), rpy=(0.0, 0.12, 0.0)),
        material=silver,
        name="blade_lower",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.130, 0.380, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "airframe_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.233, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    spin = object_model.get_articulation("airframe_to_propeller")

    ctx.allow_overlap(
        airframe,
        propeller,
        elem_a="prop_spindle",
        elem_b="hub_shell",
        reason="The nose spindle is intentionally represented as passing through the propeller hub to keep the rotating mount mechanically legible.",
    )

    ctx.check(
        "propeller uses continuous nose-axis spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        positive_elem="hub_shell",
        negative_elem="rear_bearing_ring",
        min_gap=0.0005,
        max_gap=0.0070,
        name="hub clears rear bearing ring",
    )
    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        positive_elem="hub_shell",
        negative_elem="front_bearing_ring",
        min_gap=0.0005,
        max_gap=0.0030,
        name="hub clears front bearing ring",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        elem_a="hub_shell",
        elem_b="rear_bearing_ring",
        min_overlap=0.035,
        name="hub stays centered inside support zone",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            positive_elem="hub_shell",
            negative_elem="rear_bearing_ring",
            min_gap=0.0005,
            max_gap=0.0070,
            name="spun hub still clears rear bearing ring",
        )
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            positive_elem="hub_shell",
            negative_elem="front_bearing_ring",
            min_gap=0.0005,
            max_gap=0.0030,
            name="spun hub still clears front bearing ring",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
