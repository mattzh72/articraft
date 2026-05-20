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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_loop(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    thickness: float,
):
    geom = ExtrudeWithHolesGeometry(
        _rect_loop(outer_width, outer_height),
        [_rect_loop(inner_width, inner_height)],
        height=thickness,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _fan_blade_mesh(name: str):
    geom = ExtrudeGeometry(
        [
            (-0.09, -0.038),
            (-0.01, -0.032),
            (0.08, -0.020),
            (0.10, 0.000),
            (0.06, 0.024),
            (-0.07, 0.035),
        ],
        height=0.010,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_wall_exhaust_fan")

    wall_concrete = model.material("wall_concrete", rgba=(0.76, 0.77, 0.75, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.63, 0.67, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    zinc_bolt = model.material("zinc_bolt", rgba=(0.82, 0.83, 0.84, 1.0))

    wall_housing = model.part("wall_housing")
    wall_housing.visual(
        _ring_mesh(
            "wall_frame_ring",
            outer_width=0.96,
            outer_height=0.96,
            inner_width=0.56,
            inner_height=0.56,
            thickness=0.18,
        ),
        material=wall_concrete,
        name="wall_frame",
    )
    wall_housing.visual(
        _ring_mesh(
            "front_bezel_ring",
            outer_width=0.64,
            outer_height=0.64,
            inner_width=0.52,
            inner_height=0.52,
            thickness=0.012,
        ),
        origin=Origin(xyz=(0.0, 0.096, 0.0)),
        material=painted_steel,
        name="front_bezel",
    )

    wall_housing.visual(
        Box((0.03, 0.15, 0.56)),
        origin=Origin(xyz=(-0.265, -0.025, 0.0)),
        material=galvanized_steel,
        name="left_sleeve_wall",
    )
    wall_housing.visual(
        Box((0.03, 0.15, 0.56)),
        origin=Origin(xyz=(0.265, -0.025, 0.0)),
        material=galvanized_steel,
        name="right_sleeve_wall",
    )
    wall_housing.visual(
        Box((0.50, 0.15, 0.03)),
        origin=Origin(xyz=(0.0, -0.025, 0.265)),
        material=galvanized_steel,
        name="top_sleeve_wall",
    )
    wall_housing.visual(
        Box((0.50, 0.15, 0.03)),
        origin=Origin(xyz=(0.0, -0.025, -0.265)),
        material=galvanized_steel,
        name="bottom_sleeve_wall",
    )
    wall_housing.visual(
        Box((0.50, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=painted_steel,
        name="rear_horizontal_brace",
    )
    wall_housing.visual(
        Box((0.032, 0.020, 0.50)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=painted_steel,
        name="rear_vertical_brace",
    )
    wall_housing.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_hub",
    )
    wall_housing.visual(
        Box((0.034, 0.020, 0.034)),
        origin=Origin(xyz=(-0.225, 0.104, 0.277)),
        material=painted_steel,
        name="left_hinge_ear",
    )
    wall_housing.visual(
        Box((0.034, 0.020, 0.034)),
        origin=Origin(xyz=(0.225, 0.104, 0.277)),
        material=painted_steel,
        name="right_hinge_ear",
    )
    wall_housing.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.295, 0.025, -0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="knob_boss",
    )

    bolt_positions = (
        (-0.304, 0.304),
        (0.304, 0.304),
        (-0.304, -0.304),
        (0.304, -0.304),
        (-0.304, 0.0),
        (0.304, 0.0),
    )
    for index, (bolt_x, bolt_z) in enumerate(bolt_positions):
        wall_housing.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(bolt_x, 0.106, bolt_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc_bolt,
            name=f"bolt_{index}",
        )

    wall_housing.inertial = Inertial.from_geometry(
        Box((0.96, 0.32, 0.96)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    damper_flap = model.part("damper_flap")
    damper_flap.visual(
        Box((0.54, 0.012, 0.52)),
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
        material=painted_steel,
        name="damper_panel",
    )
    damper_flap.visual(
        Box((0.50, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.008, -0.035)),
        material=dark_metal,
        name="top_stiffener",
    )
    damper_flap.visual(
        Box((0.50, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.008, -0.485)),
        material=dark_metal,
        name="bottom_stiffener",
    )
    damper_flap.visual(
        Box((0.020, 0.010, 0.450)),
        origin=Origin(xyz=(-0.240, -0.008, -0.26)),
        material=dark_metal,
        name="left_stiffener",
    )
    damper_flap.visual(
        Box((0.020, 0.010, 0.450)),
        origin=Origin(xyz=(0.240, -0.008, -0.26)),
        material=dark_metal,
        name="right_stiffener",
    )
    damper_flap.visual(
        Cylinder(radius=0.008, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    damper_flap.inertial = Inertial.from_geometry(
        Box((0.54, 0.05, 0.52)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
    )

    fan_blade_assembly = model.part("fan_blade_assembly")
    fan_blade_assembly.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    fan_blade_assembly.visual(
        Cylinder(radius=0.072, length=0.035),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="spinner_cap",
    )
    for blade_index in range(5):
        angle = blade_index * (2.0 * math.pi / 5.0)
        fan_blade_assembly.visual(
            _fan_blade_mesh(f"fan_blade_{blade_index}"),
            origin=Origin(
                xyz=(0.120 * math.cos(angle), 0.0, 0.120 * math.sin(angle)),
                rpy=(0.18, -angle, 0.0),
            ),
            material=galvanized_steel,
            name=f"blade_{blade_index}",
        )
    fan_blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.10),
        mass=5.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="knob_shaft",
    )
    control_knob.visual(
        Cylinder(radius=0.034, length=0.032),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    control_knob.visual(
        Box((0.006, 0.040, 0.008)),
        origin=Origin(xyz=(0.060, 0.0, 0.020)),
        material=galvanized_steel,
        name="knob_pointer",
    )
    control_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.070),
        mass=0.18,
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "housing_to_damper",
        ArticulationType.REVOLUTE,
        parent=wall_housing,
        child=damper_flap,
        origin=Origin(xyz=(0.0, 0.108, 0.260)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "housing_to_fan",
        ArticulationType.CONTINUOUS,
        parent=wall_housing,
        child=fan_blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=24.0,
        ),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=wall_housing,
        child=control_knob,
        origin=Origin(xyz=(0.310, 0.025, -0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_housing = object_model.get_part("wall_housing")
    damper_flap = object_model.get_part("damper_flap")
    fan_blade_assembly = object_model.get_part("fan_blade_assembly")
    control_knob = object_model.get_part("control_knob")

    housing_to_damper = object_model.get_articulation("housing_to_damper")
    housing_to_fan = object_model.get_articulation("housing_to_fan")
    housing_to_knob = object_model.get_articulation("housing_to_knob")

    ctx.check(
        "damper articulation is top-hinged revolute",
        housing_to_damper.articulation_type == ArticulationType.REVOLUTE
        and housing_to_damper.axis == (1.0, 0.0, 0.0),
        details=f"type={housing_to_damper.articulation_type}, axis={housing_to_damper.axis}",
    )
    ctx.check(
        "fan articulation is continuous spin about airflow axis",
        housing_to_fan.articulation_type == ArticulationType.CONTINUOUS
        and housing_to_fan.axis == (0.0, 1.0, 0.0),
        details=f"type={housing_to_fan.articulation_type}, axis={housing_to_fan.axis}",
    )
    ctx.check(
        "control knob articulation is continuous side rotation",
        housing_to_knob.articulation_type == ArticulationType.CONTINUOUS
        and housing_to_knob.axis == (1.0, 0.0, 0.0),
        details=f"type={housing_to_knob.articulation_type}, axis={housing_to_knob.axis}",
    )

    with ctx.pose({housing_to_damper: 0.0}):
        ctx.expect_gap(
            damper_flap,
            wall_housing,
            axis="y",
            max_gap=0.010,
            max_penetration=0.0005,
            positive_elem="damper_panel",
            negative_elem="front_bezel",
            name="damper closes near the exterior bezel",
        )
        ctx.expect_overlap(
            damper_flap,
            wall_housing,
            axes="xz",
            min_overlap=0.50,
            elem_a="damper_panel",
            elem_b="front_bezel",
            name="damper covers the wall opening when closed",
        )

    with ctx.pose({housing_to_damper: math.radians(65.0)}):
        ctx.expect_gap(
            damper_flap,
            wall_housing,
            axis="y",
            min_gap=0.14,
            positive_elem="bottom_stiffener",
            negative_elem="front_bezel",
            name="damper bottom edge swings outward when opened",
        )

    ctx.expect_contact(
        control_knob,
        wall_housing,
        elem_a="knob_shaft",
        elem_b="knob_boss",
        contact_tol=0.001,
        name="control knob is mounted on the side boss",
    )
    ctx.expect_origin_distance(
        fan_blade_assembly,
        wall_housing,
        axes="xz",
        max_dist=0.001,
        name="fan hub stays centered in the housing opening",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
