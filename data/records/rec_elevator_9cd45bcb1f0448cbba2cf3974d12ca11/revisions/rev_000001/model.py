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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _rpy_for_box_x(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    horizontal = math.hypot(dx, dy)
    pitch = -math.atan2(dz, horizontal)
    return (0.0, pitch, yaw)


def _add_bar(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    thickness: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((_distance(a, b), width, thickness)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_box_x(a, b)),
        material=material,
        name=name,
    )


def _add_primary_arm_geometry(part, material, pin_material) -> None:
    start = (0.0, -0.016, 0.0)
    end = (0.66, -0.016, 0.31)
    center = (0.33, -0.016, 0.155)
    _add_beam(
        part,
        start,
        end,
        width=0.020,
        thickness=0.038,
        material=material,
        name="arm_web",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="center_pivot_half",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="base_pivot_boss",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.66, -0.016, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="upper_slider_roller",
    )


def _add_secondary_arm_geometry(part, material, pin_material) -> None:
    start = (-0.33, 0.016, 0.155)
    end = (0.33, 0.016, -0.155)
    _add_beam(
        part,
        start,
        end,
        width=0.020,
        thickness=0.038,
        material=material,
        name="arm_web",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="center_pivot_half",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(-0.33, 0.016, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="upper_fixed_roller",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.33, 0.016, -0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="lower_slider_roller",
    )


def _add_base_slider_geometry(part, material, shoe_material) -> None:
    cavity_y = 0.016
    part.visual(
        Box((0.14, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, cavity_y - 0.017, -0.012)),
        material=material,
        name="left_cheek",
    )
    part.visual(
        Box((0.14, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, cavity_y + 0.017, -0.012)),
        material=material,
        name="right_cheek",
    )
    part.visual(
        Box((0.11, 0.044, 0.016)),
        origin=Origin(xyz=(0.0, cavity_y, -0.032)),
        material=material,
        name="bridge_plate",
    )
    part.visual(
        Box((0.11, 0.042, 0.020)),
        origin=Origin(xyz=(0.0, cavity_y, -0.05)),
        material=shoe_material,
        name="track_shoe",
    )


def _add_car_slider_geometry(part, material, shoe_material) -> None:
    cavity_y = -0.016
    part.visual(
        Box((0.14, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, cavity_y - 0.017, 0.012)),
        material=material,
        name="left_cheek",
    )
    part.visual(
        Box((0.14, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, cavity_y + 0.017, 0.012)),
        material=material,
        name="right_cheek",
    )
    part.visual(
        Box((0.11, 0.044, 0.016)),
        origin=Origin(xyz=(0.0, cavity_y, 0.032)),
        material=material,
        name="bridge_plate",
    )
    part.visual(
        Box((0.11, 0.042, 0.020)),
        origin=Origin(xyz=(0.0, cavity_y, 0.05)),
        material=shoe_material,
        name="track_shoe",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_passenger_elevator")

    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.65, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.24, 0.26, 0.28, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.39, 0.42, 0.45, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    cabin_floor = model.material("cabin_floor", rgba=(0.16, 0.17, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.84, 0.92, 0.28))
    scissor_paint = model.material("scissor_paint", rgba=(0.83, 0.69, 0.16, 1.0))
    cylinder_black = model.material("cylinder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.88, 0.89, 0.90, 1.0))
    slider_paint = model.material("slider_paint", rgba=(0.32, 0.34, 0.37, 1.0))

    pit_frame = model.part("pit_frame")
    pit_frame.visual(
        Box((2.10, 1.85, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="pit_slab",
    )
    pit_frame.visual(
        Box((0.12, 1.85, 1.02)),
        origin=Origin(xyz=(-0.99, 0.0, 0.69)),
        material=concrete,
        name="left_pit_wall",
    )
    pit_frame.visual(
        Box((0.12, 1.85, 1.02)),
        origin=Origin(xyz=(0.99, 0.0, 0.69)),
        material=concrete,
        name="right_pit_wall",
    )
    pit_frame.visual(
        Box((1.86, 0.12, 3.20)),
        origin=Origin(xyz=(0.0, -0.865, 1.78)),
        material=concrete,
        name="rear_pit_wall",
    )
    pit_frame.visual(
        Box((1.86, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.835, 0.24)),
        material=concrete,
        name="landing_sill",
    )
    for side_x in (-0.54, 0.54):
        pit_frame.visual(
            Box((0.03, 0.12, 2.92)),
            origin=Origin(xyz=(side_x, -0.80, 1.64)),
            material=rail_steel,
            name=f"guide_web_{'left' if side_x < 0.0 else 'right'}",
        )
        pit_frame.visual(
            Box((0.12, 0.03, 2.92)),
            origin=Origin(xyz=(side_x, -0.695, 1.64)),
            material=rail_steel,
            name=f"guide_flange_{'left' if side_x < 0.0 else 'right'}",
        )

    for suffix, y_pos in (("front", 0.34), ("rear", -0.34)):
        pit_frame.visual(
            Box((1.04, 0.10, 0.05)),
            origin=Origin(xyz=(0.02, y_pos, 0.205)),
            material=shaft_steel,
            name=f"{suffix}_track_pedestal",
        )
        pit_frame.visual(
            Box((1.04, 0.10, 0.02)),
            origin=Origin(xyz=(0.02, y_pos, 0.24)),
            material=shaft_steel,
            name=f"{suffix}_track_base",
        )
        pit_frame.visual(
            Box((1.04, 0.014, 0.08)),
            origin=Origin(xyz=(0.02, y_pos - 0.043, 0.28)),
            material=shaft_steel,
            name=f"{suffix}_track_inner_rail",
        )
        pit_frame.visual(
            Box((1.04, 0.014, 0.08)),
            origin=Origin(xyz=(0.02, y_pos + 0.043, 0.28)),
            material=shaft_steel,
            name=f"{suffix}_track_outer_rail",
        )
        pit_frame.visual(
            Box((0.16, 0.010, 0.12)),
            origin=Origin(xyz=(-0.42, y_pos - 0.033, 0.31)),
            material=shaft_steel,
            name=f"{suffix}_base_pivot_bracket_inner",
        )
        pit_frame.visual(
            Box((0.16, 0.010, 0.12)),
            origin=Origin(xyz=(-0.42, y_pos + 0.001, 0.31)),
            material=shaft_steel,
            name=f"{suffix}_base_pivot_bracket_outer",
        )
        pit_frame.visual(
            Box((0.16, 0.044, 0.012)),
            origin=Origin(xyz=(-0.42, y_pos - 0.016, 0.376)),
            material=shaft_steel,
            name=f"{suffix}_base_pivot_bracket_cap",
        )

    pit_frame.visual(
        Box((0.34, 0.54, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=shaft_steel,
        name="cylinder_plinth",
    )
    pit_frame.inertial = Inertial.from_geometry(
        Box((2.10, 1.85, 3.20)),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
    )

    car = model.part("car")
    car.visual(
        Box((1.44, 1.34, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cabin_floor,
        name="floor_pan",
    )
    car.visual(
        Box((0.05, 1.34, 2.15)),
        origin=Origin(xyz=(-0.695, 0.0, 1.145)),
        material=stainless,
        name="left_side_wall",
    )
    car.visual(
        Box((0.05, 1.34, 2.15)),
        origin=Origin(xyz=(0.695, 0.0, 1.145)),
        material=stainless,
        name="right_side_wall",
    )
    car.visual(
        Box((1.34, 0.05, 2.15)),
        origin=Origin(xyz=(0.0, -0.645, 1.145)),
        material=stainless,
        name="rear_wall",
    )
    car.visual(
        Box((0.07, 0.08, 2.05)),
        origin=Origin(xyz=(-0.64, 0.63, 1.095)),
        material=stainless,
        name="left_door_jamb",
    )
    car.visual(
        Box((0.07, 0.08, 2.05)),
        origin=Origin(xyz=(0.64, 0.63, 1.095)),
        material=stainless,
        name="right_door_jamb",
    )
    car.visual(
        Box((1.28, 0.08, 0.16)),
        origin=Origin(xyz=(0.0, 0.63, 2.14)),
        material=stainless,
        name="door_header",
    )
    car.visual(
        Box((1.44, 1.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        material=stainless,
        name="roof_panel",
    )
    car.visual(
        Box((0.34, 0.18, 0.03)),
        origin=Origin(xyz=(0.0, -0.53, 1.20)),
        material=glass,
        name="indicator_panel",
    )
    car.visual(
        Box((0.18, 0.10, 0.10)),
        origin=Origin(xyz=(0.58, 0.46, 1.16)),
        material=shaft_steel,
        name="cop_box",
    )
    car.visual(
        Box((1.20, 0.05, 0.16)),
        origin=Origin(xyz=(0.0, 0.665, 0.08)),
        material=stainless,
        name="toe_guard",
    )
    for y_pos in (0.34, -0.34):
        car.visual(
            Box((0.52, 0.12, 0.05)),
            origin=Origin(xyz=(-0.36, y_pos, -0.015)),
            material=shaft_steel,
            name=f"underfloor_crossbeam_{'front' if y_pos > 0.0 else 'rear'}",
        )
        car.visual(
            Box((0.44, 0.10, 0.02)),
            origin=Origin(xyz=(0.05, y_pos, -0.03)),
            material=shaft_steel,
            name=f"upper_track_base_{'front' if y_pos > 0.0 else 'rear'}",
        )
        car.visual(
            Box((0.44, 0.014, 0.06)),
            origin=Origin(xyz=(0.05, y_pos - 0.043, -0.06)),
            material=shaft_steel,
            name=f"upper_track_inner_rail_{'front' if y_pos > 0.0 else 'rear'}",
        )
        car.visual(
            Box((0.44, 0.014, 0.06)),
            origin=Origin(xyz=(0.05, y_pos + 0.043, -0.06)),
            material=shaft_steel,
            name=f"upper_track_outer_rail_{'front' if y_pos > 0.0 else 'rear'}",
        )
        car.visual(
            Box((0.13, 0.010, 0.056)),
            origin=Origin(xyz=(-0.42, y_pos - 0.001, -0.048)),
            material=shaft_steel,
            name=f"upper_fixed_bracket_inner_{'front' if y_pos > 0.0 else 'rear'}",
        )
        car.visual(
            Box((0.13, 0.010, 0.056)),
            origin=Origin(xyz=(-0.42, y_pos + 0.033, -0.048)),
            material=shaft_steel,
            name=f"upper_fixed_bracket_outer_{'front' if y_pos > 0.0 else 'rear'}",
        )
        car.visual(
            Box((0.13, 0.044, 0.012)),
            origin=Origin(xyz=(-0.42, y_pos + 0.016, -0.014)),
            material=shaft_steel,
            name=f"upper_fixed_bracket_cap_{'front' if y_pos > 0.0 else 'rear'}",
        )
    for side_x in (-0.54, 0.54):
        for level, z_pos in (("lower", 0.62), ("upper", 1.68)):
            car.visual(
                Box((0.10, 0.048, 0.12)),
                origin=Origin(xyz=(side_x, -0.656, z_pos)),
                material=rail_steel,
                name=f"{level}_guide_shoe_{'left' if side_x < 0.0 else 'right'}",
            )
    car.visual(
        Box((0.28, 0.30, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=shaft_steel,
        name="ram_pad",
    )
    car.inertial = Inertial.from_geometry(
        Box((1.44, 1.34, 2.31)),
        mass=980.0,
        origin=Origin(xyz=(0.0, 0.0, 1.155)),
    )

    cylinder_barrel = model.part("cylinder_barrel")
    cylinder_barrel.visual(
        Box((0.24, 0.34, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=shaft_steel,
        name="anchor_shoe",
    )
    cylinder_barrel.visual(
        Cylinder(radius=0.11, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=cylinder_black,
        name="barrel_shell",
    )
    cylinder_barrel.visual(
        Cylinder(radius=0.125, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=shaft_steel,
        name="gland_head",
    )
    cylinder_barrel.inertial = Inertial.from_geometry(
        Box((0.24, 0.34, 0.34)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    cylinder_rod = model.part("cylinder_rod")
    cylinder_rod.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=polished_steel,
        name="rod",
    )
    cylinder_rod.visual(
        Box((0.20, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=polished_steel,
        name="ram_head",
    )
    cylinder_rod.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.09)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    front_base_slider = model.part("front_base_slider")
    _add_base_slider_geometry(front_base_slider, slider_paint, polished_steel)
    front_base_slider.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.09)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    rear_base_slider = model.part("rear_base_slider")
    _add_base_slider_geometry(rear_base_slider, slider_paint, polished_steel)
    rear_base_slider.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.09)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    front_car_slider = model.part("front_car_slider")
    _add_car_slider_geometry(front_car_slider, slider_paint, polished_steel)
    front_car_slider.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.09)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    rear_car_slider = model.part("rear_car_slider")
    _add_car_slider_geometry(rear_car_slider, slider_paint, polished_steel)
    rear_car_slider.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.09)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    front_primary_arm = model.part("front_primary_arm")
    _add_primary_arm_geometry(front_primary_arm, scissor_paint, polished_steel)
    front_primary_arm.inertial = Inertial.from_geometry(
        Box((0.70, 0.08, 0.12)),
        mass=56.0,
        origin=Origin(xyz=(0.33, -0.016, 0.155)),
    )

    rear_primary_arm = model.part("rear_primary_arm")
    _add_primary_arm_geometry(rear_primary_arm, scissor_paint, polished_steel)
    rear_primary_arm.inertial = Inertial.from_geometry(
        Box((0.70, 0.08, 0.12)),
        mass=56.0,
        origin=Origin(xyz=(0.33, -0.016, 0.155)),
    )

    front_secondary_arm = model.part("front_secondary_arm")
    _add_secondary_arm_geometry(front_secondary_arm, scissor_paint, polished_steel)
    front_secondary_arm.inertial = Inertial.from_geometry(
        Box((0.70, 0.08, 0.12)),
        mass=56.0,
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
    )

    rear_secondary_arm = model.part("rear_secondary_arm")
    _add_secondary_arm_geometry(rear_secondary_arm, scissor_paint, polished_steel)
    rear_secondary_arm.inertial = Inertial.from_geometry(
        Box((0.70, 0.08, 0.12)),
        mass=56.0,
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
    )

    model.articulation(
        "pit_to_car",
        ArticulationType.PRISMATIC,
        parent=pit_frame,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45000.0, velocity=0.35, lower=0.0, upper=0.34),
    )
    model.articulation(
        "pit_to_cylinder_barrel",
        ArticulationType.FIXED,
        parent=pit_frame,
        child=cylinder_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )
    model.articulation(
        "cylinder_stroke",
        ArticulationType.PRISMATIC,
        parent=cylinder_barrel,
        child=cylinder_rod,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45000.0, velocity=0.35, lower=0.0, upper=0.33),
    )
    model.articulation(
        "front_base_slider_travel",
        ArticulationType.PRISMATIC,
        parent=pit_frame,
        child=front_base_slider,
        origin=Origin(xyz=(0.24, 0.34, 0.31)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.30, lower=0.0, upper=0.32),
    )
    model.articulation(
        "rear_base_slider_travel",
        ArticulationType.PRISMATIC,
        parent=pit_frame,
        child=rear_base_slider,
        origin=Origin(xyz=(0.24, -0.34, 0.31)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.30, lower=0.0, upper=0.32),
    )
    model.articulation(
        "front_car_slider_travel",
        ArticulationType.PRISMATIC,
        parent=car,
        child=front_car_slider,
        origin=Origin(xyz=(0.24, 0.34, -0.10)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.30, lower=0.0, upper=0.32),
    )
    model.articulation(
        "rear_car_slider_travel",
        ArticulationType.PRISMATIC,
        parent=car,
        child=rear_car_slider,
        origin=Origin(xyz=(0.24, -0.34, -0.10)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.30, lower=0.0, upper=0.32),
    )
    model.articulation(
        "front_primary_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=pit_frame,
        child=front_primary_arm,
        origin=Origin(xyz=(-0.42, 0.34, 0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=1.2, lower=-0.80, upper=0.15),
    )
    model.articulation(
        "rear_primary_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=pit_frame,
        child=rear_primary_arm,
        origin=Origin(xyz=(-0.42, -0.34, 0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=1.2, lower=-0.80, upper=0.15),
    )
    model.articulation(
        "front_center_pivot",
        ArticulationType.REVOLUTE,
        parent=front_primary_arm,
        child=front_secondary_arm,
        origin=Origin(xyz=(0.33, 0.0, 0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=1.2, lower=-0.15, upper=0.80),
    )
    model.articulation(
        "rear_center_pivot",
        ArticulationType.REVOLUTE,
        parent=rear_primary_arm,
        child=rear_secondary_arm,
        origin=Origin(xyz=(0.33, 0.0, 0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=1.2, lower=-0.15, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pit_frame = object_model.get_part("pit_frame")
    car = object_model.get_part("car")
    cylinder_barrel = object_model.get_part("cylinder_barrel")
    cylinder_rod = object_model.get_part("cylinder_rod")
    front_base_slider = object_model.get_part("front_base_slider")
    rear_base_slider = object_model.get_part("rear_base_slider")
    front_car_slider = object_model.get_part("front_car_slider")
    rear_car_slider = object_model.get_part("rear_car_slider")
    front_primary_arm = object_model.get_part("front_primary_arm")
    rear_primary_arm = object_model.get_part("rear_primary_arm")
    front_secondary_arm = object_model.get_part("front_secondary_arm")
    rear_secondary_arm = object_model.get_part("rear_secondary_arm")

    pit_to_car = object_model.get_articulation("pit_to_car")
    cylinder_stroke = object_model.get_articulation("cylinder_stroke")
    front_base_slider_travel = object_model.get_articulation("front_base_slider_travel")
    rear_base_slider_travel = object_model.get_articulation("rear_base_slider_travel")
    front_car_slider_travel = object_model.get_articulation("front_car_slider_travel")
    rear_car_slider_travel = object_model.get_articulation("rear_car_slider_travel")
    front_primary_arm_pivot = object_model.get_articulation("front_primary_arm_pivot")
    rear_primary_arm_pivot = object_model.get_articulation("rear_primary_arm_pivot")
    front_center_pivot = object_model.get_articulation("front_center_pivot")
    rear_center_pivot = object_model.get_articulation("rear_center_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        cylinder_barrel,
        cylinder_rod,
        reason="The hydraulic ram rod telescopes inside the barrel gland by design.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(car, pit_frame, name="car_guides_contact_rails")
    ctx.expect_contact(cylinder_barrel, pit_frame, name="barrel_seated_on_plinth")
    ctx.expect_contact(cylinder_rod, car, name="rod_contacts_ram_pad")
    ctx.expect_contact(front_base_slider, pit_frame, name="front_base_slider_on_track")
    ctx.expect_contact(rear_base_slider, pit_frame, name="rear_base_slider_on_track")
    ctx.expect_contact(front_car_slider, car, name="front_car_slider_in_track")
    ctx.expect_contact(rear_car_slider, car, name="rear_car_slider_in_track")
    ctx.expect_contact(front_primary_arm, front_secondary_arm, name="front_scissor_center_contact")
    ctx.expect_contact(rear_primary_arm, rear_secondary_arm, name="rear_scissor_center_contact")

    car_rest = ctx.part_world_position(car)
    rod_rest = ctx.part_world_position(cylinder_rod)
    front_base_rest = ctx.part_world_position(front_base_slider)
    front_car_rest = ctx.part_world_position(front_car_slider)
    assert car_rest is not None
    assert rod_rest is not None
    assert front_base_rest is not None
    assert front_car_rest is not None

    coordinated_raise_pose = {
        pit_to_car: 0.33,
        cylinder_stroke: 0.33,
        front_base_slider_travel: 0.31,
        rear_base_slider_travel: 0.31,
        front_car_slider_travel: 0.31,
        rear_car_slider_travel: 0.31,
        front_primary_arm_pivot: -0.63,
        rear_primary_arm_pivot: -0.63,
        front_center_pivot: 0.63,
        rear_center_pivot: 0.63,
    }

    with ctx.pose(coordinated_raise_pose):
        car_high = ctx.part_world_position(car)
        rod_high = ctx.part_world_position(cylinder_rod)
        front_base_high = ctx.part_world_position(front_base_slider)
        front_car_high = ctx.part_world_position(front_car_slider)
        assert car_high is not None
        assert rod_high is not None
        assert front_base_high is not None
        assert front_car_high is not None

        ctx.check(
            "car_moves_vertically",
            car_high[2] > car_rest[2] + 0.30
            and abs(car_high[0] - car_rest[0]) < 1e-6
            and abs(car_high[1] - car_rest[1]) < 1e-6,
            details=f"car rest={car_rest}, high={car_high}",
        )
        ctx.check(
            "hydraulic_rod_extends_upward",
            rod_high[2] > rod_rest[2] + 0.30
            and abs(rod_high[0] - rod_rest[0]) < 1e-6
            and abs(rod_high[1] - rod_rest[1]) < 1e-6,
            details=f"rod rest={rod_rest}, high={rod_high}",
        )
        ctx.check(
            "scissor_slider_retracts_inward",
            front_base_high[0] < front_base_rest[0] - 0.25
            and front_car_high[0] < front_car_rest[0] - 0.25,
            details=(
                f"front base slider rest={front_base_rest}, high={front_base_high}; "
                f"front car slider rest={front_car_rest}, high={front_car_high}"
            ),
        )
        ctx.expect_contact(car, pit_frame, name="raised_car_guides_stay_on_rails")
        ctx.expect_contact(cylinder_rod, car, name="raised_rod_stays_under_car")
        ctx.expect_contact(
            front_primary_arm,
            front_secondary_arm,
            name="raised_front_scissor_center_contact",
        )
        ctx.expect_contact(
            rear_primary_arm,
            rear_secondary_arm,
            name="raised_rear_scissor_center_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
