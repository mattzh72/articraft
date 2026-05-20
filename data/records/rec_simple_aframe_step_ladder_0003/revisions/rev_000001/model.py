from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_axis(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_box_beam(part, name, a, b, size_xy, material):
    part.visual(
        Box((size_xy[0], size_xy[1], _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis(a, b)),
        material=material,
        name=name,
    )


def _add_rivet_head(part, name, xyz, radius, thickness, material, axis="y"):
    if axis == "y":
        rpy = (math.pi / 2.0, 0.0, 0.0)
    elif axis == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=thickness),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _rect_profile(width: float, height: float):
    return [
        (-width * 0.5, -height * 0.5),
        (width * 0.5, -height * 0.5),
        (width * 0.5, height * 0.5),
        (-width * 0.5, height * 0.5),
    ]


def _build_tread_plate_mesh():
    outer = _rect_profile(0.088, 0.372)
    hole_profiles = []
    for x in (-0.026, 0.0, 0.026):
        for y in (-0.112, 0.0, 0.112):
            slot = rounded_rect_profile(0.012, 0.072, radius=0.003, corner_segments=6)
            hole_profiles.append([(px + x, py + y) for px, py in slot])
    tread_geom = ExtrudeWithHolesGeometry(
        outer_profile=outer,
        hole_profiles=hole_profiles,
        height=0.0035,
        center=True,
    )
    return _save_mesh("rugged_step_ladder_tread_plate.obj", tread_geom)


def _step_center_x(z: float) -> float:
    front_bottom_x = 0.026
    front_top_x = 0.168
    front_bottom_z = 0.028
    front_top_z = 0.915
    t = (z - front_bottom_z) / (front_top_z - front_bottom_z)
    return front_bottom_x + (front_top_x - front_bottom_x) * t + 0.012


def _add_front_step(part, step_mesh, step_index, z, metal, dark_steel):
    x = _step_center_x(z)
    part.visual(
        step_mesh,
        origin=Origin(xyz=(x, 0.0, z + 0.013)),
        material=metal,
        name=f"step_{step_index}_plate",
    )
    part.visual(
        Box((0.058, 0.392, 0.018)),
        origin=Origin(xyz=(x + 0.006, 0.0, z + 0.002)),
        material=metal,
        name=f"step_{step_index}_channel",
    )
    part.visual(
        Box((0.024, 0.022, 0.045)),
        origin=Origin(xyz=(x + 0.020, 0.202, z + 0.002)),
        material=dark_steel,
        name=f"step_{step_index}_left_bracket",
    )
    part.visual(
        Box((0.024, 0.022, 0.045)),
        origin=Origin(xyz=(x + 0.020, -0.202, z + 0.002)),
        material=dark_steel,
        name=f"step_{step_index}_right_bracket",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.372),
        origin=Origin(xyz=(x - 0.030, 0.0, z + 0.011), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name=f"step_{step_index}_nose",
    )
    for side_sign, y in ((1.0, 0.239), (-1.0, -0.239)):
        _add_rivet_head(
            part,
            f"step_{step_index}_rivet_{'left' if side_sign > 0 else 'right'}_a",
            (x + 0.008, y, z + 0.017),
            radius=0.0048,
            thickness=0.006,
            material=dark_steel,
        )
        _add_rivet_head(
            part,
            f"step_{step_index}_rivet_{'left' if side_sign > 0 else 'right'}_b",
            (x + 0.008, y, z - 0.003),
            radius=0.0048,
            thickness=0.006,
            material=dark_steel,
        )


def _add_spreader_geometry(
    part,
    vector,
    steel,
    dark_steel,
    *,
    coupler_pin_length=0.0,
    coupler_pin_sign=1.0,
    pivot_rivet_sign=1.0,
    tip_rivet_sign=1.0,
):
    dx, dz = vector
    length = math.sqrt(dx * dx + dz * dz)
    pitch = math.atan2(dx, dz)
    part.visual(
        Box((0.020, 0.0045, length)),
        origin=Origin(xyz=(dx * 0.5, 0.0, dz * 0.5), rpy=(0.0, pitch, 0.0)),
        material=steel,
        name="strap",
    )
    part.visual(
        Box((0.024, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_eye",
    )
    part.visual(
        Box((0.024, 0.010, 0.024)),
        origin=Origin(xyz=(dx, 0.0, dz)),
        material=dark_steel,
        name="tip_eye",
    )
    _add_rivet_head(
        part,
        "pivot_rivet_outer",
        (0.0, pivot_rivet_sign * 0.0045, 0.0),
        radius=0.0045,
        thickness=0.004,
        material=dark_steel,
        axis="y",
    )
    _add_rivet_head(
        part,
        "tip_rivet_outer",
        (dx, tip_rivet_sign * 0.0045, dz),
        radius=0.0045,
        thickness=0.004,
        material=dark_steel,
        axis="y",
    )
    if coupler_pin_length > 0.0:
        part.visual(
            Cylinder(radius=0.004, length=coupler_pin_length),
            origin=Origin(
                xyz=(
                    dx,
                    coupler_pin_sign * (0.005 + coupler_pin_length * 0.5),
                    dz,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name="coupler_pin",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_a_frame_step_ladder", assets=ASSETS)

    rail_orange = model.material("rail_orange", rgba=(0.92, 0.47, 0.12, 1.0))
    top_cap_black = model.material("top_cap_black", rgba=(0.14, 0.15, 0.16, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.77, 0.79, 1.0))
    galvanized = model.material("galvanized", rgba=(0.60, 0.62, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    tread_mesh = _build_tread_plate_mesh()
    hinge_origin = (0.170, 0.0, 0.942)

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.30, 0.50, 1.00)),
        mass=9.5,
        origin=Origin(xyz=(0.16, 0.0, 0.50)),
    )
    _add_box_beam(
        front_frame,
        "front_left_rail",
        (0.026, 0.225, 0.028),
        (0.168, 0.225, 0.915),
        (0.048, 0.028),
        rail_orange,
    )
    _add_box_beam(
        front_frame,
        "front_right_rail",
        (0.026, -0.225, 0.028),
        (0.168, -0.225, 0.915),
        (0.048, 0.028),
        rail_orange,
    )
    front_frame.visual(
        Box((0.084, 0.430, 0.062)),
        origin=Origin(xyz=(0.060, 0.0, 0.118)),
        material=rail_orange,
        name="front_lower_crossmember",
    )
    _add_box_beam(
        front_frame,
        "front_left_upper_reinforcement",
        (0.084, 0.217, 0.342),
        (0.152, 0.217, 0.854),
        (0.026, 0.012),
        dark_steel,
    )
    _add_box_beam(
        front_frame,
        "front_right_upper_reinforcement",
        (0.084, -0.217, 0.342),
        (0.152, -0.217, 0.854),
        (0.026, 0.012),
        dark_steel,
    )
    for idx, z in enumerate((0.176, 0.366, 0.556, 0.746), start=1):
        _add_front_step(front_frame, tread_mesh, idx, z, aluminum, dark_steel)
    front_frame.visual(
        Box((0.088, 0.332, 0.006)),
        origin=Origin(xyz=(0.102, 0.0, 0.928)),
        material=top_cap_black,
        name="top_tray_floor",
    )
    front_frame.visual(
        Box((0.098, 0.360, 0.010)),
        origin=Origin(xyz=(0.102, 0.0, 0.916)),
        material=top_cap_black,
        name="top_tray_base",
    )
    front_frame.visual(
        Box((0.090, 0.016, 0.040)),
        origin=Origin(xyz=(0.096, 0.164, 0.934)),
        material=top_cap_black,
        name="top_tray_left_wall",
    )
    front_frame.visual(
        Box((0.090, 0.016, 0.040)),
        origin=Origin(xyz=(0.096, -0.164, 0.934)),
        material=top_cap_black,
        name="top_tray_right_wall",
    )
    front_frame.visual(
        Box((0.018, 0.452, 0.040)),
        origin=Origin(xyz=(0.057, 0.0, 0.938)),
        material=top_cap_black,
        name="top_tray_front_rim",
    )
    front_frame.visual(
        Box((0.018, 0.150, 0.032)),
        origin=Origin(xyz=(0.146, 0.0, 0.932)),
        material=top_cap_black,
        name="top_tray_rear_rim",
    )
    front_frame.visual(
        Box((0.024, 0.108, 0.018)),
        origin=Origin(xyz=(0.147, 0.214, 0.944)),
        material=top_cap_black,
        name="top_tray_left_mount_ear",
    )
    front_frame.visual(
        Box((0.024, 0.108, 0.018)),
        origin=Origin(xyz=(0.147, -0.214, 0.944)),
        material=top_cap_black,
        name="top_tray_right_mount_ear",
    )
    front_frame.visual(
        Box((0.034, 0.010, 0.054)),
        origin=Origin(xyz=(hinge_origin[0], 0.263, 0.930)),
        material=dark_steel,
        name="front_hinge_plate_left",
    )
    front_frame.visual(
        Box((0.034, 0.010, 0.054)),
        origin=Origin(xyz=(hinge_origin[0], -0.263, 0.930)),
        material=dark_steel,
        name="front_hinge_plate_right",
    )
    front_frame.visual(
        Box((0.024, 0.020, 0.010)),
        origin=Origin(xyz=(0.166, 0.248, 0.899)),
        material=dark_steel,
        name="front_hinge_bridge_left",
    )
    front_frame.visual(
        Box((0.024, 0.020, 0.010)),
        origin=Origin(xyz=(0.166, -0.248, 0.899)),
        material=dark_steel,
        name="front_hinge_bridge_right",
    )
    front_frame.visual(
        Box((0.028, 0.010, 0.030)),
        origin=Origin(xyz=(0.112, 0.214, 0.470)),
        material=dark_steel,
        name="front_left_brace_boss",
    )
    front_frame.visual(
        Box((0.028, 0.010, 0.030)),
        origin=Origin(xyz=(0.112, -0.214, 0.470)),
        material=dark_steel,
        name="front_right_brace_boss",
    )
    front_frame.visual(
        Box((0.082, 0.056, 0.028)),
        origin=Origin(xyz=(0.020, 0.225, 0.014)),
        material=rubber,
        name="front_left_foot",
    )
    front_frame.visual(
        Box((0.082, 0.056, 0.028)),
        origin=Origin(xyz=(0.020, -0.225, 0.014)),
        material=rubber,
        name="front_right_foot",
    )
    for x, y, z in (
        (0.170, 0.271, 0.952),
        (0.170, -0.271, 0.952),
        (0.112, 0.219, 0.470),
        (0.112, -0.219, 0.470),
    ):
        _add_rivet_head(front_frame, f"rivet_{x:.3f}_{y:.3f}_{z:.3f}", (x, y, z), 0.0055, 0.006, dark_steel)

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.44, 0.44, 0.98)),
        mass=5.4,
        origin=Origin(xyz=(-0.18, 0.0, -0.48)),
    )
    _add_box_beam(
        rear_frame,
        "rear_left_rail",
        (-0.010, 0.195, -0.020),
        (-0.450, 0.195, -0.914),
        (0.040, 0.024),
        rail_orange,
    )
    _add_box_beam(
        rear_frame,
        "rear_right_rail",
        (-0.010, -0.195, -0.020),
        (-0.450, -0.195, -0.914),
        (0.040, 0.024),
        rail_orange,
    )
    rear_frame.visual(
        Box((0.026, 0.018, 0.056)),
        origin=Origin(xyz=(0.004, 0.249, -0.008)),
        material=dark_steel,
        name="rear_hinge_lug_left",
    )
    rear_frame.visual(
        Box((0.026, 0.018, 0.056)),
        origin=Origin(xyz=(0.004, -0.249, -0.008)),
        material=dark_steel,
        name="rear_hinge_lug_right",
    )
    _add_box_beam(
        rear_frame,
        "rear_hinge_bridge_left",
        (-0.010, 0.207, -0.020),
        (0.004, 0.240, -0.016),
        (0.010, 0.008),
        dark_steel,
    )
    _add_box_beam(
        rear_frame,
        "rear_hinge_bridge_right",
        (-0.010, -0.207, -0.020),
        (0.004, -0.240, -0.016),
        (0.010, 0.008),
        dark_steel,
    )
    rear_frame.visual(
        Box((0.072, 0.390, 0.046)),
        origin=Origin(xyz=(-0.060, 0.0, -0.104)),
        material=dark_steel,
        name="rear_top_crossbar",
    )
    for idx, (x, z) in enumerate(((-0.155, -0.298), (-0.270, -0.512), (-0.385, -0.724)), start=1):
        rear_frame.visual(
            Box((0.020, 0.370, 0.044)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=galvanized,
            name=f"rear_crossbar_{idx}",
        )
    rear_frame.visual(
        Box((0.028, 0.012, 0.028)),
        origin=Origin(xyz=(-0.100, 0.195, -0.472)),
        material=dark_steel,
        name="rear_left_brace_boss",
    )
    rear_frame.visual(
        Box((0.028, 0.012, 0.028)),
        origin=Origin(xyz=(-0.100, -0.195, -0.472)),
        material=dark_steel,
        name="rear_right_brace_boss",
    )
    rear_frame.visual(
        Box((0.060, 0.008, 0.032)),
        origin=Origin(xyz=(-0.100, 0.191, -0.472)),
        material=dark_steel,
        name="rear_left_brace_gusset",
    )
    rear_frame.visual(
        Box((0.060, 0.008, 0.032)),
        origin=Origin(xyz=(-0.100, -0.191, -0.472)),
        material=dark_steel,
        name="rear_right_brace_gusset",
    )
    rear_frame.visual(
        Box((0.112, 0.008, 0.020)),
        origin=Origin(xyz=(-0.154, 0.191, -0.472)),
        material=dark_steel,
        name="rear_left_brace_mount_plate",
    )
    rear_frame.visual(
        Box((0.112, 0.008, 0.020)),
        origin=Origin(xyz=(-0.154, -0.191, -0.472)),
        material=dark_steel,
        name="rear_right_brace_mount_plate",
    )
    rear_frame.visual(
        Box((0.082, 0.050, 0.028)),
        origin=Origin(xyz=(-0.450, 0.195, -0.928)),
        material=rubber,
        name="rear_left_foot",
    )
    rear_frame.visual(
        Box((0.082, 0.050, 0.028)),
        origin=Origin(xyz=(-0.450, -0.195, -0.928)),
        material=rubber,
        name="rear_right_foot",
    )
    for x, y, z in (
        (-0.002, 0.259, -0.008),
        (-0.002, -0.259, -0.008),
        (-0.100, 0.201, -0.472),
        (-0.100, -0.201, -0.472),
    ):
        _add_rivet_head(rear_frame, f"rear_rivet_{x:.3f}_{y:.3f}_{z:.3f}", (x, y, z), 0.0050, 0.005, dark_steel)
    front_left_spreader = model.part("front_left_spreader")
    front_left_spreader.inertial = Inertial.from_geometry(
        Box((0.18, 0.02, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.072, 0.0, -0.058)),
    )
    _add_spreader_geometry(
        front_left_spreader,
        (0.145, -0.115),
        galvanized,
        dark_steel,
        coupler_pin_length=0.010,
        coupler_pin_sign=-1.0,
        pivot_rivet_sign=1.0,
        tip_rivet_sign=1.0,
    )

    front_right_spreader = model.part("front_right_spreader")
    front_right_spreader.inertial = Inertial.from_geometry(
        Box((0.18, 0.02, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.072, 0.0, -0.058)),
    )
    _add_spreader_geometry(
        front_right_spreader,
        (0.145, -0.115),
        galvanized,
        dark_steel,
        coupler_pin_length=0.010,
        coupler_pin_sign=1.0,
        pivot_rivet_sign=-1.0,
        tip_rivet_sign=-1.0,
    )

    rear_left_spreader = model.part("rear_left_spreader")
    rear_left_spreader.inertial = Inertial.from_geometry(
        Box((0.23, 0.02, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.092, 0.0, -0.058)),
    )
    _add_spreader_geometry(
        rear_left_spreader,
        (0.185, -0.115),
        galvanized,
        dark_steel,
        pivot_rivet_sign=1.0,
        tip_rivet_sign=-1.0,
    )

    rear_right_spreader = model.part("rear_right_spreader")
    rear_right_spreader.inertial = Inertial.from_geometry(
        Box((0.23, 0.02, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.092, 0.0, -0.058)),
    )
    _add_spreader_geometry(
        rear_right_spreader,
        (0.185, -0.115),
        galvanized,
        dark_steel,
        pivot_rivet_sign=-1.0,
        tip_rivet_sign=1.0,
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=hinge_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.38, upper=0.08),
    )
    model.articulation(
        "front_left_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=front_left_spreader,
        origin=Origin(xyz=(0.112, 0.204, 0.470)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.18, upper=0.74),
    )
    model.articulation(
        "front_right_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=front_right_spreader,
        origin=Origin(xyz=(0.112, -0.204, 0.470)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.18, upper=0.74),
    )
    model.articulation(
        "rear_left_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=rear_left_spreader,
        origin=Origin(xyz=(-0.100, 0.184, -0.472)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.72, upper=0.18),
    )
    model.articulation(
        "rear_right_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=rear_right_spreader,
        origin=Origin(xyz=(-0.100, -0.184, -0.472)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.72, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    front_left_spreader = object_model.get_part("front_left_spreader")
    front_right_spreader = object_model.get_part("front_right_spreader")
    rear_left_spreader = object_model.get_part("rear_left_spreader")
    rear_right_spreader = object_model.get_part("rear_right_spreader")

    top_hinge = object_model.get_articulation("top_hinge")
    front_left_spreader_hinge = object_model.get_articulation("front_left_spreader_hinge")
    front_right_spreader_hinge = object_model.get_articulation("front_right_spreader_hinge")
    rear_left_spreader_hinge = object_model.get_articulation("rear_left_spreader_hinge")
    rear_right_spreader_hinge = object_model.get_articulation("rear_right_spreader_hinge")

    front_hinge_plate_left = front_frame.get_visual("front_hinge_plate_left")
    front_hinge_plate_right = front_frame.get_visual("front_hinge_plate_right")
    rear_hinge_lug_left = rear_frame.get_visual("rear_hinge_lug_left")
    rear_hinge_lug_right = rear_frame.get_visual("rear_hinge_lug_right")
    front_left_brace_boss = front_frame.get_visual("front_left_brace_boss")
    front_right_brace_boss = front_frame.get_visual("front_right_brace_boss")
    rear_left_brace_boss = rear_frame.get_visual("rear_left_brace_boss")
    rear_right_brace_boss = rear_frame.get_visual("rear_right_brace_boss")
    front_left_pivot_eye = front_left_spreader.get_visual("pivot_eye")
    front_right_pivot_eye = front_right_spreader.get_visual("pivot_eye")
    rear_left_pivot_eye = rear_left_spreader.get_visual("pivot_eye")
    rear_right_pivot_eye = rear_right_spreader.get_visual("pivot_eye")
    front_left_tip_eye = front_left_spreader.get_visual("tip_eye")
    front_right_tip_eye = front_right_spreader.get_visual("tip_eye")
    rear_left_tip_eye = rear_left_spreader.get_visual("tip_eye")
    rear_right_tip_eye = rear_right_spreader.get_visual("tip_eye")
    front_left_coupler_pin = front_left_spreader.get_visual("coupler_pin")
    front_right_coupler_pin = front_right_spreader.get_visual("coupler_pin")
    front_left_foot = front_frame.get_visual("front_left_foot")
    front_right_foot = front_frame.get_visual("front_right_foot")
    rear_left_foot = rear_frame.get_visual("rear_left_foot")
    rear_right_foot = rear_frame.get_visual("rear_right_foot")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem=front_hinge_plate_left,
        negative_elem=rear_hinge_lug_left,
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        name="left_hinge_plate_contact",
    )
    ctx.expect_gap(
        rear_frame,
        front_frame,
        axis="y",
        positive_elem=rear_hinge_lug_right,
        negative_elem=front_hinge_plate_right,
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        name="right_hinge_plate_contact",
    )
    ctx.expect_gap(
        front_frame,
        front_left_spreader,
        axis="y",
        positive_elem=front_left_brace_boss,
        negative_elem=front_left_pivot_eye,
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        name="front_left_spreader_mount_contact",
    )
    ctx.expect_gap(
        front_right_spreader,
        front_frame,
        axis="y",
        positive_elem=front_right_pivot_eye,
        negative_elem=front_right_brace_boss,
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        name="front_right_spreader_mount_contact",
    )
    ctx.expect_gap(
        rear_frame,
        rear_left_spreader,
        axis="y",
        positive_elem=rear_left_brace_boss,
        negative_elem=rear_left_pivot_eye,
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        name="rear_left_spreader_mount_contact",
    )
    ctx.expect_gap(
        rear_right_spreader,
        rear_frame,
        axis="y",
        positive_elem=rear_right_pivot_eye,
        negative_elem=rear_right_brace_boss,
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        name="rear_right_spreader_mount_contact",
    )
    ctx.expect_gap(
        front_left_spreader,
        rear_left_spreader,
        axis="y",
        positive_elem=front_left_coupler_pin,
        negative_elem=rear_left_tip_eye,
        min_gap=-1e-5,
        max_gap=0.0005,
        name="left_spreader_link_contact",
    )
    ctx.expect_gap(
        rear_right_spreader,
        front_right_spreader,
        axis="y",
        positive_elem=rear_right_tip_eye,
        negative_elem=front_right_coupler_pin,
        min_gap=-1e-5,
        max_gap=0.0005,
        name="right_spreader_link_contact",
    )
    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem=front_left_foot,
        negative_elem=rear_left_foot,
        min_gap=0.19,
        max_gap=0.26,
        name="left_foot_stance_depth",
    )
    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem=front_right_foot,
        negative_elem=rear_right_foot,
        min_gap=0.19,
        max_gap=0.26,
        name="right_foot_stance_depth",
    )

    front_aabb = ctx.part_world_aabb(front_frame)
    ctx.check(
        "front_frame_height_realistic",
        front_aabb is not None and 0.94 <= (front_aabb[1][2] - front_aabb[0][2]) <= 1.04,
        details=f"front frame bounds={front_aabb}",
    )
    step_2_aabb = ctx.part_element_world_aabb(front_frame, elem="step_2_plate")
    ctx.check(
        "step_tread_geometry_sized_for_boot_use",
        step_2_aabb is not None
        and 0.08 <= (step_2_aabb[1][0] - step_2_aabb[0][0]) <= 0.10
        and 0.34 <= (step_2_aabb[1][1] - step_2_aabb[0][1]) <= 0.39,
        details=f"step_2_plate bounds={step_2_aabb}",
    )
    tray_aabb = ctx.part_element_world_aabb(front_frame, elem="top_tray_base")
    ctx.check(
        "top_cap_has_serviceable_tool_tray_width",
        tray_aabb is not None and 0.34 <= (tray_aabb[1][1] - tray_aabb[0][1]) <= 0.38,
        details=f"top_tray_base bounds={tray_aabb}",
    )
    left_plate_aabb = ctx.part_element_world_aabb(front_frame, elem="front_hinge_plate_left")
    right_plate_aabb = ctx.part_element_world_aabb(front_frame, elem="front_hinge_plate_right")
    ctx.check(
        "hinge_hardware_stands_proud_of_side_rails",
        left_plate_aabb is not None
        and right_plate_aabb is not None
        and left_plate_aabb[0][1] > 0.255
        and right_plate_aabb[1][1] < -0.255,
        details=f"left={left_plate_aabb}, right={right_plate_aabb}",
    )
    ctx.check(
        "all_main_axes_use_side_to_side_hinges",
        tuple(round(v, 3) for v in top_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in front_left_spreader_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in front_right_spreader_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in rear_left_spreader_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in rear_right_spreader_hinge.axis) == (0.0, 1.0, 0.0),
        details=(
            f"top={top_hinge.axis}, front_left={front_left_spreader_hinge.axis}, "
            f"front_right={front_right_spreader_hinge.axis}, rear_left={rear_left_spreader_hinge.axis}, "
            f"rear_right={rear_right_spreader_hinge.axis}"
        ),
    )

    def _center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    rest_tip = ctx.part_element_world_aabb(front_left_spreader, elem="tip_eye")
    rest_right_tip = ctx.part_element_world_aabb(front_right_spreader, elem="tip_eye")
    rest_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    with ctx.pose(
        {
            top_hinge: -0.12,
            front_left_spreader_hinge: 0.12,
            front_right_spreader_hinge: 0.12,
            rear_left_spreader_hinge: -0.12,
            rear_right_spreader_hinge: -0.12,
        }
    ):
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="x",
            positive_elem=front_left_foot,
            negative_elem=rear_left_foot,
            min_gap=0.03,
            max_gap=0.12,
            name="folded_pose_reduces_stance_depth",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")
        posed_tip = ctx.part_element_world_aabb(front_left_spreader, elem="tip_eye")
        posed_right_tip = ctx.part_element_world_aabb(front_right_spreader, elem="tip_eye")
        posed_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
        ctx.check(
            "left_spreader_tip_moves_when_folded",
            rest_tip is not None
            and posed_tip is not None
            and math.dist(_center(rest_tip), _center(posed_tip)) >= 0.02,
            details=f"rest={rest_tip}, posed={posed_tip}",
        )
        ctx.check(
            "rear_leg_swings_forward_under_top_hinge",
            rest_rear_foot is not None
            and posed_rear_foot is not None
            and _center(posed_rear_foot)[0] > _center(rest_rear_foot)[0] + 0.10,
            details=f"rest={rest_rear_foot}, posed={posed_rear_foot}",
        )
        ctx.check(
            "right_spreader_tip_moves_when_folded",
            posed_right_tip is not None
            and rest_right_tip is not None
            and math.dist(_center(rest_right_tip), _center(posed_right_tip)) >= 0.02,
            details=f"rest_right={rest_right_tip}, posed_right={posed_right_tip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
