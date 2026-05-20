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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _add_member(part, name: str, a, b, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_blade_mesh():
    profile = [
        (0.024, -0.014),
        (0.050, -0.030),
        (0.100, -0.032),
        (0.142, -0.015),
        (0.150, 0.010),
        (0.128, 0.028),
        (0.084, 0.034),
        (0.046, 0.024),
        (0.024, 0.010),
    ]
    return _save_mesh("box_fan_blade.obj", ExtrudeGeometry(profile, height=0.006, center=True))


def _build_square_grille_loop():
    return _save_mesh(
        "box_fan_grille_square_loop.obj",
        wire_from_points(
            [
                (-0.164, 0.0, -0.164),
                (0.164, 0.0, -0.164),
                (0.164, 0.0, 0.164),
                (-0.164, 0.0, 0.164),
            ],
            radius=0.004,
            radial_segments=14,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.020,
            corner_segments=8,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_box_fan", assets=ASSETS)

    housing_white = model.material("housing_white", rgba=(0.88, 0.89, 0.91, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.62, 0.66, 0.70, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.26, 0.29, 0.31, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.76, 0.79, 0.82, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.56, 0.60, 0.64, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.11, 0.12, 1.0))

    blade_mesh = _build_blade_mesh()
    square_loop_mesh = _build_square_grille_loop()
    ring_meshes = {
        "ring_inner": _save_mesh("box_fan_grille_ring_inner.obj", TorusGeometry(radius=0.048, tube=0.0034)),
        "ring_mid_1": _save_mesh("box_fan_grille_ring_mid_1.obj", TorusGeometry(radius=0.082, tube=0.0034)),
        "ring_mid_2": _save_mesh("box_fan_grille_ring_mid_2.obj", TorusGeometry(radius=0.116, tube=0.0034)),
        "ring_outer": _save_mesh("box_fan_grille_ring_outer.obj", TorusGeometry(radius=0.156, tube=0.0034)),
    }

    width = 0.42
    height = 0.42
    depth = 0.11
    border = 0.045
    opening = width - (2.0 * border)
    half_w = width * 0.5
    half_h = height * 0.5
    half_d = depth * 0.5
    front_bezel_depth = 0.012

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, border)),
        origin=Origin(xyz=(0.0, 0.0, half_h - (border * 0.5))),
        material=housing_white,
        name="top_shell",
    )
    housing.visual(
        Box((width, depth, border)),
        origin=Origin(xyz=(0.0, 0.0, -half_h + (border * 0.5))),
        material=housing_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((border, depth, opening)),
        origin=Origin(xyz=(-half_w + (border * 0.5), 0.0, 0.0)),
        material=housing_white,
        name="left_shell",
    )
    housing.visual(
        Box((border, depth, opening)),
        origin=Origin(xyz=(half_w - (border * 0.5), 0.0, 0.0)),
        material=housing_white,
        name="right_shell",
    )
    housing.visual(
        Box((width - 0.006, front_bezel_depth, border + 0.012)),
        origin=Origin(xyz=(0.0, half_d - (front_bezel_depth * 0.5), half_h - (border + 0.012) * 0.5)),
        material=trim_gray,
        name="front_top_bezel",
    )
    housing.visual(
        Box((width - 0.006, front_bezel_depth, border + 0.012)),
        origin=Origin(xyz=(0.0, half_d - (front_bezel_depth * 0.5), -half_h + (border + 0.012) * 0.5)),
        material=trim_gray,
        name="front_bottom_bezel",
    )
    housing.visual(
        Box((border + 0.012, front_bezel_depth, opening + 0.016)),
        origin=Origin(xyz=(-half_w + (border + 0.012) * 0.5, half_d - (front_bezel_depth * 0.5), 0.0)),
        material=trim_gray,
        name="front_left_bezel",
    )
    housing.visual(
        Box((border + 0.012, front_bezel_depth, opening + 0.016)),
        origin=Origin(xyz=(half_w - (border + 0.012) * 0.5, half_d - (front_bezel_depth * 0.5), 0.0)),
        material=trim_gray,
        name="front_right_bezel",
    )
    housing.visual(
        Box((opening, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=trim_gray,
        name="motor_crossbar_horizontal",
    )
    housing.visual(
        Box((0.022, 0.016, opening)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=trim_gray,
        name="motor_crossbar_vertical",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="motor_pod",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.114),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="center_spindle",
    )
    housing.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(half_w + 0.004, 0.0, -0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="control_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=2.4,
        origin=Origin(),
    )

    front_grille = model.part("front_grille")
    front_grille.visual(
        square_loop_mesh,
        origin=Origin(),
        material=grille_dark,
        name="square_loop",
    )
    for ring_name, ring_mesh in ring_meshes.items():
        front_grille.visual(
            ring_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grille_dark,
            name=ring_name,
        )
    spoke_angles = (
        0.0,
        math.pi * 0.25,
        math.pi * 0.5,
        math.pi * 0.75,
        math.pi,
        math.pi * 1.25,
        math.pi * 1.5,
        math.pi * 1.75,
    )
    for index, angle in enumerate(spoke_angles):
        radius = 0.162
        end = (math.cos(angle) * radius, 0.0, math.sin(angle) * radius)
        _add_member(front_grille, f"spoke_{index}", (0.0, 0.0, 0.0), end, 0.003, grille_dark)
    front_grille.visual(
        Box((0.018, 0.014, 0.020)),
        origin=Origin(xyz=(0.156, -0.007, 0.0)),
        material=grille_dark,
        name="tab_right",
    )
    front_grille.visual(
        Box((0.018, 0.014, 0.020)),
        origin=Origin(xyz=(-0.156, -0.007, 0.0)),
        material=grille_dark,
        name="tab_left",
    )
    front_grille.visual(
        Box((0.020, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.007, 0.156)),
        material=grille_dark,
        name="tab_top",
    )
    front_grille.visual(
        Box((0.020, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.007, -0.156)),
        material=grille_dark,
        name="tab_bottom",
    )
    front_grille.inertial = Inertial.from_geometry(
        Box((0.34, 0.016, 0.34)),
        mass=0.2,
        origin=Origin(),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.034, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="hub_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="front_cap",
    )
    blade_count = 5
    for index in range(blade_count):
        angle = (2.0 * math.pi * index) / blade_count
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(math.pi / 2.0, angle, 0.0)),
            material=blade_gray,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((0.32, 0.055, 0.32)),
        mass=0.35,
        origin=Origin(),
    )

    knob = model.part("speed_knob")
    knob.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="knob_stem",
    )
    knob.visual(
        Box((0.010, 0.003, 0.010)),
        origin=Origin(xyz=(0.027, 0.0, 0.016)),
        material=housing_white,
        name="indicator_tick",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.045, 0.045)),
        mass=0.05,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "grille_mount",
        ArticulationType.FIXED,
        parent="housing",
        child="front_grille",
        origin=Origin(xyz=(0.0, half_d + 0.001, 0.0)),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent="housing",
        child="rotor",
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=22.0),
    )
    model.articulation(
        "speed_control",
        ArticulationType.CONTINUOUS,
        parent="housing",
        child="speed_knob",
        origin=Origin(xyz=(half_w + 0.008, 0.0, -0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("speed_knob")
    blade_spin = object_model.get_articulation("blade_spin")
    speed_control = object_model.get_articulation("speed_control")

    front_right_bezel = housing.get_visual("front_right_bezel")
    center_spindle = housing.get_visual("center_spindle")
    control_boss = housing.get_visual("control_boss")
    tab_right = front_grille.get_visual("tab_right")
    tab_top = front_grille.get_visual("tab_top")
    ring_outer = front_grille.get_visual("ring_outer")
    hub_sleeve = rotor.get_visual("hub_sleeve")
    blade_0 = rotor.get_visual("blade_0")
    blade_2 = rotor.get_visual("blade_2")
    blade_4 = rotor.get_visual("blade_4")
    knob_body = knob.get_visual("knob_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(rotor, housing, reason="rotor hub sleeve nests around the stationary spindle axle")
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(front_grille, housing, elem_a=tab_right, elem_b=front_right_bezel)
    ctx.expect_contact(front_grille, housing, elem_a=tab_top, elem_b=housing.get_visual("front_top_bezel"))
    ctx.expect_within(front_grille, housing, axes="xz")
    ctx.expect_overlap(front_grille, housing, axes="xz", min_overlap=0.25)
    ctx.expect_overlap(rotor, front_grille, axes="xz", min_overlap=0.08)
    ctx.expect_within(rotor, housing, axes="xz")
    ctx.expect_origin_distance(rotor, housing, axes="xz", max_dist=0.001)
    ctx.expect_overlap(rotor, housing, axes="xz", min_overlap=0.012, elem_a=hub_sleeve, elem_b=center_spindle)
    ctx.expect_within(rotor, housing, axes="xz", inner_elem=blade_4)
    ctx.expect_gap(
        front_grille,
        rotor,
        axis="y",
        min_gap=0.012,
        max_gap=0.050,
        positive_elem=ring_outer,
        negative_elem=blade_0,
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=knob_body,
        negative_elem=control_boss,
    )
    ctx.expect_within(knob, housing, axes="yz")
    ctx.expect_overlap(knob, housing, axes="yz", min_overlap=0.03)

    with ctx.pose({blade_spin: math.pi / 5.0}):
        ctx.expect_within(rotor, housing, axes="xz")
        ctx.expect_gap(
            front_grille,
            rotor,
            axis="y",
            min_gap=0.012,
            max_gap=0.050,
            positive_elem=ring_outer,
            negative_elem=blade_2,
        )

    with ctx.pose({speed_control: math.pi / 2.0}):
        ctx.expect_gap(
            knob,
            housing,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=knob_body,
            negative_elem=control_boss,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
