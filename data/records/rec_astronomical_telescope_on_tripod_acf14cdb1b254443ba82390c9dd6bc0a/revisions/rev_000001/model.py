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


def _tube_shell_mesh(*, length: float, outer_radius: float, inner_radius: float):
    half_length = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius * 0.99, -half_length),
            (outer_radius, -half_length * 0.94),
            (outer_radius, half_length * 0.94),
            (outer_radius * 0.99, half_length),
        ],
        [
            (inner_radius, -half_length + 0.004),
            (inner_radius, half_length - 0.004),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_dobsonian")

    birch_ply = model.material("birch_ply", rgba=(0.67, 0.56, 0.38, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    graphite = model.material("graphite", rgba=(0.34, 0.35, 0.37, 1.0))
    mirror_gray = model.material("mirror_gray", rgba=(0.62, 0.64, 0.68, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base_disc = model.part("disc_base")
    base_disc.visual(
        Cylinder(radius=0.22, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=birch_ply,
        name="ground_disc",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        base_disc.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(
                xyz=(0.155 * math.cos(angle), 0.155 * math.sin(angle), 0.006)
            ),
            material=rubber,
            name=f"foot_{index}",
        )
    base_disc.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=dark_metal,
        name="pivot_washer",
    )
    base_disc.inertial = Inertial.from_geometry(
        Box((0.44, 0.44, 0.040)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    rocker_box = model.part("rocker_box")
    rocker_box.visual(
        Box((0.360, 0.340, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=birch_ply,
        name="rocker_floor",
    )
    rocker_box.visual(
        Cylinder(radius=0.090, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_metal,
        name="azimuth_bearing",
    )
    rocker_box.visual(
        Box((0.300, 0.018, 0.185)),
        origin=Origin(xyz=(0.0, 0.179, 0.1095)),
        material=birch_ply,
        name="left_cheek",
    )
    rocker_box.visual(
        Box((0.300, 0.018, 0.185)),
        origin=Origin(xyz=(0.0, -0.179, 0.1095)),
        material=birch_ply,
        name="right_cheek",
    )
    rocker_box.visual(
        Box((0.050, 0.026, 0.080)),
        origin=Origin(xyz=(-0.125, 0.157, 0.058)),
        material=birch_ply,
        name="left_rear_buttress",
    )
    rocker_box.visual(
        Box((0.050, 0.026, 0.080)),
        origin=Origin(xyz=(-0.125, -0.157, 0.058)),
        material=birch_ply,
        name="right_rear_buttress",
    )
    rocker_box.inertial = Inertial.from_geometry(
        Box((0.360, 0.380, 0.210)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        _mesh(
            "dobsonian_tube_shell",
            _tube_shell_mesh(length=0.580, outer_radius=0.145, inner_radius=0.137),
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="tube_shell",
    )
    optical_tube.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.164, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="left_trunnion",
    )
    optical_tube.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.164, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="right_trunnion",
    )
    optical_tube.visual(
        Cylinder(radius=0.024, length=0.042),
        origin=Origin(
            xyz=(0.0, 0.149, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="left_axle_hub",
    )
    optical_tube.visual(
        Cylinder(radius=0.024, length=0.042),
        origin=Origin(
            xyz=(0.0, -0.149, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="right_axle_hub",
    )
    optical_tube.visual(
        Box((0.056, 0.040, 0.036)),
        origin=Origin(xyz=(0.215, 0.0, 0.163)),
        material=graphite,
        name="focuser_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.215, 0.0, 0.170)),
        material=dark_metal,
        name="focuser_drawtube",
    )
    optical_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.600),
        mass=2.6,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    secondary_spider = model.part("secondary_spider")
    secondary_spider.visual(
        Box((0.0012, 0.2738, 0.007)),
        material=dark_metal,
        name="horizontal_vane",
    )
    secondary_spider.visual(
        Box((0.0012, 0.007, 0.2738)),
        material=dark_metal,
        name="vertical_vane",
    )
    secondary_spider.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="central_hub",
    )
    secondary_spider.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="mirror_stalk",
    )
    secondary_spider.visual(
        Box((0.006, 0.040, 0.055)),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=mirror_gray,
        name="secondary_mirror",
    )
    secondary_spider.inertial = Inertial.from_geometry(
        Box((0.080, 0.270, 0.270)),
        mass=0.14,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_disc,
        child=rocker_box,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=rocker_box,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=1.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "tube_to_secondary",
        ArticulationType.FIXED,
        parent=optical_tube,
        child=secondary_spider,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_disc = object_model.get_part("disc_base")
    rocker_box = object_model.get_part("rocker_box")
    optical_tube = object_model.get_part("optical_tube")
    secondary_spider = object_model.get_part("secondary_spider")
    azimuth = object_model.get_articulation("azimuth_rotation")
    altitude = object_model.get_articulation("altitude_axis")

    def _visual_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "azimuth joint axis is vertical",
        tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"axis={azimuth.axis}",
    )
    ctx.check(
        "altitude joint axis pitches upward from horizontal rest",
        tuple(altitude.axis) == (0.0, -1.0, 0.0),
        details=f"axis={altitude.axis}",
    )
    ctx.expect_contact(
        base_disc,
        rocker_box,
        elem_a="ground_disc",
        elem_b="rocker_floor",
        name="rocker floor sits on the azimuth base disc",
    )
    ctx.expect_contact(
        optical_tube,
        rocker_box,
        elem_a="left_trunnion",
        elem_b="left_cheek",
        name="left trunnion bears on the left rocker cheek",
    )
    ctx.expect_contact(
        optical_tube,
        rocker_box,
        elem_a="right_trunnion",
        elem_b="right_cheek",
        name="right trunnion bears on the right rocker cheek",
    )
    ctx.expect_contact(
        secondary_spider,
        optical_tube,
        contact_tol=5e-4,
        name="secondary spider is mounted to the inside of the tube",
    )
    ctx.expect_within(
        secondary_spider,
        optical_tube,
        axes="yz",
        margin=0.0,
        name="secondary assembly stays inside the tube envelope",
    )

    with ctx.pose({azimuth: 0.0, altitude: 0.0}):
        rest_focuser = _visual_center(optical_tube, "focuser_body")
    with ctx.pose({azimuth: 0.0, altitude: 1.10}):
        raised_focuser = _visual_center(optical_tube, "focuser_body")
    ctx.check(
        "altitude motion lifts the nose of the tube",
        rest_focuser is not None
        and raised_focuser is not None
        and raised_focuser[2] > rest_focuser[2] + 0.09,
        details=f"rest={rest_focuser}, raised={raised_focuser}",
    )

    with ctx.pose({azimuth: 0.0, altitude: 0.0}):
        front_focuser = _visual_center(optical_tube, "focuser_body")
    with ctx.pose({azimuth: math.pi / 2.0, altitude: 0.0}):
        turned_focuser = _visual_center(optical_tube, "focuser_body")
    ctx.check(
        "azimuth motion swings the optical tube around the vertical axis",
        front_focuser is not None
        and turned_focuser is not None
        and front_focuser[0] > 0.08
        and abs(front_focuser[1]) < 0.03
        and turned_focuser[1] > 0.08
        and abs(turned_focuser[0]) < 0.03
        and abs(turned_focuser[2] - front_focuser[2]) < 0.01,
        details=f"front={front_focuser}, turned={turned_focuser}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
