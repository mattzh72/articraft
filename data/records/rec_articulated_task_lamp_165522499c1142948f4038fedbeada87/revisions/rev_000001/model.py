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


def _cyl_x(radius: float, length: float, *, xyz: tuple[float, float, float], name: str | None = None):
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)), name


def _cyl_y(radius: float, length: float, *, xyz: tuple[float, float, float], name: str | None = None):
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)), name


def _add_visual(part, geometry, *, origin: Origin, material, name: str | None = None) -> None:
    part.visual(geometry, origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="headboard_swing_lamp")

    black_metal = model.material("black_metal", rgba=(0.16, 0.16, 0.17, 1.0))
    brass = model.material("brass", rgba=(0.63, 0.54, 0.32, 1.0))
    ivory_enamel = model.material("ivory_enamel", rgba=(0.90, 0.88, 0.82, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.060, 0.070, 0.120)),
        mass=0.85,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )
    _add_visual(
        wall_bracket,
        Box((0.008, 0.060, 0.112)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=black_metal,
        name="bracket_plate",
    )
    _add_visual(
        wall_bracket,
        Box((0.006, 0.028, 0.056)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=black_metal,
        name="pivot_block",
    )
    for name, z in (("shoulder_ear_lower", -0.0085), ("shoulder_ear_upper", 0.0085)):
        _add_visual(
            wall_bracket,
            Cylinder(radius=0.013, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=name,
        )
    for name, z in (("upper_screw", 0.030), ("lower_screw", -0.030)):
        geom, origin, visual_name = _cyl_x(0.005, 0.004, xyz=(-0.014, 0.0, z), name=name)
        _add_visual(wall_bracket, geom, origin=origin, material=brass, name=visual_name)

    first_link = model.part("first_link")
    first_link.inertial = Inertial.from_geometry(
        Box((0.210, 0.024, 0.024)),
        mass=0.38,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
    )
    _add_visual(
        first_link,
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(),
        material=brass,
        name="shoulder_barrel",
    )
    _add_visual(
        first_link,
        Box((0.184, 0.004, 0.010)),
        origin=Origin(xyz=(0.100, -0.007, 0.0)),
        material=black_metal,
        name="link1_bar_left",
    )
    _add_visual(
        first_link,
        Box((0.184, 0.004, 0.010)),
        origin=Origin(xyz=(0.100, 0.007, 0.0)),
        material=black_metal,
        name="link1_bar_right",
    )
    _add_visual(
        first_link,
        Box((0.020, 0.018, 0.008)),
        origin=Origin(xyz=(0.098, 0.0, 0.0)),
        material=black_metal,
        name="link1_spacer",
    )
    for name, z in (("elbow_ear_lower", -0.0085), ("elbow_ear_upper", 0.0085)):
        _add_visual(
            first_link,
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(0.200, 0.0, z)),
            material=brass,
            name=name,
        )

    second_link = model.part("second_link")
    second_link.inertial = Inertial.from_geometry(
        Box((0.200, 0.032, 0.032)),
        mass=0.32,
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
    )
    _add_visual(
        second_link,
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(),
        material=brass,
        name="elbow_barrel",
    )
    _add_visual(
        second_link,
        Box((0.182, 0.004, 0.009)),
        origin=Origin(xyz=(0.096, -0.007, 0.0)),
        material=black_metal,
        name="link2_bar_left",
    )
    _add_visual(
        second_link,
        Box((0.182, 0.004, 0.009)),
        origin=Origin(xyz=(0.096, 0.007, 0.0)),
        material=black_metal,
        name="link2_bar_right",
    )
    _add_visual(
        second_link,
        Box((0.018, 0.018, 0.008)),
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
        material=black_metal,
        name="link2_spacer",
    )
    _add_visual(
        second_link,
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=black_metal,
        name="tip_yoke_block",
    )
    for name, y in (("tip_yoke_left", -0.008), ("tip_yoke_right", 0.008)):
        _add_visual(
            second_link,
            Box((0.014, 0.006, 0.028)),
            origin=Origin(xyz=(0.189, y, 0.0)),
            material=brass,
            name=name,
        )

    shade = model.part("shade")
    shade.inertial = Inertial.from_geometry(
        Box((0.170, 0.110, 0.110)),
        mass=0.42,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )
    geom, origin, visual_name = _cyl_y(0.0075, 0.010, xyz=(0.0, 0.0, 0.0), name="tilt_barrel")
    _add_visual(shade, geom, origin=origin, material=brass, name=visual_name)
    geom, origin, visual_name = _cyl_x(0.0045, 0.016, xyz=(0.008, 0.0, 0.0), name="shade_neck")
    _add_visual(shade, geom, origin=origin, material=brass, name=visual_name)
    geom, origin, visual_name = _cyl_x(0.012, 0.024, xyz=(0.026, 0.0, 0.0), name="shade_collar")
    _add_visual(shade, geom, origin=origin, material=black_metal, name=visual_name)

    shade_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.016, 0.000),
            (0.020, 0.016),
            (0.031, 0.060),
            (0.045, 0.112),
            (0.052, 0.140),
        ],
        [
            (0.011, 0.004),
            (0.014, 0.018),
            (0.024, 0.060),
            (0.039, 0.109),
            (0.046, 0.134),
        ],
        segments=56,
    ).rotate_y(math.pi / 2.0)
    shade_mesh = mesh_from_geometry(shade_shell_geom, "lamp_shade_shell")
    _add_visual(
        shade,
        shade_mesh,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=ivory_enamel,
        name="shade_shell",
    )

    model.articulation(
        "bracket_to_first_link",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=first_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.45,
            upper=1.45,
        ),
    )
    model.articulation(
        "first_to_second_link",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-2.35,
            upper=2.35,
        ),
    )
    model.articulation(
        "second_to_shade",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=shade,
        origin=Origin(xyz=(0.189, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.70,
        ),
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

    wall_bracket = object_model.get_part("wall_bracket")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("bracket_to_first_link")
    elbow = object_model.get_articulation("first_to_second_link")
    tilt = object_model.get_articulation("second_to_shade")

    shoulder_barrel = first_link.get_visual("shoulder_barrel")
    bracket_ear = wall_bracket.get_visual("shoulder_ear_upper")
    elbow_barrel = second_link.get_visual("elbow_barrel")
    elbow_ear = first_link.get_visual("elbow_ear_upper")
    tilt_barrel = shade.get_visual("tilt_barrel")
    tip_yoke = second_link.get_visual("tip_yoke_right")
    shade_shell = shade.get_visual("shade_shell")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, tilt: 0.0}):
        ctx.expect_contact(
            first_link,
            wall_bracket,
            elem_a=shoulder_barrel,
            elem_b=bracket_ear,
            contact_tol=1e-5,
            name="first link mounts to wall bracket pivot",
        )
        ctx.expect_contact(
            second_link,
            first_link,
            elem_a=elbow_barrel,
            elem_b=elbow_ear,
            contact_tol=1e-5,
            name="second link mounts to elbow pivot",
        )
        ctx.expect_contact(
            shade,
            second_link,
            elem_a=tilt_barrel,
            elem_b=tip_yoke,
            contact_tol=1e-5,
            name="shade mounts into tip yoke",
        )
        ctx.expect_gap(
            shade,
            wall_bracket,
            axis="x",
            min_gap=0.32,
            name="rest pose projects shade outward from bracket",
        )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({shoulder: 0.95, elbow: 0.0, tilt: 0.0}):
        swung_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "shoulder joint swings lamp laterally",
        rest_shade_pos is not None
        and swung_shade_pos is not None
        and swung_shade_pos[1] > rest_shade_pos[1] + 0.22
        and swung_shade_pos[0] < rest_shade_pos[0] - 0.10,
        details=f"rest={rest_shade_pos}, swung={swung_shade_pos}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 1.10, tilt: 0.0}):
        elbowed_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "elbow joint folds the outer link",
        rest_shade_pos is not None
        and elbowed_shade_pos is not None
        and elbowed_shade_pos[1] > rest_shade_pos[1] + 0.15
        and elbowed_shade_pos[0] < rest_shade_pos[0] - 0.05,
        details=f"rest={rest_shade_pos}, elbowed={elbowed_shade_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade, elem=shade_shell)
    with ctx.pose({shoulder: 0.0, elbow: 0.0, tilt: 0.55}):
        tilted_shell_aabb = ctx.part_element_world_aabb(shade, elem=shade_shell)
    ctx.check(
        "shade tilt raises the cone opening",
        rest_shell_aabb is not None
        and tilted_shell_aabb is not None
        and tilted_shell_aabb[1][2] > rest_shell_aabb[1][2] + 0.03,
        details=f"rest_aabb={rest_shell_aabb}, tilted_aabb={tilted_shell_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
