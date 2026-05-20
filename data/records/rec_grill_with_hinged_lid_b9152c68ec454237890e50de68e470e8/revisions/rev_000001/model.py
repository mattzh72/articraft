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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BOWL_RADIUS = 0.280
BOWL_DEPTH = 0.190
LID_RADIUS_INNER = 0.286
LID_RADIUS_OUTER = 0.292
LID_HEIGHT = 0.175
WALL_THICKNESS = 0.004


def _leg_angles() -> tuple[float, float, float]:
    return (math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kettle_charcoal_grill")

    black_enamel = model.material("black_enamel", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.16, 0.13, 0.11, 1.0))

    bowl_outer_profile = [
        (0.032, -BOWL_DEPTH),
        (0.110, -0.176),
        (0.205, -0.110),
        (0.252, -0.050),
        (BOWL_RADIUS, 0.000),
    ]
    bowl_inner_profile = [
        (0.024, -BOWL_DEPTH + WALL_THICKNESS),
        (0.102, -0.170),
        (0.197, -0.104),
        (0.246, -0.046),
        (BOWL_RADIUS - 0.006, -0.004),
    ]
    bowl_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            bowl_outer_profile,
            bowl_inner_profile,
            segments=72,
            lip_samples=10,
        ),
        "kettle_bowl_shell",
    )

    lid_outer_profile = [
        (LID_RADIUS_OUTER, 0.000),
        (0.262, 0.052),
        (0.182, 0.120),
        (0.080, 0.165),
        (0.010, LID_HEIGHT),
    ]
    lid_inner_profile = [
        (LID_RADIUS_INNER, -0.004),
        (0.256, 0.048),
        (0.176, 0.114),
        (0.074, 0.158),
        (0.000, LID_HEIGHT - 0.010),
    ]
    lid_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            lid_outer_profile,
            lid_inner_profile,
            segments=72,
            lip_samples=10,
        ),
        "kettle_lid_shell",
    )

    leg_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.000),
                (0.004, 0.000, -0.060),
                (0.050, 0.000, -0.300),
                (0.100, 0.000, -0.635),
            ],
            radius=0.009,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "tripod_leg_tube",
    )

    bowl = model.part("bowl")
    bowl.visual(bowl_shell, material=black_enamel, name="bowl_shell")

    bracket_radius = 0.205
    bracket_center_z = -0.095
    bracket_size = (0.034, 0.018, 0.050)
    for index, angle in enumerate(_leg_angles(), start=1):
        bowl.visual(
            Box(bracket_size),
            origin=Origin(
                xyz=(
                    bracket_radius * math.cos(angle),
                    bracket_radius * math.sin(angle),
                    bracket_center_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"leg_bracket_{index}",
        )
    lid = model.part("lid")
    lid.visual(
        lid_shell,
        origin=Origin(xyz=(0.000, BOWL_RADIUS, 0.000)),
        material=black_enamel,
        name="lid_shell",
    )
    lid.visual(
        Box((0.060, 0.018, 0.036)),
        origin=Origin(xyz=(0.000, -0.002, 0.018)),
        material=steel,
        name="lid_hinge_tongue",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(
            xyz=(0.000, -0.020, 0.014),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.000, BOWL_RADIUS, LID_HEIGHT - 0.006)),
        material=steel,
        name="lid_vent_boss",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(
            xyz=(-0.070, BOWL_RADIUS + 0.105, 0.142),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=steel,
        name="lid_handle_post_left",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(
            xyz=(0.070, BOWL_RADIUS + 0.105, 0.142),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=steel,
        name="lid_handle_post_right",
    )
    lid.visual(
        Cylinder(radius=0.013, length=0.170),
        origin=Origin(
            xyz=(0.000, BOWL_RADIUS + 0.105, 0.170),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_handle,
        name="lid_handle_bar",
    )

    for index, angle in enumerate(_leg_angles(), start=1):
        leg = model.part(f"leg_{index}")
        leg.visual(leg_mesh, material=steel, name="leg_tube")
        model.articulation(
            f"bowl_to_leg_{index}",
            ArticulationType.FIXED,
            parent=bowl,
            child=leg,
            origin=Origin(
                xyz=(
                    bracket_radius * math.cos(angle),
                    bracket_radius * math.sin(angle),
                    bracket_center_z - bracket_size[2] / 2.0,
                ),
                rpy=(0.0, 0.0, angle),
            ),
        )

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        Cylinder(radius=0.056, length=0.0016),
        origin=Origin(xyz=(0.000, 0.000, 0.0032)),
        material=steel,
        name="vent_plate",
    )
    vent_cap.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=steel,
        name="vent_pivot",
    )
    vent_cap.visual(
        Box((0.022, 0.050, 0.004)),
        origin=Origin(xyz=(0.048, 0.000, 0.0042)),
        material=dark_handle,
        name="vent_tab",
    )

    lid_hinge = model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.000, -BOWL_RADIUS, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.85,
        ),
    )

    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.000, BOWL_RADIUS, LID_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
        meta={"qc_samples": [0.0, math.pi / 3.0, 2.0 * math.pi / 3.0]},
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

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    legs = [object_model.get_part(f"leg_{index}") for index in range(1, 4)]
    lid_hinge = object_model.get_articulation("bowl_to_lid")
    vent_joint = object_model.get_articulation("lid_to_vent_cap")

    lid_limits = lid_hinge.motion_limits
    ctx.check(
        "lid hinge is a rear pitching revolute joint",
        lid_hinge.parent == bowl.name
        and lid_hinge.child == lid.name
        and lid_hinge.axis == (1.0, 0.0, 0.0)
        and lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and lid_limits.upper >= 1.7,
        details=(
            f"parent={lid_hinge.parent}, child={lid_hinge.child}, axis={lid_hinge.axis}, "
            f"limits={lid_limits}"
        ),
    )
    ctx.check(
        "vent cap rotates about the lid crown",
        vent_joint.parent == lid.name
        and vent_joint.child == vent_cap.name
        and vent_joint.axis == (0.0, 0.0, 1.0),
        details=f"parent={vent_joint.parent}, child={vent_joint.child}, axis={vent_joint.axis}",
    )

    for index, leg in enumerate(legs, start=1):
        ctx.expect_contact(
            leg,
            bowl,
            contact_tol=0.004,
            elem_a="leg_tube",
            elem_b=f"leg_bracket_{index}",
            name=f"leg {index} is mounted to the bowl bracket",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            bowl,
            axes="xy",
            elem_a="lid_shell",
            elem_b="bowl_shell",
            min_overlap=0.52,
            name="lid covers the circular bowl opening when closed",
        )
        ctx.expect_gap(
            lid,
            bowl,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="bowl_shell",
            max_gap=0.010,
            max_penetration=0.008,
            name="lid seats at the bowl rim without a large gap",
        )
        ctx.expect_gap(
            vent_cap,
            lid,
            axis="z",
            positive_elem="vent_plate",
            negative_elem="lid_shell",
            min_gap=0.001,
            max_gap=0.012,
            name="vent cap sits just above the lid crown",
        )

    lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: lid_limits.upper if lid_limits and lid_limits.upper is not None else 1.85}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward from the rear rim hinge",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.10,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    vent_closed_center = _aabb_center(ctx.part_element_world_aabb(vent_cap, elem="vent_tab"))
    with ctx.pose({vent_joint: math.pi / 2.0}):
        vent_turned_center = _aabb_center(ctx.part_element_world_aabb(vent_cap, elem="vent_tab"))
    ctx.check(
        "vent tab visibly swings around the pivot",
        vent_closed_center is not None
        and vent_turned_center is not None
        and (
            abs(vent_closed_center[0] - vent_turned_center[0]) > 0.020
            or abs(vent_closed_center[1] - vent_turned_center[1]) > 0.020
        ),
        details=f"closed_center={vent_closed_center}, turned_center={vent_turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
