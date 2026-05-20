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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("walnut", rgba=(0.33, 0.20, 0.12, 1.0))
    suede = model.material("suede", rgba=(0.66, 0.58, 0.47, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.35, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    cushion_fabric = model.material("cushion_fabric", rgba=(0.24, 0.22, 0.20, 1.0))

    body_width = 0.19
    body_depth = 0.16
    body_height = 0.095
    shell_wall = 0.014

    lid_width = 0.212
    lid_depth = 0.182
    lid_height = 0.052
    lid_wall = 0.009
    lid_overlap = 0.028

    spindle_y = 0.053
    spindle_z = 0.049

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, shell_wall)),
        origin=Origin(xyz=(0.0, 0.0, shell_wall / 2.0)),
        material=walnut,
        name="base_floor",
    )
    body.visual(
        Box((shell_wall, body_depth, body_height)),
        origin=Origin(xyz=(-(body_width / 2.0) + (shell_wall / 2.0), 0.0, body_height / 2.0)),
        material=walnut,
        name="left_wall",
    )
    body.visual(
        Box((shell_wall, body_depth, body_height)),
        origin=Origin(xyz=((body_width / 2.0) - (shell_wall / 2.0), 0.0, body_height / 2.0)),
        material=walnut,
        name="right_wall",
    )
    body.visual(
        Box((body_width - (2.0 * shell_wall) + 0.006, shell_wall, body_height)),
        origin=Origin(xyz=(0.0, -(body_depth / 2.0) + (shell_wall / 2.0), body_height / 2.0)),
        material=walnut,
        name="front_wall",
    )
    body.visual(
        Box((body_width - (2.0 * shell_wall) + 0.006, shell_wall, body_height)),
        origin=Origin(xyz=(0.0, (body_depth / 2.0) - (shell_wall / 2.0), body_height / 2.0)),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((0.156, 0.126, 0.004)),
        origin=Origin(xyz=(0.0, -0.005, shell_wall + 0.002)),
        material=suede,
        name="base_liner",
    )
    body.visual(
        Box((0.116, 0.074, 0.018)),
        origin=Origin(xyz=(0.0, -0.045, shell_wall + 0.009)),
        material=suede,
        name="presentation_plinth",
    )
    body.visual(
        Box((0.084, 0.026, 0.052)),
        origin=Origin(xyz=(0.0, (body_depth / 2.0) + 0.005, 0.050)),
        material=dark_metal,
        name="rear_drive_pod",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(
            xyz=(0.0, spindle_y + 0.008, spindle_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="spindle_stub",
    )
    body.visual(
        Box((0.148, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.076, 0.102)),
        material=brass,
        name="hinge_rest",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, 0.010)),
        origin=Origin(xyz=(0.0, -(lid_depth / 2.0), -0.005)),
        material=walnut,
        name="lid_top",
    )
    lid.visual(
        Box((lid_wall, lid_depth, lid_height)),
        origin=Origin(xyz=(-(lid_width / 2.0) + (lid_wall / 2.0), -(lid_depth / 2.0), -(lid_height / 2.0))),
        material=walnut,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_wall, lid_depth, lid_height)),
        origin=Origin(xyz=((lid_width / 2.0) - (lid_wall / 2.0), -(lid_depth / 2.0), -(lid_height / 2.0))),
        material=walnut,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_width - (2.0 * lid_wall) + 0.004, lid_wall, lid_height)),
        origin=Origin(xyz=(0.0, -lid_depth + (lid_wall / 2.0), -(lid_height / 2.0))),
        material=walnut,
        name="front_rail",
    )
    lid.visual(
        Box((0.176, 0.120, 0.004)),
        origin=Origin(xyz=(0.0, -0.100, -0.008)),
        material=suede,
        name="lid_liner",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_height)),
        mass=1.0,
        origin=Origin(xyz=(0.0, -(lid_depth / 2.0), -(lid_height / 2.0))),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub_journal",
    )
    cradle.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="backing_disc",
    )
    cradle.visual(
        Box((0.074, 0.036, 0.058)),
        origin=Origin(xyz=(0.0, -0.042, 0.0)),
        material=cushion_fabric,
        name="cushion_block",
    )
    cradle.visual(
        Cylinder(radius=0.014, length=0.074),
        origin=Origin(xyz=(0.0, -0.042, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cushion_fabric,
        name="upper_roll",
    )
    cradle.visual(
        Cylinder(radius=0.014, length=0.074),
        origin=Origin(xyz=(0.0, -0.042, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cushion_fabric,
        name="lower_roll",
    )
    cradle.visual(
        Box((0.026, 0.008, 0.040)),
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
        material=brass,
        name="retainer_bar",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.084, 0.070, 0.074)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.038, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(
                0.0,
                (body_depth / 2.0) + ((lid_depth - body_depth) / 2.0),
                body_height - lid_overlap + lid_height,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, spindle_y, spindle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
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
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    with ctx.pose({lid_hinge: 0.0, cradle_spin: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.155,
            name="closed lid covers the body footprint",
        )
        ctx.expect_contact(
            cradle,
            body,
            elem_a="hub_journal",
            elem_b="spindle_stub",
            name="cradle is mounted on the spindle stub",
        )
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            margin=0.0,
            name="cradle stays inside the body opening",
        )
        closed_front_rail = ctx.part_element_world_aabb(lid, elem="front_rail")
        closed_cushion = ctx.part_element_world_aabb(cradle, elem="cushion_block")

    with ctx.pose({lid_hinge: 1.0}):
        open_front_rail = ctx.part_element_world_aabb(lid, elem="front_rail")

    ctx.check(
        "lid front edge lifts clear when opened",
        closed_front_rail is not None
        and open_front_rail is not None
        and open_front_rail[0][2] > closed_front_rail[0][2] + 0.070,
        details=f"closed={closed_front_rail}, open={open_front_rail}",
    )

    with ctx.pose({cradle_spin: math.pi / 2.0}):
        turned_cushion = ctx.part_element_world_aabb(cradle, elem="cushion_block")
        ctx.expect_contact(
            cradle,
            body,
            elem_a="hub_journal",
            elem_b="spindle_stub",
            name="hub stays seated on the spindle while spinning",
        )

    closed_cushion_width = None
    closed_cushion_height = None
    turned_cushion_width = None
    turned_cushion_height = None
    if closed_cushion is not None:
        closed_cushion_width = closed_cushion[1][0] - closed_cushion[0][0]
        closed_cushion_height = closed_cushion[1][2] - closed_cushion[0][2]
    if turned_cushion is not None:
        turned_cushion_width = turned_cushion[1][0] - turned_cushion[0][0]
        turned_cushion_height = turned_cushion[1][2] - turned_cushion[0][2]

    ctx.check(
        "cradle visibly rotates about the spindle axis",
        closed_cushion_width is not None
        and closed_cushion_height is not None
        and turned_cushion_width is not None
        and turned_cushion_height is not None
        and turned_cushion_width < closed_cushion_width - 0.010
        and turned_cushion_height > closed_cushion_height + 0.010,
        details=(
            f"closed_width={closed_cushion_width}, closed_height={closed_cushion_height}, "
            f"turned_width={turned_cushion_width}, turned_height={turned_cushion_height}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
