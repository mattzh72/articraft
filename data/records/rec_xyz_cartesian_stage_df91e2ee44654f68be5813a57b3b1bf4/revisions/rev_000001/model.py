from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.42
BASE_W = 0.28
BASE_T = 0.024
BASE_RAIL_L = 0.34
BASE_RAIL_W = 0.042
BASE_RAIL_H = 0.018
BASE_RAIL_Y = 0.076
BASE_HOLE_R = 0.0055

LOWER_L = 0.30
LOWER_W = 0.19
LOWER_H = 0.050
LOWER_POCKET_L = 0.26
LOWER_POCKET_W = 0.108
LOWER_POCKET_H = 0.020
LOWER_TOP_X = 0.125
LOWER_TOP_Y = 0.170
LOWER_TOP_H = 0.014

CROSS_L = 0.18
CROSS_W = 0.15
CROSS_BASE_H = 0.040
CROSS_PAD_X = 0.110
CROSS_PAD_Y = 0.100
CROSS_PAD_H = 0.010

RAM_X = 0.030
RAM_Y = 0.026
RAM_H = 0.190
RAM_NECK_X = 0.040
RAM_NECK_Y = 0.018
RAM_NECK_H = 0.020
RAM_NECK_CENTER_Y = 0.0
RAM_COLUMN_CENTER_Y = 0.0
RAM_SHOE_X = 0.054
RAM_SHOE_Y = 0.036
RAM_SHOE_H = 0.012
RAM_CAP_X = 0.048
RAM_CAP_Y = 0.032
RAM_CAP_H = 0.016

X_TRAVEL = 0.060
Y_TRAVEL = 0.045
Z_TRAVEL = 0.100

def _base_shape() -> cq.Workplane:
    shape = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))

    rail_z = BASE_T
    for y in (-BASE_RAIL_Y, BASE_RAIL_Y):
        rail = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, y, rail_z))
            .box(BASE_RAIL_L, BASE_RAIL_W, BASE_RAIL_H, centered=(True, True, False))
        )
        shape = shape.union(rail)

    holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.165, -0.105),
                (-0.165, 0.105),
                (0.165, -0.105),
                (0.165, 0.105),
            ]
        )
        .circle(BASE_HOLE_R)
        .extrude(BASE_T + 0.002)
    )
    return shape.cut(holes)


def _lower_carriage_shape() -> cq.Workplane:
    shape = cq.Workplane("XY").box(
        LOWER_L, LOWER_W, LOWER_H, centered=(True, True, False)
    )

    pocket = cq.Workplane("XY").box(
        LOWER_POCKET_L,
        LOWER_POCKET_W,
        LOWER_POCKET_H,
        centered=(True, True, False),
    )
    shape = shape.cut(pocket)

    top_way = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, LOWER_H))
        .box(LOWER_TOP_X, LOWER_TOP_Y, LOWER_TOP_H, centered=(True, True, False))
    )
    return shape.union(top_way)


def _cross_slide_shape() -> cq.Workplane:
    shape = cq.Workplane("XY").box(
        CROSS_L, CROSS_W, CROSS_BASE_H, centered=(True, True, False)
    )

    pad = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, CROSS_BASE_H))
        .box(
            CROSS_PAD_X,
            CROSS_PAD_Y,
            CROSS_PAD_H,
            centered=(True, True, False),
        )
    )
    return shape.union(pad)


def _vertical_ram_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(
        RAM_SHOE_X, RAM_SHOE_Y, RAM_SHOE_H, centered=(True, True, False)
    )

    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, RAM_NECK_CENTER_Y, RAM_SHOE_H))
        .box(RAM_NECK_X, RAM_NECK_Y, RAM_NECK_H, centered=(True, True, False))
    )
    shape = shoe.union(neck)

    stem = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, RAM_COLUMN_CENTER_Y, RAM_SHOE_H + RAM_NECK_H))
        .box(RAM_X, RAM_Y, RAM_H, centered=(True, True, False))
    )
    shape = shape.union(stem)

    cap = (
        cq.Workplane("XY")
        .transformed(
            offset=(0.0, RAM_COLUMN_CENTER_Y, RAM_SHOE_H + RAM_NECK_H + RAM_H)
        )
        .box(RAM_CAP_X, RAM_CAP_Y, RAM_CAP_H, centered=(True, True, False))
    )
    return shape.union(cap)


def _dims_from_aabb(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple(upper[i] - lower[i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_xyz_stage")

    model.material("base_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("machined_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("light_gray", rgba=(0.80, 0.82, 0.85, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "xyz_stage_base"),
        material="base_black",
        name="base_body",
    )

    lower = model.part("lower_carriage")
    lower.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "xyz_stage_lower_carriage"),
        material="machined_gray",
        name="lower_body",
    )

    cross = model.part("cross_slide")
    cross.visual(
        mesh_from_cadquery(_cross_slide_shape(), "xyz_stage_cross_slide"),
        material="machined_gray",
        name="cross_body",
    )

    ram = model.part("vertical_ram")
    ram.visual(
        mesh_from_cadquery(_vertical_ram_shape(), "xyz_stage_vertical_ram"),
        material="light_gray",
        name="ram_body",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + BASE_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=220.0,
            velocity=0.25,
        ),
    )

    model.articulation(
        "lower_to_cross",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=cross,
        origin=Origin(xyz=(0.0, 0.0, LOWER_H + LOWER_TOP_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=180.0,
            velocity=0.22,
        ),
    )

    model.articulation(
        "cross_to_ram",
        ArticulationType.PRISMATIC,
        parent=cross,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, CROSS_BASE_H + CROSS_PAD_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=140.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_carriage")
    cross = object_model.get_part("cross_slide")
    ram = object_model.get_part("vertical_ram")

    joint_x = object_model.get_articulation("base_to_lower")
    joint_y = object_model.get_articulation("lower_to_cross")
    joint_z = object_model.get_articulation("cross_to_ram")

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

    ctx.expect_contact(lower, base, name="lower carriage is supported on the base rails")
    ctx.expect_contact(
        cross, lower, name="cross slide is supported on the lower carriage"
    )
    ctx.expect_contact(ram, cross, name="vertical ram is seated in the upper guide")

    ctx.expect_overlap(
        lower,
        base,
        axes="xy",
        min_overlap=0.18,
        name="lower carriage stays over the grounded base footprint",
    )
    ctx.expect_overlap(
        cross,
        lower,
        axes="xy",
        min_overlap=0.12,
        name="cross slide stays over the lower carriage footprint",
    )

    base_dims = _dims_from_aabb(ctx.part_world_aabb(base))
    lower_dims = _dims_from_aabb(ctx.part_world_aabb(lower))
    cross_dims = _dims_from_aabb(ctx.part_world_aabb(cross))
    ram_dims = _dims_from_aabb(ctx.part_world_aabb(ram))
    silhouette_ok = (
        base_dims is not None
        and lower_dims is not None
        and cross_dims is not None
        and ram_dims is not None
        and base_dims[0] > lower_dims[0] > cross_dims[0]
        and base_dims[1] > lower_dims[1] > ram_dims[1]
        and ram_dims[2] > ram_dims[0] * 3.0
    )
    ctx.check(
        "stage silhouette proportions",
        bool(silhouette_ok),
        details=(
            f"base={base_dims}, lower={lower_dims}, "
            f"cross={cross_dims}, ram={ram_dims}"
        ),
    )

    lower_rest = ctx.part_world_position(lower)
    with ctx.pose({joint_x: 0.050}):
        lower_pos = ctx.part_world_position(lower)
    ctx.check(
        "lower carriage moves in +x",
        lower_rest is not None
        and lower_pos is not None
        and lower_pos[0] > lower_rest[0] + 0.045,
        details=f"rest={lower_rest}, moved={lower_pos}",
    )

    cross_rest = ctx.part_world_position(cross)
    with ctx.pose({joint_y: 0.035}):
        cross_pos = ctx.part_world_position(cross)
    ctx.check(
        "cross slide moves in +y",
        cross_rest is not None
        and cross_pos is not None
        and cross_pos[1] > cross_rest[1] + 0.030,
        details=f"rest={cross_rest}, moved={cross_pos}",
    )

    ram_rest = ctx.part_world_position(ram)
    with ctx.pose({joint_z: 0.080}):
        ram_pos = ctx.part_world_position(ram)
    ctx.check(
        "vertical ram moves in +z",
        ram_rest is not None
        and ram_pos is not None
        and ram_pos[2] > ram_rest[2] + 0.075,
        details=f"rest={ram_rest}, moved={ram_pos}",
    )

    with ctx.pose({joint_x: 0.050, joint_y: -0.035, joint_z: 0.080}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="representative displaced pose remains clear"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
