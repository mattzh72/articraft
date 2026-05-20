from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_TOP_Z = 0.052
YAW_LIMIT = 3.14159
PITCH_LOWER = -0.75
PITCH_UPPER = 1.15
HALF_PI = 1.57079632679


def make_base_stage() -> cq.Workplane:
    plinth = cq.Workplane("XY").box(0.180, 0.160, 0.024).translate((0.0, 0.0, 0.012))
    pedestal = cq.Workplane("XY").box(0.110, 0.095, 0.020).translate((0.0, 0.0, 0.034))
    bearing_ring = cq.Workplane("XY").circle(0.070).extrude(0.008).translate((0.0, 0.0, 0.044))
    center_boss = cq.Workplane("XY").circle(0.026).extrude(0.010).translate((0.0, 0.0, 0.042))
    return plinth.union(pedestal).union(bearing_ring).union(center_boss)


def _y_cylinder(radius: float, length: float, start_y: float, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(x, start_y, z),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )


def _make_fork_arm(y_center: float) -> cq.Workplane:
    arm = cq.Workplane("XY").box(0.018, 0.012, 0.068).translate((0.000, y_center, 0.126))
    root = cq.Workplane("XY").box(0.026, 0.012, 0.026).translate((-0.006, y_center, 0.102))
    boss = _y_cylinder(0.018, 0.012, y_center - 0.006, x=0.0, z=0.145)
    bore = _y_cylinder(0.010, 0.016, y_center - 0.008, x=0.0, z=0.145)
    return arm.union(root).union(boss).cut(bore)


def make_fork_stage() -> cq.Workplane:
    turntable = cq.Workplane("XY").circle(0.068).extrude(0.010)
    column = cq.Workplane("XY").box(0.036, 0.072, 0.086).translate((-0.024, 0.0, 0.053))
    crosshead = cq.Workplane("XY").box(0.020, 0.100, 0.018).translate((-0.008, 0.0, 0.101))
    rear_spine = cq.Workplane("XY").box(0.014, 0.092, 0.048).translate((-0.038, 0.0, 0.069))
    left_arm = _make_fork_arm(0.046)
    right_arm = _make_fork_arm(-0.046)
    return turntable.union(column).union(crosshead).union(rear_spine).union(left_arm).union(right_arm)


def make_pitch_cradle() -> cq.Workplane:
    main_body = cq.Workplane("XY").box(0.044, 0.034, 0.034).translate((0.042, 0.0, 0.0))
    neck = cq.Workplane("XY").box(0.024, 0.024, 0.024).translate((0.012, 0.0, 0.0))
    top_plate = cq.Workplane("XY").box(0.034, 0.026, 0.006).translate((0.042, 0.0, 0.020))
    front_lip = cq.Workplane("XY").box(0.010, 0.026, 0.012).translate((0.064, 0.0, -0.008))
    lower_pad = cq.Workplane("XY").box(0.024, 0.020, 0.010).translate((0.038, 0.0, -0.018))
    hub_barrel = _y_cylinder(0.014, 0.040, -0.020, x=0.0, z=0.0)
    left_stub = _y_cylinder(0.0095, 0.020, 0.020, x=0.0, z=0.0)
    right_stub = cq.Workplane(
        obj=cq.Solid.makeCylinder(
            0.0095,
            0.020,
            cq.Vector(0.0, -0.020, 0.0),
            cq.Vector(0.0, -1.0, 0.0),
        )
    )
    top_slot = (
        cq.Workplane("XY")
        .center(0.042, 0.0)
        .slot2D(0.018, 0.008, 0.0)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.012))
    )
    cradle = main_body.union(neck).union(top_plate).union(front_lip).union(lower_pad).union(hub_barrel).union(left_stub).union(right_stub)
    return cradle.cut(top_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_fork_yaw_pitch_head")

    base_mat = model.material("base_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    fork_mat = model.material("fork_graphite", rgba=(0.27, 0.29, 0.31, 1.0))
    cradle_mat = model.material("cradle_alloy", rgba=(0.73, 0.75, 0.78, 1.0))

    base = model.part("base_stage")
    base.visual(
        mesh_from_cadquery(make_base_stage(), "base_stage"),
        material=base_mat,
        name="base_stage_shell",
    )

    fork = model.part("fork_stage")
    fork.visual(
        Cylinder(radius=0.068, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=fork_mat,
        name="turntable_disk",
    )
    fork.visual(
        Box((0.036, 0.072, 0.086)),
        origin=Origin(xyz=(-0.024, 0.0, 0.053)),
        material=fork_mat,
        name="rear_column",
    )
    fork.visual(
        Box((0.020, 0.100, 0.018)),
        origin=Origin(xyz=(-0.008, 0.0, 0.101)),
        material=fork_mat,
        name="crosshead",
    )
    for side_name, side_y in (("left", 0.046), ("right", -0.046)):
        fork.visual(
            Box((0.026, 0.012, 0.026)),
            origin=Origin(xyz=(-0.006, side_y, 0.104)),
            material=fork_mat,
            name=f"{side_name}_arm_root",
        )
        fork.visual(
            Box((0.018, 0.012, 0.068)),
            origin=Origin(xyz=(0.0, side_y, 0.130)),
            material=fork_mat,
            name=f"{side_name}_arm_leg",
        )
        fork.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, side_y, 0.145), rpy=(HALF_PI, 0.0, 0.0)),
            material=fork_mat,
            name=f"{side_name}_pivot_boss",
        )

    cradle = model.part("pitch_cradle")
    cradle.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=cradle_mat,
        name="pivot_barrel",
    )
    cradle.visual(
        Box((0.024, 0.024, 0.024)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=cradle_mat,
        name="pivot_neck",
    )
    cradle.visual(
        Box((0.044, 0.034, 0.034)),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=cradle_mat,
        name="cradle_body",
    )
    cradle.visual(
        Box((0.034, 0.026, 0.006)),
        origin=Origin(xyz=(0.042, 0.0, 0.020)),
        material=cradle_mat,
        name="top_plate",
    )
    cradle.visual(
        Box((0.010, 0.026, 0.012)),
        origin=Origin(xyz=(0.064, 0.0, -0.008)),
        material=cradle_mat,
        name="front_lip",
    )
    cradle.visual(
        Box((0.024, 0.020, 0.010)),
        origin=Origin(xyz=(0.038, 0.0, -0.018)),
        material=cradle_mat,
        name="lower_pad",
    )

    model.articulation(
        "base_to_fork_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )

    model.articulation(
        "fork_to_cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_stage")
    fork = object_model.get_part("fork_stage")
    cradle = object_model.get_part("pitch_cradle")
    yaw = object_model.get_articulation("base_to_fork_yaw")
    pitch = object_model.get_articulation("fork_to_cradle_pitch")

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
        "yaw_axis_is_vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        f"expected vertical yaw axis, got {yaw.axis}",
    )
    ctx.check(
        "pitch_axis_is_horizontal",
        tuple(pitch.axis) == (0.0, -1.0, 0.0),
        f"expected fork-supported horizontal pitch axis, got {pitch.axis}",
    )

    ctx.expect_contact(fork, base, name="fork_turntable_supported_by_base")
    ctx.expect_contact(cradle, fork, name="cradle_supported_by_fork_arms")
    ctx.expect_within(cradle, fork, axes="y", margin=0.0015, name="cradle_stays_between_fork_width")
    ctx.expect_gap(cradle, base, axis="z", min_gap=0.045, name="cradle_clears_lower_stage")

    with ctx.pose({yaw: 1.2}):
        ctx.expect_contact(fork, base, name="yaw_support_contact_persists")

    with ctx.pose({pitch: PITCH_UPPER}):
        ctx.expect_contact(cradle, fork, name="pitch_support_contact_persists")

    with ctx.pose({pitch: PITCH_LOWER}):
        lower_aabb = ctx.part_world_aabb(cradle)
    with ctx.pose({pitch: PITCH_UPPER}):
        upper_aabb = ctx.part_world_aabb(cradle)

    if lower_aabb is None or upper_aabb is None:
        ctx.fail("pitch_pose_bounds_resolve", "could not resolve cradle AABB in pitch test poses")
    else:
        lower_max_z = lower_aabb[1][2]
        upper_max_z = upper_aabb[1][2]
        ctx.check(
            "positive_pitch_lifts_cradle",
            upper_max_z > lower_max_z + 0.035,
            f"expected upper pose to lift cradle noticeably: lower max z={lower_max_z:.4f}, upper max z={upper_max_z:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
